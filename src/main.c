/**
 ******************************************************************************
 * @file    research/main.c
 * @author  Radley Scott
 * @date    15012019
 * @brief	Main function for sensor board functionality
 ******************************************************************************
 */

/* ----------------------------------------------------------------------
** Includes
** ------------------------------------------------------------------- */
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "arm_math.h"
#include "board.h"

/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */
#define FFT_SAMPLES				2048
#define FFT_SIZE				1024
#define BLOCK_SIZE            	32
#define NUM_TAPS              	21
#define OFF						0
#define	VIBE_IN_PROGRESS		1
#define	VIBE_DONE				2
#define	VIBE_FFT				3

/* ----------------------------------------------------------------------
** Global Variables
** ------------------------------------------------------------------- */
float32_t fftInput[FFT_SAMPLES];
float32_t fftOutput[FFT_SIZE];
uint8_t vibrationOutput[105];
float32_t filtOutput[FFT_SIZE];
float32_t filtInput[FFT_SIZE];
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FFT_SIZE/BLOCK_SIZE;
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
uint8_t measureMode = OFF;
uint16_t sample = 0;
//FIR filter coefficients calculated in MATLAB using fir1(20, 2kHz/10kHz/2)
const float32_t firCoeffs32[NUM_TAPS] = {
		0.0, -0.00212227114882539, -0.00632535399151418, -0.0116118103776210, -0.0123546567489824,
		0.0, 0.0317744975585673, 0.0814359075642177, 0.137493781701943, 0.182125490388735,
		0.199168830106960, 0.182125490388735, 0.137493781701943, 0.0814359075642177, 0.0317744975585673,
		0.0, -0.0123546567489824, -0.0116118103776210, -0.00632535399151418, -0.00212227114882539,
		0.0
};

/* ----------------------------------------------------------------------
** Function Prototypes
** ------------------------------------------------------------------- */
void process_measurements(void);
void vibe_fft(void);
void vibe_filter(void);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {

	BRD_init();
	ADC_init();

	for(;;) {

		if (BRD_button_pushed()) {

			process_measurements();
		}

		/* Heartbeat*/
		BRD_delay(50);
		BRD_led_toggle();
	}
}

/**
  * @brief  Processes steps to capture all measurements
  * @param  None
  * @retval None
  */
void process_measurements(void) {

	/* Initialise timer for vibration sensor */
	if (measureMode == OFF) {
		TIM2_Init();
		measureMode = VIBE_IN_PROGRESS;
		return;
	}
	/* Capturing vibrations */
	if (measureMode == VIBE_IN_PROGRESS) {
		return;
	}
	/* Deinitialise timer for vibration sensor */
	if (measureMode == VIBE_DONE) {
		TIM2_Deinit();
		measureMode = VIBE_FFT;
		return;
	}
	/* Filter and FFT of captured data */
	if (measureMode == VIBE_FFT) {
		vibe_filter();
		vibe_fft();
		measureMode = OFF;
	}

	debug_printf("START\n\r");
	for (int i = 0; i < 105; i++) {
		debug_printf("%d, ", vibrationOutput[i]);
	}
	debug_printf("DONE\n\r");
	BRD_button_unpush();
	return;
}


/**
  * @brief  Filters captured data - LPF: 2kHz
  * @param  None
  * @retval None
  */
void vibe_filter(void) {

	arm_fir_instance_f32 Filt;
	float32_t  *inputF32, *outputF32;

	/* DC offset from ADC data */
	for (int i = 0; i < SAMPLES; i++) {
	  filtInput[i] = (float32_t)ADC_get_buffer(i) - 127;
	}

	/* Initialize input and output buffer pointers */
	inputF32 = &filtInput[0];
	outputF32 = &filtOutput[0];

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&Filt, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

	/* Call the FIR process function for every blockSize samples */
	for(uint32_t i=0; i < numBlocks; i++) {
		arm_fir_f32(&Filt, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
}


/**
  * @brief  Performs Fast Fourier Transform of vibration sensor data
  * @param  None
  * @retval None
  */
void vibe_fft(void) {

	arm_cfft_radix4_instance_f32 Signal;	/* ARM CFFT module */
	float32_t maxValue;				/* Max FFT value is stored here */
	uint32_t maxIndex;				/* Index in Output array where max value is */

	/* Change to complex form of ADC data */
	for (uint16_t i = 0; i < FFT_SAMPLES; i += 2) {
		/* Real part */
		fftInput[(uint16_t)i] = filtOutput[((uint16_t)i)/2];
		/* Imaginary part */
		fftInput[(uint16_t)(i + 1)] = 0;
	}

	/* Initialize the CFFT/CIFFT module */
	arm_cfft_radix4_init_f32(&Signal, FFT_SIZE, 0, 1);

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&Signal, fftInput);

	/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fftInput, fftOutput, FFT_SIZE);

	/* Set up package of 100 samples*/
	for (int i = 0; i < 105; i++) {
		vibrationOutput[i] = (uint8_t)(fftOutput[i]/256);
	}
	/* Remove DC - 50Hz frequency content */
	for (int i = 0; i < 5; i++) {
		vibrationOutput[i] = 0;
	}

	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(fftOutput, FFT_SIZE, &maxValue, &maxIndex);
}


/**
* @brief Period elapsed callback in non blocking mode
* @param htim: Pointer to a TIM_HandleTypeDef that contains the configuration information for the TIM module.
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		/* End ADC sampling */
		if (sample >= SAMPLES) {
			measureMode = VIBE_DONE;
			sample = 0;
			return;
		}

		if (measureMode != VIBE_DONE) {
			/* Capture ADC samples */
			ADC_fill_buffer(sample);
			sample++;
		}
	}
}
