/*
 * TouchScreen.hpp
 *
 *  Created on: Jun 30, 2024
 *      Author: KHOA
 */

#ifndef INC_TOUCHSCREEN_HPP_
#define INC_TOUCHSCREEN_HPP_
#include "main.h"
#define NUMSAMPLES 2
/** Object that encapsulates the X,Y, and Z/pressure measurements for a touch
 * event. */
class TSPoint {
	public:
		TSPoint(void) {
			x = y = z = 0;
		}
		/**
		 * @brief Construct a new TSPoint::TSPoint object
		 *
		 * @param x0 The point's X value
		 * @param y0 The point's Y value
		 * @param z0 The point's Z value
		 */
		TSPoint(int16_t x0, int16_t y0, int16_t z0) {
			x = x0;
			y = y0;
			z = z0;
		}

		bool operator==(TSPoint);
		bool operator!=(TSPoint);

		int16_t x, ///< state variable for the x value
				y,     ///< state variable for the y value
				z;     ///< state variable for the z value
};
/** Object that controls and keeps state for a touch screen. */

class TouchScreen {
	public:
		/**
		 * @brief Construct a new Touch Screen object
		 *
		 * @param xpPin X+ pin. Must be an analog pin
		 * @param ypPin Y+ pin. Must be an analog pin
		 * @param xmPin X- pin. Can be a digital pin
		 * @param ymPin Y- pin. Can be a digital pin
		 * @param rx The resistance in ohms between X+ and X- to calibrate pressure
		 * sensing
		 *
		 */
//@formatter:off
		TouchScreen(uint16_t rxplate = 300, uint8_t xmRankADC = 2, uint8_t ypRankADC = 1)
							:_rxplate(rxplate), _xmRank(xmRankADC), _ypRank(ypRankADC)
		{}
		bool IsTouchDisable() {return _disableTouch;}
		void SaveTouchPoint(TSPoint ts){
			_x = ts.x;
			_y = ts.y;
		}
//@formatter:on
		void Begin(ADC_TypeDef *adc) {
			_adc = adc;
			_hadc.Instance = adc;
			// Must enable ADC CLK
			if (adc == ADC1) {
				__HAL_RCC_ADC1_CLK_ENABLE();
			}
			else if (adc == ADC2) {
				__HAL_RCC_ADC2_CLK_ENABLE();
			}
			else if (adc == ADC3) {
				__HAL_RCC_ADC3_CLK_ENABLE();
			}
			ADC_Common_TypeDef *tmpADC_Common;
			tmpADC_Common = ADC_COMMON_REGISTER(&_hadc);
			tmpADC_Common->CCR &= ~(ADC_CCR_ADCPRE);
			tmpADC_Common->CCR |= ADC_CLOCK_SYNC_PCLK_DIV8;
			/* Set ADC resolution */
			_adc->CR1 &= ~ADC_CR1_RES;
			_adc->CR1 |= ADC_RESOLUTION12b;
			/* Set ADC data alignment */
			_adc->CR2 &= ~(ADC_CR2_ALIGN);
			_adc->CR2 |= ADC_DATAALIGN_RIGHT;
			/* Enable or disable ADC end of conversion selection */
			_adc->CR2 &= ~(ADC_CR2_EOCS);
			_adc->CR2 |= ADC_CR2_EOCSelection(ADC_EOC_SINGLE_CONV);
		}

		uint16_t ReadTouchPressure() {
			SetupReadPressure();
			return (uint16_t) GetTouchPressure();

		}

		void TouchOff() {
			_disableTouch = true;
			ConfigOutputPin(_xmPort, _xmPin);
			ConfigOutputPin(_ypPort, _ypPin);
			ConfigOutputPin(_xpPort, _xpPin);
			ConfigOutputPin(_ymPort, _ymPin);
		}

		void TouchOn() {
			_disableTouch = false;
			GetPoint(); // skip garbage value before actually read
		}

		int ReadTouchX() {
			int value[1] = {0};
			SetupReadTouchX();
			GetSampleADC(value, sizeof(value), 1);
			return 4096 - value[0];
		}

		int ReadTouchY() {
			int value[1] = {0};
			SetupReadTouchY();
			GetSampleADC(value, sizeof(value), 1);
			return 4096 - value[0];
		}

		TSPoint GetPoint() {
			if (_disableTouch) return TSPoint(_x, _y, _z);
			int16_t x = 0, y = 0, z = 0;
			int samples[NUMSAMPLES];
			uint8_t valid;
			valid = 1;

			SetupReadTouchX();
			GetSampleADC(samples, sizeof(samples), NUMSAMPLES);

#if NUMSAMPLES > 2
  insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
			// Allow small amount of measurement noise, because capacitive
			// coupling to a TFT display's signals can induce some noise.
			if (samples[0] - samples[1] < -25 || samples[0] - samples[1] > 25) {
				valid = 0;
			}
			else {
				samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
			}
#endif
			_x = x = (4096 - samples[NUMSAMPLES / 2]);

			SetupReadTouchY();
			GetSampleADC(samples, sizeof(samples), NUMSAMPLES);

#if NUMSAMPLES > 2
  insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
			// Allow small amount of measurement noise, because capacitive
			// coupling to a TFT display's signals can induce some noise.
			if (samples[0] - samples[1] < -25 || samples[0] - samples[1] > 25) {
				valid = 0;
			}
			else {
				samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
			}
#endif
			_y = y = (4096 - samples[NUMSAMPLES / 2]);
			SetupReadPressure();
			_z = z = (int16_t) GetTouchPressure();
			if (!valid) {
				z = 0;
			}
			HAL_Delay(100);
			return TSPoint(x, y, z);
		}
	protected:
		void ConfigInputPin(GPIO_TypeDef *port, uint16_t pin) {
			uint8_t position = 0;
			while (!(pin & 0x01)) {
				position++;
				pin >>= 1;
			}
			port->MODER &= ~(3 << (position * 2));
		}
		void ConfigOutputPin(GPIO_TypeDef *port, uint16_t pin) {
			uint8_t position = 0;
			while (!(pin & 0x01)) {
				position++;
				pin >>= 1;
			}
			port->BSRR = 1 << (position + 16); // reset output pin to low
			port->MODER &= ~(3 << (position * 2)); // clear config to default value
			port->MODER |= (1 << (position * 2)); // output push-pull
			port->PUPDR &= ~(3 << (position * 2)); // no pull-up pull-down
			port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
			port->OSPEEDR |= GPIO_SPEED_FREQ_LOW << (position * 2U);
		}
		void AllowToGetSampleADC(bool isAllow) {
			if ((_adc->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON && isAllow) {
				_adc->CR2 |= ADC_CR2_ADON;
				/* Delay for ADC stabilization time */
				/* Compute number of CPU cycles to wait for */
				uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
				while (counter != 0U) {
					counter--;
				}
			}
			else {
				_adc->CR2 &= ~ADC_CR2_ADON;

			}
		}

	private:
		void ConfigAnalogXM() {
			// Config analog
			GPIOA->MODER |= (3 << (4 * 2)); //PA4
			GPIOA->PUPDR &= ~(3 << (4 * 2));

		}
		void ConfigAnalogYP() {
			// Config analog
			GPIOB->MODER |= (3 << (0 * 2)); //PB0
			GPIOB->PUPDR &= ~(3 << (0 * 2));
		}
		void SetupReadTouchX() {
			// Sample YP (PB0) only
			// Set rank YP to 1 and ADC get sample this channel only
			_ypRank = 1;
			_adc->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, _ypRank);
			_adc->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_8, _ypRank);
			// Select one of two method below
			// method1: stop scan groups for sampling
			_adc->CR1 &= ~(ADC_CR1_SCAN);
			// method2: set num of conversion to 1
//			_adc->SQR1 &= ~(ADC_SQR1_L);
//			_adc->SQR1 |= ADC_SQR1(1);
			ConfigAnalogYP();
			ConfigInputPin(_ymPort, _ymPin);

			ConfigOutputPin(_xpPort, _xpPin);
			ConfigOutputPin(_xmPort, _xmPin);
			_xpPort->BSRR = _xpPin;
		}

		void SetupReadTouchY() {
			// Sample X- (PA4) only
			// Set rank X- to 1 and ADC get sample this channel only
			_xmRank = 1;
			_adc->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, _xmRank);
			_adc->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_4, _xmRank);
			// Select one of two method below
			// method1: stop scan groups for sampling
			_adc->CR1 &= ~(ADC_CR1_SCAN);
			// method2: set num of conversion to 1
//			_adc->SQR1 &= ~(ADC_SQR1_L);
//			_adc->SQR1 |= ADC_SQR1(1);
			ConfigAnalogXM();
			ConfigInputPin(_xpPort, _xpPin);

			ConfigOutputPin(_ypPort, _ypPin);
			ConfigOutputPin(_ymPort, _ymPin);
			_ypPort->BSRR = _ypPin; // output high Y+, low Y-
		}

		void SetupReadPressure() {
			if (!_adc) while (1);
			ConfigOutputPin(_xpPort, _xpPin);
			ConfigOutputPin(_ymPort, _ymPin);
			_ymPort->BSRR = _ymPin; //Set Y- to VCC and X+ is default to GND
			ConfigAnalogXM();
			ConfigAnalogYP();
			// Clear and set number of conversion ADC
			_adc->SQR1 &= ~(ADC_SQR1_L);
			_adc->SQR1 |= ADC_SQR1(2);
			/* Set ADC scan mode */
			_adc->CR1 |= ADC_CR1_SCANCONV(ENABLE);
			_xmRank = 1;
			_adc->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, _xmRank);
			_adc->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_4, _xmRank);
			_ypRank = 2;
			_adc->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, _ypRank);
			_adc->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_8, _ypRank);
		}

		uint32_t GetTouchPressure() {
			AllowToGetSampleADC(true);
			if ((_adc->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON) {
				_adc->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
				_adc->CR2 |= (uint32_t) ADC_CR2_SWSTART;
			}
			while ((_adc->SR & ADC_SR_EOC) != ADC_SR_EOC);
			uint32_t z1 = _adc->DR;
			while ((_adc->SR & ADC_SR_EOC) != ADC_SR_EOC);
			uint32_t z2 = _adc->DR;
			AllowToGetSampleADC(false);
			if (_rxplate != 0) {
				// now read the x
				float rtouch;
				rtouch = z2;
				rtouch /= z1;
				rtouch -= 1;
				rtouch *= ReadTouchX();
				rtouch *= _rxplate;
				rtouch /= 1024;
				return (uint32_t) rtouch;
			}
			else {
				return (4096 - (z2 - z1));
			}
		}

		void GetSampleADC(int *sampleBuffer, uint8_t sizeOfBuffer, uint8_t numOfSample) {
			if (sampleBuffer == NULL || (sizeOfBuffer / sizeof(int)) < numOfSample) return;
			uint8_t i = 0;
			AllowToGetSampleADC(true);
			while (numOfSample-- > 0) {
				if ((_adc->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON) {
					_adc->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
					_adc->CR2 |= (uint32_t) ADC_CR2_SWSTART;
				}
				while ((_adc->SR & ADC_SR_EOC) != ADC_SR_EOC);
				sampleBuffer[i++] = _adc->DR;
			}
			AllowToGetSampleADC(false);
		}

		static void InsertSort(int array[], uint8_t size) {
			uint8_t j;
			int save;

			for (int i = 1; i < size; i++) {
				save = array[i];
				for (j = i; j >= 1 && save < array[j - 1]; j--)
					array[j] = array[j - 1];
				array[j] = save;
			}
		}
//@formatter:off
		ADC_HandleTypeDef _hadc;
		ADC_TypeDef *_adc;
		uint16_t _rxplate;
		uint8_t _xmRank;
		uint8_t _ypRank;
		bool _test;
		int16_t _x,_y,_z;
		bool _disableTouch = false;
		// Use LCD_D0 (PA9) to read digital pin
		GPIO_TypeDef *_xpPort = GPIOA;uint16_t _xpPin = GPIO_PIN_9;
		// Use LCD_CS (PB0) to read analog pin
		GPIO_TypeDef *_ypPort = GPIOB;uint16_t _ypPin = GPIO_PIN_0;
		// Use LCD_CD(PA4) to read analog pin
		GPIO_TypeDef *_xmPort = GPIOA;uint16_t _xmPin = GPIO_PIN_4;
		// Use LCD_D1 (PC7) to read digital pin
		GPIO_TypeDef *_ymPort = GPIOC;uint16_t _ymPin = GPIO_PIN_7;
//@formatter:on
};

#endif /* INC_TOUCHSCREEN_HPP_ */
