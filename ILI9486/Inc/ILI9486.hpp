/*
 * ILI9486.hpp
 *
 *  Created on: Jun 25, 2024
 *      Author: KHOA
 */

// Only use for TFT 3.5 inch 8080 8 bit bus interface
#ifndef INC_ILI9486_HPP_
#define INC_ILI9486_HPP_
#include "main.h"
#include "algorithm"
#include "Adafruit_GFX.h"
#include "TouchScreen.hpp"
#define TFT_WIDTH 480
#define TFT_HEIGHT 320

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// PA4
#define TFT_CD_DATA (GPIOA->BSRR = 1 << 4)
#define TFT_CD_COMMAND (GPIOA->BSRR = 1 << 20)
// PB0
#define TFT_CS_IDLE (GPIOB->BSRR = 1 << 0)
#define TFT_CS_ACTIVE (GPIOB->BSRR = 1 << 16)
// PC1
#define TFT_RESET_IDLE (GPIOC->BSRR = 1 << 1)
#define TFT_RESET_ACTIVE (GPIOC->BSRR = 1 << 17)
// PA1
#define TFT_WR_IDLE (GPIOA->BSRR = 1 << 1)
#define TFT_WR_ACTIVE (GPIOA->BSRR = 1 << 17)
// PA0
#define TFT_RD_IDLE (GPIOA->BSRR = 1 << 0)
#define TFT_RD_ACTIVE (GPIOA->BSRR = 1 << 16)

#define TFT_RD_STROBE {TFT_RD_IDLE; TFT_RD_ACTIVE; TFT_RD_ACTIVE; TFT_RD_ACTIVE;} // Generate a rising edge pulse at RD pin
#define TFT_WR_STROBE {TFT_WR_ACTIVE; TFT_WR_IDLE;}// Generate a falling edge pulse at RW pin

#define TFT_READ_DELAY_72MHZ {TFT_RD_ACTIVE;}
// at clock speed = 180MHz
#define TFT_WRITE_DELAY_180MHZ {for(uint8_t i = 0; i < 8; i++){TFT_RW_ACTIVE;}}
#define TFT_READ_DELAY_180MHZ {for(uint8_t i = 0; i < 16; i++){TFT_RD_ACTIVE;}}

#define DELAY(n) {for(uint8_t i = 0; i < n; i++){__NOP();}}

#define TFT_PORTA_MASK ((1<<9)|(1<<10)|(1<<8))        //#LCD_D0, #LCD_D2, #LCD_D7
#define TFT_PORTB_MASK ((1<<3)|(1<<5)|(1<<4)|(1<<10)) //#LCD_D3, #LCD_D4, #LCD_D5, #LCD_D6
#define TFT_PORTC_MASK ((1<<7))                       //#LCD_D1

/*
 // Reset all LCD pins data of TFT LCD
 GPIOA->BSRR = TFT_PORTA_MASK << 16;\ // clear PA9, 10, 8
 GPIOB->BSRR = TFT_PORTB_MASK << 16;\// clear PB3, 4, 5, 6
 GPIOC->BSRR = TFT_PORTC_MASK << 16;\// clear PC7
 GPIOA->BSRR |= ((((data) & (1 << 0)) << 9)\// LCD_D0 connect to PA9 -> if bit 0 == 1 then shift 9 time to reach 9
 | (((data) & (1 << 2)) << 8)\// LCD_D2 connect to PA10 -> if bit 2 == 1 then shift 8 time to reach 10
 | (((data) & (1 << 7)) << 1));\// LCD_D7 connect to PA8 -> if bit 7 == 1 then shift 1 time to reach 8
 GPIOB->BSRR |= ((((data) & (1 << 3)) << 0) \// LCD_D3 connect to PB3 -> if bit 3 == 1 then no need to shift
 | (((data) & (1 << 4)) << 1)\// LCD_D4 connect to PB5 -> if bit 4 == 1 then shift 1 time to reach 5
 | (((data) & (1 << 5)) >> 1)\// LCD_D5 connect to PB4 -> if bit 5 == 1 then shift right 1 time to reach 4
 | (((data) & (1 << 6)) << 4));\// LCD_D6 connect to PB10 -> if bit 6 == 1 then shift 4 time to reach 10
 GPIOC->BSRR |= (((data) & (1 << 1)) << 6); \// LCD_D1 connect to PC7 -> if bit 1 == 1 then shift 6 time to reach 7
 TFT_WR_IDLE;\
TFT_WR_STROBE;\
TFT_WR_IDLE;\
)
 */
//@formatter:off

// BSRR no need to OR(|) or AND(&), ODR need to OR, AND
#define Write8bit(data) {										\
GPIOA->BSRR = TFT_PORTA_MASK << 16;					\
GPIOB->BSRR = TFT_PORTB_MASK << 16;					\
GPIOC->BSRR = TFT_PORTC_MASK << 16;					\
GPIOA->BSRR = ((((data) & (1 << 0)) << 9)		\
	| (((data) & (1 << 2)) << 8)							\
	| (((data) & (1 << 7)) << 1));						\
GPIOB->BSRR = ((((data) & (1 << 3)) << 0) 	\
						 | (((data) & (1 << 4)) << 1)		\
						 | (((data) & (1 << 5)) >> 1)		\
						 | (((data) & (1 << 6)) << 4));	\
 GPIOC->BSRR = (((data) & (1 << 1)) << 6);	\
 TFT_WR_IDLE;																\
 TFT_WR_STROBE;															\
 TFT_WR_IDLE;																\
}

#define Write16bit(data) { 	\
			Write8bit(data >> 8);	\
			Write8bit(data);			\
}

 //@formatter:on
class ILI9486: public Adafruit_GFX, public TouchScreen {
	private:
		bool _flipY = false;
		bool _flipX = false;

		void ConfigPortInput() {
			// Reset all pin of bus 8 lines TFT to 00 (Input mode)
			GPIOA->MODER &= ~0x3F0000;
			GPIOB->MODER &= ~0x300FC0;
			GPIOC->MODER &= ~0xC000;
		}

		void ConfigPortOutput() {
			// Config all pin of bus 8 lines TFT to 01 (Output mode)
			GPIOA->MODER |= 0x150000;
			GPIOB->MODER |= 0x100540;
			GPIOC->MODER |= 0x4000;
		}

		uint8_t Read8bit() {
			uint8_t data;
			TFT_RD_STROBE
			TFT_READ_DELAY_72MHZ
			data = (((GPIOA->IDR & (1 << 9)) >> 9) // formatter need line comment
							| ((GPIOC->IDR & (1 << 7)) >> 6) //
							| ((GPIOA->IDR & (1 << 10)) >> 8) //
							| ((GPIOB->IDR & (1 << 3)) >> 0) //
							| ((GPIOB->IDR & (1 << 5)) >> 1) //
							| ((GPIOB->IDR & (1 << 4)) << 1) //
							| ((GPIOB->IDR & (1 << 10)) >> 4) //
							| ((GPIOA->IDR & (1 << 8)) >> 1));
			TFT_RD_IDLE;
			return data;
		}

		uint16_t Read16bit() {
			uint16_t ret;
			uint8_t lo;
			ret = Read8bit();
			HAL_Delay(1);
			lo = Read8bit();
			return (ret << 8) | lo;
		}
	public:
		/*Start Adafruit_GFX override method */

		ILI9486(int16_t w, int16_t h) : Adafruit_GFX(w, h) {
		}

		void drawPixel(int16_t x, int16_t y, uint16_t color) {
			if (x < 0 || y < 0) return;
			SetAddrWindow(x, x, y, y);
			TFT_CS_ACTIVE;
			WriteCommand(0x2c);
			WriteData(color);
			TFT_CS_IDLE;
		}

		void setRotation(uint8_t r) {
			uint8_t val = 0;
			rotation = r & 3;
			_width = (rotation & 1) ? HEIGHT : WIDTH;
			_height = (rotation & 1) ? WIDTH : HEIGHT;
			switch (rotation) {
			//Memory Access Control (36h)
			case 0:                    //PORTRAIT:
				val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
				break;
			case 1:                    //LANDSCAPE: 90 degrees
				val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
				break;
			case 2:                    //PORTRAIT_REV: 180 degrees
				val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
				break;
			case 3:                    //LANDSCAPE_REV: 270 degrees
				val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
				break;
			}
			if (_flipY) val ^= 0x80; //MY
			if (_flipX) val ^= 0x40; //MX
//			val ^= 0x20; // MV
			TFT_CS_ACTIVE;
			WriteCommand(0x36);
			Write8bit(val);
			TFT_CS_IDLE;
			SetAddrWindow(0, 0, width() - 1, height() - 1);
		}

		void print(char *s) {
			uint8_t len = strlen(s);
			uint8_t i = 0;
			while (i < len) {
				Adafruit_GFX::write((uint8_t) *(s + i));
				i++;
			}
		}

		void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
			TFT_CS_ACTIVE;
			if (width() != WIDTH) { // if column and page exchanged
				int16_t temp = x;
				x = y;
				y = temp;
				// now that x mean y and y mean x
				WriteCommand(0x2b);
				Write8bit(x >> 8); //SP[15:8]
				Write8bit(x); //SP[7:0]
				Write8bit(h >> 8); //EP[15:8]
				Write8bit(h); //EP[7:0]
				WriteCommand(0x2a);
				Write8bit(y >> 8); //SC[15:8]
				Write8bit(y); //SC[7:0]
				Write8bit(y >> 8); //EC[15:8]
				Write8bit(y); //EC[7:0]
				WriteCommand(0x2c);
			}
			else {
				WriteCommand(0x2b);
				Write8bit(x >> 8); //SP[15:8]
				Write8bit(x); //SP[7:0]
				Write8bit(x >> 8); //EP[15:8]
				Write8bit(x); //EP[7:0]
				WriteCommand(0x2a);
				Write8bit(y >> 8); //SC[15:8]
				Write8bit(y); //SC[7:0]
				Write8bit(h >> 8); //EC[15:8]
				Write8bit(h); //EC[7:0]
				WriteCommand(0x2c);
			}
			do {
				Write16bit(color);
			} while (h-- > 0);
			TFT_CS_IDLE;
		}

		void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
			TFT_CS_ACTIVE;

			if (width() != WIDTH) { // if column and page exchanged
				int16_t temp = x;
				x = y;
				y = temp;
				// now that x mean y and y mean x
				WriteCommand(0x2b);
				Write8bit(x >> 8); //SP[15:8]
				Write8bit(x); //SP[7:0]
				Write8bit(x >> 8); //EP[15:8]
				Write8bit(x); //EP[7:0]
				WriteCommand(0x2a);
				Write8bit(y >> 8); //SC[15:8]
				Write8bit(y); //SC[7:0]
				Write8bit(w >> 8); //EC[15:8]
				Write8bit(w); //EC[7:0]
				WriteCommand(0x2c);
			}
			else {
				WriteCommand(0x2b);
				Write8bit(x >> 8); //SP[15:8]
				Write8bit(x); //SP[7:0]
				Write8bit(w >> 8); //EP[15:8]
				Write8bit(w); //EP[7:0]
				WriteCommand(0x2a);
				Write8bit(y >> 8); //SC[15:8]
				Write8bit(y); //SC[7:0]
				Write8bit(y >> 8); //EC[15:8]
				Write8bit(y); //EC[7:0]
				WriteCommand(0x2c);
			}
			do {
				Write16bit(color);
			} while (w-- > 0);
			TFT_CS_IDLE;
		}
		/*End Adafruit_GFX override method */

		void Begin() {
			TouchScreen::Begin(ADC1);
			//cubeMX config ADC pin to Analog
			TouchScreen::ConfigOutputPin(GPIOB, GPIO_PIN_0);
			TouchScreen::ConfigOutputPin(GPIOA, GPIO_PIN_4);
			// Soft reset
			TFT_CS_ACTIVE;
			WriteCommand(0x01);
			TFT_CS_IDLE;
			HAL_Delay(150); // need to delay after reset

			// Display off
			TFT_CS_ACTIVE;
			WriteCommand(0x28);
			TFT_CS_IDLE;
			HAL_Delay(1); // need to delay after reset

			// Interface pixel format
			TFT_CS_ACTIVE;
			WriteCommand(0x3a);
			Write8bit(0x55);
			TFT_CS_IDLE;

			// Power control 1
			TFT_CS_ACTIVE;
			WriteCommand(0xc0);
			Write8bit(0x0d);
			Write8bit(0x0d);
			TFT_CS_IDLE;

			// Power control 2
			TFT_CS_ACTIVE;
			WriteCommand(0xc1);
			Write8bit(0x43);
			Write8bit(0x00);
			TFT_CS_IDLE;

			// Power control 3
			TFT_CS_ACTIVE;
			WriteCommand(0xc2);
			Write8bit(0x00);
			TFT_CS_IDLE;

			// VCOM control 1
			TFT_CS_ACTIVE;
			WriteCommand(0xc5);
			Write8bit(0x00);
			Write8bit(0x48);
			Write8bit(0x00);
			Write8bit(0x48);
			TFT_CS_IDLE;

			// Inversion control
			TFT_CS_ACTIVE;
			WriteCommand(0xb4);
			TFT_CS_IDLE;

			// Display Function control
			TFT_CS_ACTIVE;
			WriteCommand(0xb6);
			Write8bit(0x02);
			Write8bit(0x02);
			Write8bit(0x3b);
			TFT_CS_IDLE;

			// Positive Gamma setting
//@formatter:off
			uint8_t pGammaSetting[] = {0x0F,0x21,0x1C,0x0B,0x0E,0x08,0x49,0x98,0x38,0x09,0x11,0x03,0x14,0x10,0x00};
//@formatter:on
			TFT_CS_ACTIVE;
			TFT_CD_COMMAND;
			WriteCommand(0xe0);
			for (auto gamma : pGammaSetting) {
				TFT_CD_DATA;
				Write8bit(gamma);
			}
			TFT_CS_IDLE;

			// Negative Gamma setting
//@formatter:off
			uint8_t nGammaSetting[] = {0x0F,0x2F,0x2B,0x0C,0x0E,0x06,0x47,0x76,0x37,0x07,0x11,0x04,0x23,0x1E,0x00};
//@formatter:on
			TFT_CS_ACTIVE;
			TFT_CD_COMMAND;
			WriteCommand(0xe1);
			for (auto gamma : nGammaSetting) {
				TFT_CD_DATA;
				Write8bit(gamma);
			}
			TFT_CS_IDLE;

			// Sleep out
			TFT_CS_ACTIVE;
			WriteCommand(0x11);
			TFT_CS_IDLE;
			HAL_Delay(150); // delay for self-diagnostic

			// Display on
			TFT_CS_ACTIVE;
			WriteCommand(0x29);
			TFT_CS_IDLE;

			// Memory access control, to rotate, flip
			TFT_CS_ACTIVE;
			WriteCommand(0x36);
			Write8bit(0x48);
			TFT_CS_IDLE;

			// No inverse
			TFT_CS_ACTIVE;
			WriteCommand(0x20);
			TFT_CS_IDLE;

		}

		void SetAddrWindow(int16_t x0, int16_t x, int16_t y0, int16_t y) {
			TFT_CS_ACTIVE;
			WriteCommand(0x2b);
			Write8bit(x0 >> 8); //SP[15:8]
			Write8bit(x0); //SP[7:0]
			Write8bit(x >> 8); //EP[15:8]
			Write8bit(x); //EP[7:0]
			WriteCommand(0x2a);
			Write8bit(y0 >> 8); //SC[15:8]
			Write8bit(y0); //SC[7:0]
			Write8bit(y >> 8); //EC[15:8]
			Write8bit(y); //EC[7:0]
			TFT_CS_IDLE;
		}

		void WriteCommand(uint16_t cmd) {
			TFT_CD_COMMAND;
			Write16bit(cmd);
			TFT_CD_DATA;
		}

		void WriteData(uint16_t data) {
			Write8bit(data >> 8);
			Write8bit((uint8_t ) data & 0xff);
		}

		void ReadGRAM(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *outData) {
			if (x < 0 || y < 0) return;
			uint16_t ret;
			int16_t n = w * h;
			SetAddrWindow(x, x + w - 1, y, y + h - 1);
			while (n > 0) {
				TFT_CS_ACTIVE;
				WriteCommand(0x2e);
				ConfigPortInput();
				Read8bit(); // dummy byte
				while (n) {
					ret = (Read8bit() << 8 | Read8bit());
					*outData++ = ret;
					n--;
				}
				TFT_RD_IDLE;
				TFT_CS_IDLE;
				ConfigPortOutput();
			}

		}

		uint32_t ReadID() {
			uint32_t ret;
			TFT_CS_ACTIVE;
			WriteCommand(0x00D3);
			ConfigPortInput();
			DELAY(10);
			// No need to read last byte because this is ignore byte (according to datasheet)
			ret = (Read8bit() << 24 | Read8bit() << 16 | Read8bit() << 8 | Read8bit());
			TFT_RD_IDLE;
			TFT_CS_IDLE;
			ConfigPortOutput();
			return ret;
		}

		void ReadRegister(uint16_t reg, uint8_t byteToRead, uint8_t *outBuffer) {
			TFT_CS_ACTIVE;
			WriteCommand(reg);
			ConfigPortInput();
			HAL_Delay(1);
			Read8bit();
			byteToRead--;
			while (byteToRead-- > 0)
				*outBuffer++ = Read8bit();
			TFT_RD_IDLE;
			TFT_CS_IDLE;
			ConfigPortOutput();
		}

		uint16_t ReadRegister(uint16_t reg, int8_t byteToRead) {
			uint16_t ret;
			TFT_CS_ACTIVE;
			WriteCommand(reg);
			ConfigPortInput();
			HAL_Delay(1);
			do {
				ret = Read16bit();
			} while (--byteToRead >= 0);
			TFT_RD_IDLE;
			TFT_CS_IDLE;
			ConfigPortOutput();
			return ret;
		}

		void FlipX(bool flip) {
			_flipX = true;
		}
		void FlipY(bool flip) {
			_flipY = true;
		}

		void Reset() {
			ConfigPortOutput();
			TFT_CS_IDLE;
			TFT_RD_IDLE;
			TFT_WR_IDLE;
			TFT_RESET_IDLE;
			HAL_Delay(1);
			TFT_RESET_ACTIVE;
			HAL_Delay(1);
			TFT_RESET_IDLE;
			HAL_Delay(1);
		}
};

#endif /* INC_ILI9486_HPP_ */
