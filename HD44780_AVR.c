/** HD44780.c
 * Source file for controlling HD44780 based LCD displays
 *
 * This code is open source, free to use and distribute. I only ask that you 
 * update the revision history with any changes or improvements that you might make
 * and that you keep my name and email at the top of the list
 *
 *
 * HOST PROCESSOR: Atmel / Microchip AVR Series
 * TARGET DISPLAY: New Haven Display NHD-0216HZ-FSW-FBW-33V3C and similar HC44780 based display in 4 bit mode
 *
 *
 * REVISION HISTORY
 *
 * 1.0: 11/21/2025	Created By Nicholas C. Ipri (NCI) nick@ny3s.com
 * 		This initial version supplies only very basic functionality to initialize and write the display in 4 bit mode
 */


#include "HD44780_AVR.h"
#include <string.h>
#include <util/delay.h>

void HD44780_GotoXY(uint8_t row, uint8_t col);
void HD44780_WriteData(uint8_t row, uint8_t col, char* data, uint8_t clearDisplay);
void HD44780_WriteCommand(uint8_t data);
void HD44780_Init(void);

void HD44780_WriteCommand(uint8_t data) {

	HD44780_CommandMode;
	HD44780_ClearRW;

	_delay_us(50); 

	if ( (data==0x03) || (data== 0x02) )  { // Used only for initialization

		HD44780_SetEnable;
		_delay_us(50); //150

		if ( (data&0x08) == 0x08 )
			PORTC |= (1 << PC3); // LCD pin 14
		else
			PORTC &= ~(1 << PC3);

		if ( (data&0x04) == 0x04) // LCD pin 13
			PORTC |= (1 << PC2);
		else
			PORTC &= ~(1 << PC2);

		if ( (data&0x02) == 0x02) // LCD pin 12
			PORTC |= (1 << PC1);
		else
			PORTC &= ~(1 << PC1);

		if ( (data&0x01) == 0x01) // LCD pin 11
			PORTC |= (1 << PC0);
		else
			PORTC &= ~(1 << PC0);

		_delay_us(50); //150
		HD44780_ClearEnable;
		_delay_us(50); //150
	}

	else {

		HD44780_SetEnable;
		_delay_us(50); //150

		if ( (data&0x80) == 0x80 )
			PORTC |= (1 << PC3);
		else
			PORTC &= ~(1 << PC3);

		if ( (data&0x40) == 0x40)
			PORTC |= (1 << PC2);
		else
			PORTC &= ~(1 << PC2);

		if ( (data&0x20) == 0x20)
			PORTC |= (1 << PC1);
		else
			PORTC &= ~(1 << PC1);

		if ( (data&0x10) == 0x10)
			PORTC |= (1 << PC0);
		else
			PORTC &= ~(1 << PC0);

		_delay_us(50); //150
		HD44780_ClearEnable;
		_delay_us(50); //150
		HD44780_SetEnable;
		_delay_us(150); //150

		if ( (data&0x08) == 0x08 )
			PORTC |= (1 << PC3);
		else
			PORTC &= ~(1 << PC3);

		if ( (data&0x04) == 0x04)
			PORTC |= (1 << PC2);
		else
			PORTC &= ~(1 << PC2);

		if ( (data&0x02) == 0x02)
			PORTC |= ~(1 << PC1);
		else
			PORTC &= ~(1 << PC1);

		if ( (data&0x01) == 0x01)
			PORTC |= (1 << PC0);
		else
			PORTC &= ~(1 << PC0);

		_delay_us(50); //150
		HD44780_ClearEnable;
		_delay_us(50); //150
	}
}

void HD44780_WriteData(uint8_t row, uint8_t col, char* data, uint8_t clearDisplay) {

	uint8_t length;
	uint8_t i;

	HD44780_ClearRW;

	if (clearDisplay)
	{
		HD44780_WriteCommand(CLEAR_DISPLAY);
		_delay_us(2000);
	}

	HD44780_GotoXY(row, col);
	_delay_us(15000); //15000

	HD44780_DataMode;
	_delay_us(50); //150

	length = strlen(data);

	for(i=0; i<length; i++) {

		HD44780_SetEnable;
		_delay_us(50); //150

		if ( (*data & 0x80) == 0x80 )
			PORTC |= (1 << PC3);
		else
			PORTC &= ~(1 << PC3);

		if ( (*data & 0x40) == 0x40)
			PORTC |= (1 << PC2);
		else
			PORTC &= ~(1 << PC2);

		if ( (*data & 0x20) == 0x20)
			PORTC |= (1 << PC1);
		else
			PORTC &= ~(1 << PC1);

		if ( (*data & 0x10) == 0x10)
			PORTC |= (1 << PC0);
		else
			PORTC &= ~(1 << PC0);

		_delay_us(50); //150
		HD44780_ClearEnable;
		_delay_us(50); //150
		HD44780_SetEnable;
		_delay_us(50); //150

		if ( (*data & 0x08) == 0x08 )
			PORTC |= (1 << PC3);
		else
			PORTC &= ~(1 << PC3);

		if ( (*data & 0x04) == 0x04)
			PORTC |= (1 << PC2);
		else
			PORTC &= ~(1 << PC2);

		if ( (*data & 0x02) == 0x02)
			PORTC |= (1 << PC1);
		else
			PORTC &= ~(1 << PC1);

		if ( (*data & 0x01) == 0x01)
			PORTC |= (1 << PC0);
		else
			PORTC &= ~(1 << PC0);

		_delay_us(50); //150
		HD44780_ClearEnable;
		_delay_us(50);//150

		data++;
	}
}

void HD44780_GotoXY(uint8_t row, uint8_t col) {

	uint8_t row_offsets[] = {0x00, 0x40};
	HD44780_WriteCommand(SET_DDRAM_ADDRESS | (col + row_offsets[row]));
}

void HD44780_Init() {

	HD44780_WriteCommand(0x03);
	_delay_us(50);

	HD44780_WriteCommand(0x03);
	_delay_us(50);

	HD44780_WriteCommand(0x03);
	_delay_us(50);

	HD44780_WriteCommand(0x02);
	_delay_us(50);

	HD44780_WriteCommand(FUNCTION_SET | SET_2LINE);
	_delay_us(50);

	HD44780_WriteCommand(DISPLAY_ON_OFF_CONTROL | SET_DISPLAY_ON | SET_CURSOR_OFF);
	_delay_us(50);

	HD44780_WriteCommand(CLEAR_DISPLAY);
	_delay_us(2000);

	HD44780_WriteCommand(ENTRY_MODE_SET | SET_CURSOR_INC);
	_delay_us(2000);

}

void HD44780_ReadBusy()
{
	uint8_t data;

	HD44780_CommandMode;
	HD44780_SetRW;


}


