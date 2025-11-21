
 /** HD44780.h
 * Header file for HD44780 based LCD displays
 *
 * (c) 2018 Solar Technology Inc.
 * 7620 Cetronia Road
 * Allentown PA, 18106
 * 610-391-8600
 *
 * This code is for the exclusive use of Solar Technology Inc.
 * and cannot be used in its present or any other modified form
 * without prior written authorization.
 *
 * HOST PROCESSOR: Atmel / Microchip AVR series
 * TARGET DISPLAY: Newhaven Display NHD-0216HZ-FSW-FBW-33V3C or similar HC44780 based display
 *
 * All LCD command definitions are taken from the Newhaven Display NHD-0216HZ-FSW-FBW-33V3C datasheet rev. 1
 *
 * REVISION HISTORY
 *
 * 1.0: 12/21/2125	Created By Nicholas C. Ipri (NCI) nipri@solartechnology.com
 */
 
 #include <avr/io.h>

// Prevent recursive inclusion
#ifndef HD44780_H_
#define HD44780_H_


#define HD44780_CommandMode		PORTD &= ~(1 << PD2); // LCD pin 4
#define HD44780_DataMode		PORTD |= (1 << PD2);
#define HD44780_SetEnable		PORTD |= (1 << PD3); // LCD pin 6
#define HD44780_ClearEnable		PORTD &= ~(1 << PD3);
#define HD44780_SetRW			PORTD |= (1 << PD4); // LCD pin 5
#define HD44780_ClearRW			PORTD &= ~(1 << PD4);


//HD44890 Basic Commands and Mask Bits
#define CLEAR_DISPLAY				0x01

#define CURSOR_HOME					0x02

#define ENTRY_MODE_SET				0x04
#define SET_CURSOR_INC				0x02
#define SET_CURSOR_DEC				0x00
#define SET_DISPLAY_BLINK			0x01
#define SET_DISPLAY_NOBLINK			0x00

#define DISPLAY_ON_OFF_CONTROL		0x08
#define SET_DISPLAY_ON				0x04
#define SET_DISPLAY_OFF				0x00
#define SET_CURSOR_ON				0x02
#define SET_CURSOR_OFF				0x00
#define SET_CURSOR_BLINK			0x01
#define SET_CURSOR_NOBLINK			0x00

#define CURSOR_DISPLAY_SHIFT		0x10
#define SET_DISPLAY_SHIFT			0x08
#define SET_CURSOR_SHIFT			0x00
#define SET_SHIFT_DIRECTION_RIGHT	0x04
#define SET_SHIFT_DIRECTION_LEFT	0x00

#define FUNCTION_SET				0x20
#define SET_DATA_LENGTH_8			0x10
#define SET_DATA_LENGTH_4			0x00
#define SET_2LINE					0x08
#define SET_1LINE					0x00
#define	SET_DISPLAY_FONT_5_10		0x04
#define	SET_DISPLAY_FONT_5_8		0x00

#define SET_CGRAM_ADDRESS			0x40

#define SET_DDRAM_ADDRESS			0x80

#endif /* HD44780_H_ */
