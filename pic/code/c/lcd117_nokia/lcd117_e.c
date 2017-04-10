// -------------------------------------------------------------------
// LCD117_E
//
// copyright, Peter H Anderson, East Corinth, VT, July, '03
//
// Nov 10, '04 - Corrected error on cursor type command.  Was not saving to
// EEPROM.  Added ?+ and ?- commands to enable and disable startup display.
// Reduced delay on boot.
//
// This adds big number and other capabilities. - April 2, '06
//
// Change chip fromm 16F648A to 16F1827 - LCD117.D - PHA - '10
//
// Release LCD117.D source code under MIT license as LCD117.E with
//    minor fix to EEPROM code - estate of PHA and BBR - May '13
//
// Modified for Mplab Hitech C compilation with 1847, 88, 688, 690 chips
// 	  and for use with Nokia 1100 lcd (96 x 65) - T. Ogden - July '16
// ---------------------------------------------------------------------

/**************************************************************************
 * Copyright (c) <2013>  - estate of Peter H. Anderson
 *                         Eric Anderson, Brian B. Riley, Paul Badger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be
 *     included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 *   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 *   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 **************************************************************************/

/**************************************************************************
 * The Peter Anderson code released here under MIT License is essentially
 * Peter's source code for the LCD117.D firmware release _UNTOUCHED_ save
 * for two exceptions
 * (1) - the firmware parameter display has been edited to reflect "LCD117.E"
 * (2) - there is a quirk in the CCS compiler with regard to how it writes
 *       strings to EEPROMs using he "#rom" directive. See the code correction
 *       and comments in the vicinity of line 1630
 *
 *  BBR, Underhill Center,VT  -- 22 May 2013
 **************************************************************************/


#include <htc.h>
//#include <string.h>
#include <stdio.h>
#include "..\include\lcd1100.h"

#define byte unsigned char
#define W 0
#define F 1

//#device PIC16F1827 *=16

//#define FINAL

#ifdef DEVELOP
//#device ICD=TRUE
//#rom 0x8007 = {0x09e4, 0x0003}  // Manual Section 4.0
#endif

#ifdef FINAL
//#rom 0x8007 = {0x095c, 0x1610}
#endif

// Chip selection
//#define CHIP88
//#define CHIP690
#define CHIP688		// 688 has NO PWM for LCD backlight brightness!!
//#define CHIP1847


//#fuses INTRC_IO, WDT, PUT, MCLR, PROTECT, NOCPD, NOBROWNOUT, NOCLKOUT
//#fuses NOIESO, NOFCMEN, WRT, STVREN, NODEBUG, NOLVP
// for htc
#ifdef CHIP688
__CONFIG(FOSC_INTOSCIO & WDTE_OFF & PWRTE_OFF & MCLRE_ON & CP_OFF & CPD_OFF & IESO_OFF & FCMEN_ON & BOREN_OFF);	// config word 2
#else
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & CP_OFF & CPD_OFF & BOREN_OFF & CLKOUTEN_OFF & IESO_ON & FCMEN_ON);	// config word 1
__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_ON & LVP_OFF);	// config word 2
#endif

#ifndef _XTAL_FREQ
 // Unless already defined assume 4MHz system frequency
 // This definition is required to calibrate __delay_us() and __delay_ms()
 #define _XTAL_FREQ 4000000
#endif

// for xc8
//#pragma config CLKOUTEN = OFF, WDTE = ON, PWRTE = OFF, CP = OFF, BOREN = ON, FCMEN = ON, MCLRE = ON, CPD = OFF, IESO = ON, FOSC = INTOSC

#define _19200
//#define _9600
//#define _4800
//#define _2400
//#define _1200
//#define _300

// timing routines - 4 MHz clock
void delay_10us(byte t);
void delay_ms(long t);

void lcd_init(void);
void lcd_clr(void);
void lcd_char(byte c);
void lcd_string(char*);
void lcd_write_hex(byte c);
//void lcd_cmd_byte(byte c);
//void lcd_cmd_nibble(byte c);
void lcd_clr_line(byte line);
void lcd_cursor_pos(byte line, byte pos);
void lcd_line_feed(void);
void lcd_reverse_line_feed(void);
void lcd_CR(void);

//void lcd_set_cursor_style(byte v);

void lcd_non_destructive_back_space(void);
void lcd_destructive_back_space(void);
void lcd_increment_cursor_pos(void);
void lcd_control_sequence(void);

void lcd_cursor_home(void);
void lcd_tab(void);
void lcd_beep(void);
void lcd_test(void);

void lcd_write_user_chars(void);

// serial routines
void asynch_enable(void);
void asynch_disable(void);

void open_ser_com(void);
void close_ser_com(void);   // never used
byte get_rx_buff(byte *ch_ptr);
void reset_rx_buff(void);

//unsigned char num_to_char(byte val);
char num_to_char(byte val);

byte get_char(void);
byte ser_get_hex2(byte *p);

// eeprom routines
void put_byte_eeprom(byte adr, byte d);
byte get_byte_eeprom(byte adr);

void put_bytes_eeprom(byte adr, byte *pd, byte num_bytes);

/*
 * lcd pinout
 * 1   2   3    4    5   6   7   8
 * LED Vdd Vdd1 sclk sda GND CS  RES
 *
 * ICSP connector
 * 1       2    3      4      5
 * orange  red  black  white  white
 * Vpp     Vcc  Gnd    dat    clk
 *
 *                  disable lvp  no CCP(PWM)
 * SIGNAL   1827/47    NEW(88)	 NEW (688) 	NEW (690)
 * sclk  	RA0        RA0          RC0			RC0
 * sda   	RA1        RA1          RC1			RC1
 * cs    	RA2        RA2          RC2			RC2
 * rst   	RA3        RA3          RC3			RC3
 * SPKR     RB0        RB0          RA5			RC7    // speaker output
 * PWM      RB3        RB3          RA4*		RC5    // pwm backlight intensity
 * RX       RB1        RB2          RC5			RB5
 * GPIO		RB4567	   RB4567		RA0-2		RA0-5(RB67 UNUSED)

 *688 has no PWM module, use timer interrupts instead

*/
// IO pins
#define LED PORTAbits.RA5
#ifdef CHIP688
    #define SPKR PORTAbits.RA5
    #define SPKR_TRIS TRISA5
    #define RX PORTCbits.RC5
    #define RX_TRIS TRISC5
    #define PWM PORTAbits.RA4
    #define PWM_TRIS TRISA4

	#define GPIO_PORT PORTA
	#define GPIO_TRIS TRISA
	#define GPIO_MIN 0
	#define GPIO_MAX 2
#endif

#define SUCCESS !0
#define FAILURE 0
#define POSITIVE 1
#define MINUS 0
#define TRUE !0
#define FALSE 0

#define RX_BUFF_MAX 64   // for serial routines - 32 byte buffer
#define RX_BUFF_FULL 3

#define S_C_START_ADR 0x00	// special characters
#define TAB_ADR 0x40
#define CURSOR_ADR 0x41
#define BACKLIGHT_ADR 0x42
#define START_UP_ADR 0x43
#define NUM_LINES_ADR 0x44
#define NUM_COLS_ADR 0x45

#define CUSTOM_START 0x48
#define CUSTOM_LINE_0 0x48
#define CUSTOM_LINE_1 0x48 + 20
#define CUSTOM_LINE_2 0x48 + 40
#define CUSTOM_LINE_3 0x48 + 60

#define MK_LONG(h, l)  ((unsigned long)(h) << 8) | (l)

const  byte _0_mask[8] = {0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
const  byte _1_mask[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

unsigned char const msgTitle[11]={'S','e','r','i','a','l',' ','L','C','D','\0'};
unsigned char const msgBaud[11]={'1','9','2','0','0',' ','B','a','u','d','\0'};

// for lcd
byte num_cols, num_lines;	// screen resolution { 1100: 96x65, 3310: 84x48, 8390: 84x48 }
byte current_line, current_col;
byte tab_size, bl_val, bl_count;

// for RS232 serial
byte rx_buff_full, rx_put_index, rx_get_index;
byte rx_buff[RX_BUFF_MAX];


void main(void){
    byte ch, x, row, col;

	// set clock speed to 4.0 MHz (0x6B)
	// osccon = 0110 1110 (chip88), 0110 1011 (chip688,690)
#ifdef CHIP1847
    OSCCON = 0x6B;   // 0110 1011
#else
    OSCCON = 0x61;   // 0110 0101.  shouldn't it be 0110 0001?
#endif

	// clear ansel (analog input)
#ifdef CHIP1847
    ANSELA = 0x00;
    ANSELB = 0x00;
	// set remapable pins for pwm and rx - 1827/47 specific
    CCP1SEL = 0; // on RB3 (CCP1/P1A)
    RXDTSEL = 0; // RB1 (RX/DT)
#else
	ANSEL = 0x00;
#endif

    // configure terminals for LCD
#ifdef CHIP1847
    PORTA = 0x00;
    TRISA = 0x00;
    PORTB = 0x00;
    TRISB = 0x06; // all except B1, B2 (for 1827/628/88)
	// set weak pull ups on B1, B2
	WPUB2 = 1;
	nWPUEN = 0;
#endif
#ifdef CHIP688
    PORTC = 0x00;
    TRISC = 0x20;	// 00100000 - all of port C as output except C5 (rx input)
    PORTA = 0x00;
    TRISA = 0x10;	// 00010000 all port A output except A4 (test)
	// set weak pull ups on A4
	WPUA = 0x10;	// 00010000
	nRAPU = 0;

	// shut off comparators (688)
	CMCON0 = 0x7;	// 111 comparators off
	VRCON = 0;		// Vref off
#endif

	LED = 1;
	delay_ms(100);
	LED = 0;
	delay_ms(50);
	LED = 1;
	delay_ms(100);
	LED = 0;

	lcd_init();
	lcdSetContrast(0x49);

	delay_ms(150);

	//lcd_init();
//	lcd_cmd_byte(0x06);   // increment cursor
	x = get_byte_eeprom(CURSOR_ADR);
//	lcd_set_cursor_style(x);
	num_lines = get_byte_eeprom(NUM_LINES_ADR);
	num_cols = get_byte_eeprom(NUM_COLS_ADR);
	tab_size = get_byte_eeprom(TAB_ADR);
	bl_val = get_byte_eeprom(BACKLIGHT_ADR);

	PWM = 0;
	PWM_TRIS = 0;

#ifdef CHIP688 // 688 has NO PWM! use timer0 @ 1ms overflow
	OPTION_REG = OPTION_REG & 0b11010000;	// set timer0 prescale to 4
	T0IF = 0;
	T0IE = 1;
#else	// configure CCP1 for PWM operation
	PR2 = 0xff;
	T2CON = 0x04;      // timer 2 on
	CCP1CON = 0x0c; // PWM Mode
	CCPR1L = bl_val;
#endif

	// enable interrupts
	PEIE = 1;
	GIE = 1;

	x = get_byte_eeprom(START_UP_ADR);
	x = x;
	if (x==1){
		lcd_clr_line(0);
#ifdef _19200
		lcdPrintString("#117E 19200");
#endif
#ifdef _9600
		lcdPrintString("#117E 9600");
#endif
#ifdef _4800
		lcdPrintString("#117E 4800");
#endif
#ifdef _2400
		lcdPrintString("#117E 2400");
#endif
		lcd_cursor_pos(2, 1);
		lcd_write_hex(num_lines);
		lcd_char(' ');
		lcd_write_hex(num_cols);
		lcd_char(' ');
		lcd_write_hex(tab_size);
		lcd_char(' ');
		lcd_write_hex(bl_val);

		//lcd_write_user_chars();

//       lcd_cursor_pos(2, 1);
//       for (n=0; n<8; n++){   // display the special characters
//           lcd_char(n);
//       }
		delay_ms(1500);
	}

//    if (x == 2){
//        for (row = 0; row < num_lines; row++){
//            lcd_clr_line(row);
//            for (col = 0; col < num_cols; col++){
//                 ch = get_byte_eeprom(CUSTOM_START + row * num_cols + col);
//                 lcd_char(ch);
//            }
//        }
//        delay_ms(2000);
//    }

	open_ser_com();
	lcd_clr();

	while(1){
		ch = get_char();
		if (ch == 0xfe){
			ch = get_char();
//			lcd_cmd_byte(ch);
			delay_ms(2);
		} else if (ch == '?'){
			lcd_control_sequence();
		} else if ((ch >= 32) && (ch <= 128)){   // ignore special characters
			lcd_char(ch);
			lcd_increment_cursor_pos();
		}
	}
}

void delay_10us(byte t){
   do{
#asm
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
#endasm
   } while(--t);
}

void delay_ms(long t){   // delays t millisecs
   do{
     delay_10us(99);
   } while(--t);
}

void lcd_control_sequence(void){
	byte n, x, y, v, row, col, z;
	byte ch, ch1;
	byte vals[8];
	byte custom_chars[20];

	ch = get_char();
	if ((ch >= '0')  && (ch <='7')){ // write special character to LCD
		x = ch - '0';
		lcd_char(x);
		lcd_increment_cursor_pos();
	} else {
		switch(ch){
			case 'a':   // home cursor
				lcd_cursor_home();
				delay_ms(2);
				break;
			case 'b':   // destructive backspace
				lcd_destructive_back_space();
				break;
//			case 'c':   // cursor style
//				ch1 = get_char();
//				x = ch1 - '0';
//				if ((x>=0) && (x<4)){
//					lcd_set_cursor_style(x);
//					put_byte_eeprom(CURSOR_ADR, x);
//				}
//				break;
			case 'f':
				lcd_clr();
				//delay_ms(2);
				break;
			case 'g':   // beep
				lcd_beep();
				break;
			case 'G':  // set geometry
				ch1 = get_char();
				x = ch1 - '0';
				if ((x >= 2) && (x <=4)){   // num_lines
	 				ch1 = get_char();   // first digit of num_cols
					y = ch1 - '0';
					if ((y>=1) && (y<=4)){
						v = y;
					} else {
						break;
					}

					ch1 = get_char();   // second digit of num_cols
					y = ch1 - '0';
					if ((y>=0) && (y<=9)){
						v = v * 10 + y;
					} else {
						break;
					}
					num_lines = x;
					num_cols = v;
					put_byte_eeprom(NUM_LINES_ADR, num_lines);
					put_byte_eeprom(NUM_COLS_ADR, num_cols);
				}
				break;
			case 'B':  // set backlight
			case 'E':  // save backlight setting to eeprom
				if (ser_get_hex2(&v) == SUCCESS){
					bl_val = v;
#ifndef CHIP688  // 688 had no PWM!
					CCPR1L = bl_val;
#endif
					if (ch == 'E')
						put_byte_eeprom(BACKLIGHT_ADR, v);
				}
				break;
			case 'h':
				lcd_non_destructive_back_space();
				break;
			case 'i':
				lcd_increment_cursor_pos();
				break;
			case 'I':
				if (ser_get_hex2(&v) == SUCCESS){
					if ((v>=0) && (v<=3)){
						lcd_char(v);
					}
				}
				break;
			case 'j':
				lcd_reverse_line_feed();
				break;
			case 'k':
				lcd_line_feed();
				break;
			case 'l':
				lcd_clr_line(current_line);
				break;
			case 'm':
				lcd_CR();
				break;
			case 'n':
				lcd_CR();
				lcd_line_feed();
				lcd_clr_line(current_line);
				break;
			case 's':   // set horizonatal tab size
				ch1 = get_char();
				x = ch1 - '0';
				if ((x>0) && (x<9)){
					tab_size = x;
					put_byte_eeprom(TAB_ADR, tab_size);
				}
				break;
			case 't':
				tab_size = get_byte_eeprom(TAB_ADR);   // probably an overkill
				lcd_tab();
				break;
			case 'x':   // cursor_col
				ch1 = get_char();
				v = 0x00;
				x = ch1 - '0';
#ifdef OLD
				if ((x == 0) || (x == 1))
#endif
				if ((x>=0) && (x<=3)){
					v = x;
					ch1 = get_char();
					x = ch1 - '0';
					if ((x>=0) && (x<=9)){
						v = v*10 + x;
					} else {
						break;
					}
					lcd_cursor_pos(current_line, v);
					break;
				}
				break;
			case 'y':   // cursor_line
				ch1 = get_char();
				x = ch1 - '0';
				if ((x>=0) && (x<4)){
					lcd_cursor_pos(x, current_col);
				}
				break;
//			case '!':   // command char to LCD
//				if (ser_get_hex2(&v) == SUCCESS){
//					lcd_cmd_byte(v);
//				}
//				break;
			case '?':
				lcd_char('?');
				lcd_increment_cursor_pos();
				break;
			case 'D':   // define special character
				ch1 = get_char();
				x = ch1 - '0';
				if (!((x >=0) && (x<=7))){   // number is outside range
					break;
				}

				for (n=0; n<8; n++){
					if (ser_get_hex2(&v) == SUCCESS){
						vals[n] = v;
					} else {
						break;
					}
				}

				// now save vals to EEPROM for special character x
				put_bytes_eeprom(S_C_START_ADR + 8 * x, vals, 8);
				lcd_write_user_chars(); // also send to LCD RAM
				break;
			case 'H':
			case 'L':
				ch1 = get_char();
				x = ch1 - '0';
				if (!((x >=GPIO_MIN) && (x<=GPIO_MAX))){   // number is outside range
					break;
				}

				GPIO_TRIS = GPIO_TRIS & _0_mask[x];
				if (ch == 'H'){
					GPIO_PORT = GPIO_PORT | _1_mask[x];
				} else {
					GPIO_PORT = GPIO_PORT & _0_mask[x];
				}
				break;
#ifdef OLD
			case '+':  // turn on startup message ######################
				put_byte_eeprom(START_UP_ADR, 0x01);
				break;
			case '-':  // turn off startup message
				put_byte_eeprom(START_UP_ADR, 0x00);
				break;
#endif
// ################################### added, Nov 10,
			case 'S':
				ch1 = get_char();
				x = ch1 - '0';
				if (!((x >=0) && (x<=2))){   // number is outside range
					break;
				}
				put_byte_eeprom(START_UP_ADR, x);
				break;
			case 'R': // restore default custom characters
				lcd_write_user_chars();
				break;
			case '*': // display custom screen, Mar, '05
				for (row = 0; row < num_lines; row++){
					lcd_clr_line(row);
					for (col = 0; col < num_cols; col++){
						ch = get_byte_eeprom(CUSTOM_START + row * num_cols + col);
						lcd_char(ch);
					}
				}
				current_line = 0; current_col = 0;
				break;
			case 'C': // custom line definition, Mar, '05
				ch1 = get_char();
				row = ch1 - '0';
				if (!((row >=0) && (row<=3))){   // number is outside range
					break;
				}
				lcd_cursor_pos(row, 0);
				for (col=0; col<num_cols; col++){
					ch = get_char();
					custom_chars[col] = ch;
					lcd_cursor_pos(row, col);
					lcd_char(ch);
				}
				for (col = 0; col<num_cols; col++){
					put_byte_eeprom(CUSTOM_START +  row * num_cols + col, custom_chars[col]);
				}

				current_line = 0;  current_col = 0;
				lcd_cursor_pos(current_line, current_col);
				break;
			case 'T': // lcd test
				lcd_test();
				break;
			default:
				break;
		}
	}
}

// LCD routines
void lcd_char(byte c){      // displays ASCII character c to LCD

    lcdPrintChar(c);
}


void lcd_string(unsigned char * str){      // displays ASCII character c to LCD
	while(*str){
		lcdPrintChar(*str++);
	}
}

void lcd_write_hex(char value) {
	char nibble;

	nibble = (value & 0xF0) >> 4;	// high nibble
	if (nibble > 9){
		nibble = nibble + 0x37;
	} else {
		nibble = nibble + 0x30;
	}
	lcdPrintChar(nibble);
	nibble = value & 0x0F;			// low nibble
	if (nibble > 9){
		nibble = nibble + 0x37;
	} else {
		nibble = nibble + 0x30;
	}
	lcdPrintChar(nibble);
}

//void lcd_cmd_byte(byte c){  // used for sending byte commands
//    lcd_cmd_nibble(c>>4);    // high byte followed by low
//    lcd_cmd_nibble(c&0x0f);
//    delay_10us(7);
//}

//void lcd_cmd_nibble(byte c){   // RS is at logic zero for commands
//    EN = 0;   // clock;
//    RS = 0;
//    LCD_PORT = (LCD_PORT & 0xf0) | c;
//    EN = 1;
//    EN = 0;
//}

void lcd_init(void){
	lcdInit();
	current_line = 0;
	current_col = 0;
}

void lcd_clr(void){      // clear LCD and cursor to upper left
    lcdClear();
	current_line = 0;
	current_col = 0;
}

void lcd_clr_line(byte line){   // clear indicated line and leave cursor at the beginning of the line
    byte n;
    lcd_cursor_pos(line, 0);
    for (n=0; n<num_cols; n++){
        lcd_char(' ');
    }
    lcd_cursor_pos(line, 0);
    current_line = line;
	current_col = 0;
}

void lcd_cursor_pos(byte line, byte col){
    const byte a[4] = {0x80, 0xc0, 0x80, 0xc0};
	// start address for the two lines
    if (line >= 0 && line <= 8){
		lcdSetX(col * 6);
		lcdSetY(line * 8);
	    current_line = line;
		current_col = col;
    }
}

void lcd_line_feed(void){
   ++current_line;
   if (current_line > (num_lines - 1)){
      current_line = 0;
   }
   lcd_cursor_pos(current_line, current_col);
}

void lcd_reverse_line_feed(void){
   if (current_line == 0){
      current_line = num_lines - 1;
   }
   else{
      --current_line;
   }
   lcd_cursor_pos(current_line, current_col);
}

void lcd_CR(void){
   current_col = 0;
   lcd_cursor_pos(current_line, current_col);
}

void lcd_non_destructive_back_space(void){
   if (current_col == 0){
      if (current_line == 0){
         current_line = num_lines - 1;
      }
      else{
         --current_line;
      }
      current_col = num_cols - 1;
   }
   else{
      -- current_col;
   }
   lcd_cursor_pos(current_line, current_col);
}

void lcd_destructive_back_space(void){
   lcd_non_destructive_back_space();
   lcd_char(' ');
   lcd_increment_cursor_pos();
   lcd_non_destructive_back_space();
}

//void lcd_set_cursor_style(byte v){
//   lcd_cmd_byte(0x0c+v);
//}

void lcd_cursor_home(void){
   lcd_cursor_pos(0, 0);
   current_line = 0;  current_col = 0;
}

void lcd_increment_cursor_pos(void){
   ++current_col;
   if (current_col > (num_cols-1)){
      current_col = 0;
      ++current_line;
      if (current_line > (num_lines - 1)){
         current_line = 0;
      }
   }
   lcd_cursor_pos(current_line, current_col);
}

void lcd_tab(void){
   byte n;

   if (current_col >= (num_cols-1)){ // already at end of line
      // do nothing
   } else if (((current_col) % tab_size) == 0){ // currently at a tab set
      for (n=0; n<tab_size; n++){
          if (current_col >= (num_cols-1)){ // at end of line
              break;
          }
          lcd_char(' ');
          lcd_increment_cursor_pos();
      }
   } else{
       for (n=0; ; n++){
           if (current_col >= (num_cols-1)){ // at end of line
              break;
           } else{
               if (((current_col) % tab_size) == 0){
                  break;
               }
               lcd_char(' ');
               lcd_increment_cursor_pos();
           }
       }
    }
}

void lcd_write_user_chars(void){   // write all user characters to LCD
//	byte char_num, n, x;
//
//	for (char_num = 0; char_num < 8; char_num++){
//		lcd_cmd_byte(0x40 + 8 * char_num);   // set CGRAM to address 0x00 + 8 * char_num
//		for (n=0; n<8; n++){
//			x = get_byte_eeprom(8 * char_num + n);
//			lcd_char(x);
//		}
//		delay_ms(5);
//	}
//	lcd_cursor_pos(current_line, current_col);   // return to regular mode
//	return;
}

void lcd_beep(void){
   byte n;
   SPKR = 0;
   SPKR_TRIS = 0;

   for(n = 0; n<25; n++){
        SPKR = 1;
        delay_10us(100);
        SPKR = 0;
        delay_10us(100);
   }
}

void lcd_test(void){
	byte n;
	delay_ms(300); // ######## Nov 11
	while(1){
		for (n=32; n<128; n++){
			if(n != '?'){
				lcd_char(n);
				delay_ms(250);
			}
		}
	}
}

///////////////// Serial Routines

void asynch_enable(void){
   //rxdtsel = 0;    // RX to RB1 1827 specific
   SYNC = 0;     // asynchronous
   BRGH = 1;
   BRG16 = 1;

#ifdef _300
   SPBRGH = 13;
   SPBRG = 4;  // 13 * 256 + 4 = 3332
#endif

#ifdef _19200
   SPBRGH = 0;
   SPBRG = 51;
#endif

#ifdef _9600
   SPBRGH = 0;
   SPBRG = 103;
#endif

#ifdef _4800
   SPBRGH = 0;
   SPBRG = 205;
#endif

#ifdef _2400
   SPBRGH = 1;
   SPBRG = 160;  // 1 * 256 + 160 = 416
#endif

#ifdef _1200
   SPBRGH = 3;
   SPBRG = 64;
#endif

   SPEN = 1;    // serial port enabled
   CREN = 1;
   //TRISB.1 = 1; // replaced with define
   RX_TRIS = 1;
}

void asynch_disable(void){
   CREN = 0;
   SPEN = 0;
}

void reset_rx_buff(void){
   //char ch;
   byte ch;

   rx_put_index = 0;
   rx_get_index = 0;
   rx_buff_full = FALSE;
   ch = RCREG;      // get any junk that may be in the buffer
   ch = RCREG;

   RCIF = 0;
   RCIE = 1;
   PEIE = 1;
   GIE = 1;
}

void open_ser_com(void){
   //char ch;
   byte ch;

   asynch_enable();
   rx_put_index = 0;
   rx_get_index = 0;
   rx_buff_full = FALSE;
   ch = RCREG;      // get any junk that may be in the buffer
   ch = RCREG;

   RCIF = 0;
   RCIE = 1;
}

void close_ser_com(void){
   RCIE = 0;
   RCIF = 0;
}

byte get_char(void){
   byte ch, error_code;
   // cren=0;
   CREN=1;
   RCIF=0;

   while((error_code = get_rx_buff(&ch)) == FAILURE){    // no character
      //if (nPOR == 0){
      //    nPOR = 1;
      //    while(1)     ;  // wait for watch dog timeout
      //}
	  if (OERR == 1){
		CREN = 0;
		CREN = 1;
	  }
   }

   if (error_code == RX_BUFF_FULL){
      return(0);   // no meaning
   } else if (error_code == SUCCESS){
      return(ch);
   } else{
      return(0);   // unexplained
   }
}

byte get_rx_buff(byte *ch_ptr){   // fetch a character
   byte error_code;

   if (rx_buff_full == TRUE){
      rx_buff_full = FALSE;
      error_code = RX_BUFF_FULL;   // overflow of rx_buff
      rx_put_index = 0;
      rx_get_index = 0;
      *ch_ptr = 0;      // buff was full.  returned character has no meaning
   } else if (rx_get_index == rx_put_index){   // there is no character
       error_code = FAILURE;      // no character
      *ch_ptr = 0;
   } else{
       *ch_ptr = rx_buff[rx_get_index];
      ++rx_get_index;
      if (rx_get_index >= RX_BUFF_MAX){
         rx_get_index = 0;   // wrap
      }
      error_code = SUCCESS;   // success
   }
   return(error_code);
}

byte ser_get_hex2(byte *p){   // clean this up
   byte ch, x, v;
   ch = get_char();
   if ((ch >= '0') && (ch<='9')){
      x = ch - '0';
   } else if ((ch >='a') && (ch <='f')){
      x = ch - 'a' + 10;
   } else if ((ch >='A') && (ch <='F')){
      x = ch - 'A' + 10;
   } else {
      return(FAILURE);
   }
   v = x;
   ch = get_char();
   if ((ch >= '0') && (ch <='9')){
      x = ch - '0';
   } else if ((ch >='a') && (ch <='f')){
      x = ch - 'a' + 10;
   } else if ((ch >='A') && (ch <='F')){
      x = ch - 'A' + 10;
   } else {
      return(FAILURE);
   }
   v = v*16 + x;
   *p = v;
   return(SUCCESS);
}

void put_byte_eeprom(byte adr, byte d){
   byte intcon_save; // n=20;

#ifdef CHIP1847
   CFGS = 0;
#endif

   EEPGD = 0;
   EEADR  = adr;
   EEDATA = d;
   intcon_save = INTCON;

   while(GIE){
      GIE = 0;
   }
   EEIF = 0;
   PEIE = 1;
   EEIE = 1;
   WREN = 1;
   EECON2 = 0x55;
   EECON2 = 0xaa;
   WR = 1;      // begin programming sequence
   WREN = 0;
   while(WR){
   }
   EEIE = 0;
   EEIF = 0;
   INTCON = intcon_save;
   GIE = 1;
}

void put_bytes_eeprom(byte adr, byte *pd, byte num_bytes){
   byte n;
   for (n=0; n<num_bytes; ++n, ++adr, ++pd){
      put_byte_eeprom(adr, *pd);
   }
}

byte get_byte_eeprom(byte adr){
   byte x;

#ifdef CHIP1847
   CFGS = 0;
#endif
   EEPGD = 0;
   EEADR = adr;
   RD = 1;
#asm
   NOP
   NOP
   NOP
   NOP
   NOP
#endasm
   x = EEDATA;
   return(x);
}


//#int_rda
void interrupt interrupt_handler(void){
	if(RCIE && RCIF){ // received data on RX line
		if (rx_buff_full == FALSE){
			rx_buff[rx_put_index] = RCREG;   // fetch the character
			++rx_put_index;
			if (rx_put_index >= RX_BUFF_MAX){
				rx_put_index = 0;   // wrap around
			}
			if (rx_put_index == rx_get_index){
				rx_buff_full = TRUE;
			}
		}
		RCIF = 0; // clear rda interrupt
	}
	if (T0IE && T0IF){ // timer overflow, ~1ms
        bl_count ++;
        if(bl_count == 30){
            bl_count = 0; // period for pwm
            if (bl_val > 0)
				PWM = 0; // turn on bl only if brightness > 0
        }
        // turn off bl after period expires
        if(bl_count == bl_val)
			PWM = 1;
        T0IF = 0; // clear timer interrupt
	}
}


// special characters
//__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f); // 00-07
//__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f); // 08-0F
//__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f); // 10-17
//__EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f); // 18-1F
//__EEPROM_DATA(0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f); // 20-27
//__EEPROM_DATA(0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f); // 28-2F
//__EEPROM_DATA(0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f); // 30-37
//__EEPROM_DATA(0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f); // 38-3F


// 40-47
__EEPROM_DATA(3,3,0,1,8,16,0,0); // TAB, CURSOR, BACKLIGHT, START_UP, NUM_LINES, NUM_COLS, ??

// 48-??
//__EEPROM_DATA('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h');
//__EEPROM_DATA('i', 'j', 'k', 'l', 'm', 'n', 'o', 'p');
//__EEPROM_DATA('q', 'r', 's', 't', 'A', 'B', 'C', 'D');
//__EEPROM_DATA('E', 'F', 'G', 'H', 'I', 'J', 'K', 'L');
//__EEPROM_DATA('M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T');
//__EEPROM_DATA('0', '1', '2', '3', '4', '5', '6', '7');
//__EEPROM_DATA('8', '9', '0', '1', '2', '3', '4', '5');
//__EEPROM_DATA('6', '7', '8', '9', '9', '8', '7', '6');
//__EEPROM_DATA('5', '4', '3', '2', '1', '0', '9', '8');
//__EEPROM_DATA('7', '6', '5', '4', '3', '2', '1', '0');
//
