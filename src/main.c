/*
 * CreatiVision Controller Interface 
 * ---------------------------------
 *
 * This work is licensed under GNU General Public License v3.0
 * 
 * Version: 1.0
 * 
 * Copyright 2024 by DigicoolThings
 * 
 * Author:  Greg@DigicoolThings.com            
 * Date: August 2024
 * 
 * Controller Interface for CreatiVision Consoles and CreatiVision re-Creations.
 * 
 * Written for 28 pin AVR DA series. 
 * e.g. AVR32DA28, AVR64DA128, AVR128DA28
 * 
 * Utilizes MT8816 8 x 16 Analog Switch Array
 *  for CreatiVision key-switch function replication.
 * 
 * Implements Dual Atari Joystick interfaces
 *  (to re-create Left & Right Controller Joystick function).
 * 
 * Implements PS/2 Keyboard input
 *  (to re-create the CreatiVision 48 Key Keyboard function).
 * 
 * NOTE: PS/2 Keyboard is implemented at a low-level to provide the most
 *  accurate operation, via key-down & key-up scan-code direct translation
 *  to CreatiVision key-switch on & off operation.
 * 
 *  As a result, some key usage differs between the default PS/2 key legends
 *  and CreatVision key legends. In particular the shifted keys.
 *  Physical keys have been mapped based on their non-shifted/main  key-legend.  
 *  e.g. The numeric keys on the CreatiVision keyboard have different shifted
 *  symbols than on a PS/2 keyboard.
 * 
 *  Also, CreatiVision has separate ':' and ';' keys (which are combined on
 *  a PS/2 keyboard as a single key).
 *  For this reason the PS/2 single-quote (') key, alongside the semicolon (;)
 *  key, is mapped to the CreatiVision colon (:) key.
 * 
 *  Therefore, to intuitively utilize a PS/2 keyboard you may wish to implement
 *  custom key-cap legends (to replicate the actual CreatiVision key cap
 *  legends), or utilize a custom key-cap PS/2 keyboard.
 * 
 * Code was developed in MPLAB X v6.20
 * MCC is used to generate code for AVR device settings:
 *  - Internal 4MHz clock (default)
 *  - Reset Pin (PF6) set to "Reset mode"
 *  - Global Interrupt Enabled
 *  - PA0 - PA7 GPIO defined as Outputs
 *  - PC0 - PC3, PD0 - PD7, PF0 - PF1 GPIO defined as Inputs,
 *          with Pull-ups enabled
 *  - PF0 (PS2_Clock_bm) - Input Sense Interrupt = "Sense Falling Edge"
 *     
 *  Credits: This work builds on work done by Kym Greenshields and Thomas
 *   Gutmeier. Specifically, their work identifying CreatiVision Controller
 *   key to controller pin mapping (i.e. "Keymappings_v2.xlsx" spreadsheet),
 *   and the use of a MT8808 Analog Switch Array. 
 * 
 * Change Log
 * ----------
 * v1.0  - Initial release.
 * 
 *    
 */
#include "mcc_generated_files/system/system.h"

#include "avr/io.h"
#include "avr/interrupt.h"
#include "util/atomic.h"

/*
 * PS/2 Keyboard Interrupt driven ScanCode Input Buffer
 */
#define PS2_ScanCodeBuffer_Size 254
static volatile uint8_t PS2_ScanCodeBuffer[PS2_ScanCodeBuffer_Size];
static volatile uint8_t PS2_ScanCodeBuffer_Start = 0;
static volatile uint8_t PS2_ScanCodeBuffer_End   = 0;

/*
 * PS/2 PORTF  PIN Bit Mask (bm) Definitions
 */
static const uint8_t PS2_Clock_bm  = PIN0_bm;
static const uint8_t PS2_Data_bm = PIN1_bm;

/*
 * MT8816 PORTA PIN Bit Mask (bm) Definitions
 * 
 * Note: PORTA is fully assigned to the MT8816
 * Pin0 - Pin3 = AX0 - AX3
 * Pin4 - Pin5 = AY0 - AY1 (AY3 input is unused / grounded)
 * Pin6 - Pin7 = Strobe & Data (as below) 
 */
static const uint8_t MT_Strobe_bm = PIN6_bm;
static const uint8_t MT_Data_bm   = PIN7_bm;

/*
 * MT8816 AY0-2 / AX0-3 Address Input definitions for CreatiVision Controllers
 * 
 * X0 - X7 / Y0 - Y1 are used for Left Controller
 * X8 - X15 / Y2 - Y3 are used for Right Controller
 * 
 * NOTE: Uses AVR PORTA - which MUST be configured as Outputs. 
 * 
 * Pin References are based on CreatiVEmu Forum shared "Keymappings_v2.xlsx"
 * 
 * IMPORTANT NOTES:
 *  Pin Numbers in the above spreadsheet are stated to be: "left to right as 
 *  seen from the front of the console".
 *  However, this does NOT correlate to the port numbering on the CreatiVision
 *  schematic!
 *  The CreatiVision schematic has numbering which equates to right to left,
 *  when looking at the CreatiVision's controller ports from the console front!
 *  The pin numbering on the CreatiVision schematic therefore correlates to
 *  a left to right 1 - 10 numbering of the Controller plug pins, when looking
 *  into the front of the controller's plug.
 * 
 *  Therefore, on this "CreatiVision Controller Interface", I have used the
 *  port connector pin numbering which matches the CreatiVision Schematics!
 *  My numbering (and CreatiVision Schematic numbering) therefore reflects
 *   1 - 10 pin numbering as equivalent to the spreadsheets 10 - 1 numbering!
 * i.e. 
 * "Keymappings_v2.xlsx"  <->  CreatiVision Schematic (and all following)
 *                Pin 10   =    Pin 1
 *                Pin 9    =    Pin 2
 *                Pin 8    =    Pin 3
 *                Pin 7    =    Pin 4
 *                Pin 6    =    Pin 5
 *                Pin 5    =    Pin 6
 *                Pin 4    =    Pin 7
 *                Pin 3    =    Pin 8
 *                Pin 2    =    Pin 9
 *                Pin 1    =    Pin 10
 * 
 *  With this understanding, then by referring to the CreatiVision schematics
 *  we have the following connector pin number mapping to actual PIA port pins:
 * 
 * Left Controller
 *  Pin 1  = PIA_PA1
 *  Pin 2  = PIA_PA0
 *  Pin 3  = PIA_PB0
 *  Pin 4  = PIA_PB1
 *  Pin 5  = PIA_PB2
 *  Pin 6  = PIA_PB3
 *  Pin 7  = PIA_PB4
 *  Pin 8  = PIA_PB7
 *  Pin 9  = PIA_PB6
 *  Pin 10 = PIA_PB5  
 * 
 * Right Controller
 *  Pin 1  = PIA_PB2
 *  Pin 2  = PIA_PB1
 *  Pin 3  = PIA_PB0
 *  Pin 4  = PIA_PB3
 *  Pin 5  = PIA_PB4
 *  Pin 6  = PIA_PB5
 *  Pin 7  = PIA_PB6
 *  Pin 8  = PIA_PB7
 *  Pin 9  = PIA_PA3
 *  Pin 10 = PIA_PA2  
 * 
 *  As both controllers connect to PIA Port B (via diode isolation), 
 *  and PORT A (directly), we can also create the following PIA Port pin
 *  to Controller Pin mapping equivalents:
 *
 *  PIA Port =  Left   =  Right
 *  --------    -----     -----
 *  PIA_PA0  =  Pin 2  =  x
 *  PIA_PA1  =  Pin 1  =  x
 *  PIA_PA2  =  x      =  Pin 10
 *  PIA_PA3  =  x      =  Pin 9
 *  PIA_PB0  =  Pin 3  =  Pin 3
 *  PIA_PB1  =  Pin 4  =  Pin 2
 *  PIA_PB2  =  Pin 5  =  Pin 1
 *  PIA_PB3  =  Pin 6  =  Pin 4
 *  PIA_PB4  =  Pin 7  =  Pin 5
 *  PIA_PB5  =  Pin 10 =  Pin 6
 *  PIA_PB6  =  Pin 9  =  Pin 7
 *  PIA_PB7  =  Pin 8  =  Pin 8
 *  
 * For the Switch byte binary defines below. we then have the binary data
 * representing: ruYYXXXX
 * Where: 'X' addresses Switch Array X0 - X15, 
 *        'Y' addresses Switch Array Y0 - Y3, 
 *        'r' is reserved (for a No Switch Action flag),
 *        'u' is unused.
 * 
 * Got it?  Yes, it is slightly complicated and a little confusing.
 * Let's try and simplify that, with the following! :)
 * 
 */

/*
 * Switch Action and PIA Port Switch binary byte defines
 * These are created to make the following Key Switch constant definitions
 * as readable as possible! :)
 */
#define NO_SWITCH_ACTION 0b10000000

#define PIA_PA0 0b00000000
#define PIA_PA1 0b00010000
#define PIA_PA2 0b00101000
#define PIA_PA3 0b00111000

#define PIA_PB0 0b00000000
#define PIA_PB1 0b00000001
#define PIA_PB2 0b00000010
#define PIA_PB3 0b00000011
#define PIA_PB4 0b00000100
#define PIA_PB5 0b00000101
#define PIA_PB6 0b00000110
#define PIA_PB7 0b00000111

/*
 * Individual Key Switch address constants are declared below, using
 * the above port defines (for clarity / maximum readability)!
 */
/*
 * CreatiVision Left Controller Keyboard (24 keys)
 */

/* Key 1 = Pin 2 -> Pin 6 + Pin 5 (PIA_PA0 -> PIA_PB3 + PIA_PB2) */
static const uint8_t Switch_1_a = PIA_PA0 | PIA_PB3;
static const uint8_t Switch_1_b = PIA_PA0 | PIA_PB2;

/* Key 2 = Pin 1 -> Pin 10 + Pin 7 (PIA_PA1 -> PIA_PB5 + PIA_PB4) */
static const uint8_t Switch_2_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_2_b = PIA_PA1 | PIA_PB4;

/* Key 3 = Pin 1 -> Pin 10 + Pin 9 (PIA_PA1 -> PIA_PB5 + PIA_PB6) */
static const uint8_t Switch_3_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_3_b = PIA_PA1 | PIA_PB6;

/* Key 4 = Pin 1 -> Pin 10 + Pin 6 (PIA_PA1 -> PIA_PB5 + PIA_PB3) */
static const uint8_t Switch_4_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_4_b = PIA_PA1 | PIA_PB3;

/* Key 5 = Pin 1 -> Pin 9 + Pin 6 (PIA_PA1 -> PIA_PB6 + PIA_PB3) */
static const uint8_t Switch_5_a = PIA_PA1 | PIA_PB6;
static const uint8_t Switch_5_b = PIA_PA1 | PIA_PB3;

/* Key 6 = Pin 1 -> Pin 9 + Pin 7 (PIA_PA1 -> PIA_PB6 + PIA_PB4) */
static const uint8_t Switch_6_a = PIA_PA1 | PIA_PB6;
static const uint8_t Switch_6_b = PIA_PA1 | PIA_PB4;

/* Key CNT'L = Pin 2 -> Pin 8 (PIA_PA0 -> PIA_PB7) */
static const uint8_t Switch_CNTL = PIA_PA0 | PIA_PB7;

/* Key Q = Pin 1 -> Pin 7 + Pin 6 (PIA_PA1 -> PIA_PB4 + PIA_PB3) */
static const uint8_t Switch_Q_a = PIA_PA1 | PIA_PB4;
static const uint8_t Switch_Q_b = PIA_PA1 | PIA_PB3;

/* Key W = Pin 1 -> Pin 6 + Pin 5 (PIA_PA1 -> PIA_PB3 + PIA_PB2) */
static const uint8_t Switch_W_a = PIA_PA1 | PIA_PB3;
static const uint8_t Switch_W_b = PIA_PA1 | PIA_PB2;

/* Key E = Pin 1 -> Pin 7 + Pin 5 (PIA_PA1 -> PIA_PB4 + PIA_PB2) */
static const uint8_t Switch_E_a = PIA_PA1 | PIA_PB4;
static const uint8_t Switch_E_b = PIA_PA1 | PIA_PB2;

/* Key R = Pin 1 -> Pin 10 + Pin 5 (PIA_PA1 -> PIA_PB5 + PIA_PB2) */
static const uint8_t Switch_R_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_R_b = PIA_PA1 | PIA_PB2;

/* Key T = Pin 1 -> Pin 9 + Pin 5 (PIA_PA1 -> PIA_PB6 + PIA_PB2) */
static const uint8_t Switch_T_a = PIA_PA1 | PIA_PB6;
static const uint8_t Switch_T_b = PIA_PA1 | PIA_PB2;

/* Key LEFT ARROW = Pin 1 -> Pin 6 + Pin 3 (PIA_PA1 -> PIA_PB3 + PIA_PB0) */
static const uint8_t Switch_LEFT_a = PIA_PA1 | PIA_PB3;
static const uint8_t Switch_LEFT_b = PIA_PA1 | PIA_PB0;

/* Key A = Pin 1 -> Pin 7 + Pin 3 (PIA_PA1 -> PIA_PB4 + PIA_PB0) */
static const uint8_t Switch_A_a = PIA_PA1 | PIA_PB4;
static const uint8_t Switch_A_b = PIA_PA1 | PIA_PB0;

/* Key S = Pin 1 -> Pin 10 + Pin 3 (PIA_PA1 -> PIA_PB5 + PIA_PB0) */
static const uint8_t Switch_S_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_S_b = PIA_PA1 | PIA_PB0;

/* Key D = Pin 1 -> Pin 9 + Pin 3 (PIA_PA1 -> PIA_PB6 + PIA_PB0) */
static const uint8_t Switch_D_a = PIA_PA1 | PIA_PB6;
static const uint8_t Switch_D_b = PIA_PA1 | PIA_PB0;

/* Key F = Pin 1 -> Pin 4 + Pin 3 (PIA_PA1 -> PIA_PB1 + PIA_PB0) */
static const uint8_t Switch_F_a = PIA_PA1 | PIA_PB1;
static const uint8_t Switch_F_b = PIA_PA1 | PIA_PB0;

/* Key G = Pin 1 -> Pin 5 + Pin 3 (PIA_PA1 -> PIA_PB2 + PIA_PB0) */
static const uint8_t Switch_G_a = PIA_PA1 | PIA_PB2;
static const uint8_t Switch_G_b = PIA_PA1 | PIA_PB0;

/* Key SHIFT = Pin 1 -> Pin 8 (PIA_PA1 -> PIA_PB7) */
static const uint8_t Switch_SHIFT = PIA_PA1 | PIA_PB7;

/* Key Z = Pin 1 -> Pin 6 + Pin 4 (PIA_PA1 -> PIA_PB3 + PIA_PB1) */
static const uint8_t Switch_Z_a = PIA_PA1 | PIA_PB3;
static const uint8_t Switch_Z_b = PIA_PA1 | PIA_PB1;

/* Key X = Pin 1 -> Pin 7 + Pin 4 (PIA_PA1 -> PIA_PB4 + PIA_PB1) */
static const uint8_t Switch_X_a = PIA_PA1 | PIA_PB4;
static const uint8_t Switch_X_b = PIA_PA1 | PIA_PB1;

/* Key C = Pin 1 -> Pin 10 + Pin 4 (PIA_PA1 -> PIA_PB5 + PIA_PB1) */
static const uint8_t Switch_C_a = PIA_PA1 | PIA_PB5;
static const uint8_t Switch_C_b = PIA_PA1 | PIA_PB1;

/* Key V = Pin 1 -> Pin 9 + Pin 4 (PIA_PA1 -> PIA_PB6 + PIA_PB1) */
static const uint8_t Switch_V_a = PIA_PA1 | PIA_PB6;
static const uint8_t Switch_V_b = PIA_PA1 | PIA_PB1;

/* Key B = Pin 1 -> Pin 5 + Pin 4 (PIA_PA1 -> PIA_PB2 + PIA_PB1) */
static const uint8_t Switch_B_a = PIA_PA1 | PIA_PB2;
static const uint8_t Switch_B_b = PIA_PA1 | PIA_PB1;

/*
 * CreatiVision Right Controller Keyboard (24 keys)
 */

/* Key 7 = Pin 9 -> Pin 2 + Pin 1 (PIA_PA3 -> PIA_PB1 + PIA_PB2) */
static const uint8_t Switch_7_a = PIA_PA3 | PIA_PB1;
static const uint8_t Switch_7_b = PIA_PA3 | PIA_PB2;

/* Key 8 = Pin 9 -> Pin 7 + Pin 2 (PIA_PA3 -> PIA_PB6 + PIA_PB1) */
static const uint8_t Switch_8_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_8_b = PIA_PA3 | PIA_PB1;

/* Key 9 = Pin 9 -> Pin 6 + Pin 2 (PIA_PA3 -> PIA_PB5 + PIA_PB1) */
static const uint8_t Switch_9_a = PIA_PA3 | PIA_PB5;
static const uint8_t Switch_9_b = PIA_PA3 | PIA_PB1;

/* Key 0 = Pin 9 -> Pin 5 + Pin 2 (PIA_PA3 -> PIA_PB4 + PIA_PB1) */
static const uint8_t Switch_0_a = PIA_PA3 | PIA_PB4;
static const uint8_t Switch_0_b = PIA_PA3 | PIA_PB1;

/* Key : = Pin 9 -> Pin 4 + Pin 2 (PIA_PA3 -> PIA_PB3 + PIA_PB1) */
static const uint8_t Switch_COLON_a = PIA_PA3 | PIA_PB3;
static const uint8_t Switch_COLON_b = PIA_PA3 | PIA_PB1;

/* Key - = Pin 9 -> Pin 8 (PIA_PA3 -> PIA_PB7) */
static const uint8_t Switch_MINUS = PIA_PA3 | PIA_PB7;

/* Key Y = Pin 9 -> Pin 3 + Pin 1 (PIA_PA3 -> PIA_PB0 + PIA_PB2) */
static const uint8_t Switch_Y_a = PIA_PA3 | PIA_PB0;
static const uint8_t Switch_Y_b = PIA_PA3 | PIA_PB2;

/* Key U = Pin 9 -> Pin 3 + Pin 2 (PIA_PA3 -> PIA_PB0 + PIA_PB1) */
static const uint8_t Switch_U_a = PIA_PA3 | PIA_PB0;
static const uint8_t Switch_U_b = PIA_PA3 | PIA_PB1;

/* Key I = Pin 9 -> Pin 7 + Pin 3 (PIA_PA3 -> PIA_PB6 + PIA_PB0) */
static const uint8_t Switch_I_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_I_b = PIA_PA3 | PIA_PB0;

/* Key O = Pin 9 -> Pin 6 + Pin 3 (PIA_PA3 -> PIA_PB5 + PIA_PB0) */
static const uint8_t Switch_O_a = PIA_PA3 | PIA_PB5;
static const uint8_t Switch_O_b = PIA_PA3 | PIA_PB0;

/* Key P = Pin 9 -> Pin 5 + Pin 3 (PIA_PA3 -> PIA_PB4 + PIA_PB0) */
static const uint8_t Switch_P_a = PIA_PA3 | PIA_PB4;
static const uint8_t Switch_P_b = PIA_PA3 | PIA_PB0;

/* Key RET'N = Pin 9 -> Pin 4 + Pin 3 (PIA_PA3 -> PIA_PB3 + PIA_PB0) */
static const uint8_t Switch_RETN_a = PIA_PA3 | PIA_PB3;
static const uint8_t Switch_RETN_b = PIA_PA3 | PIA_PB0;

/* Key H = Pin 9 -> Pin 7 + Pin 1 (PIA_PA3 -> PIA_PB6 + PIA_PB2) */
static const uint8_t Switch_H_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_H_b = PIA_PA3 | PIA_PB2;

/* Key J = Pin 9 -> Pin 6 + Pin 1 (PIA_PA3 -> PIA_PB5 + PIA_PB2) */
static const uint8_t Switch_J_a = PIA_PA3 | PIA_PB5;
static const uint8_t Switch_J_b = PIA_PA3 | PIA_PB2;

/* Key K = Pin 9 -> Pin 5 + Pin 1 (PIA_PA3 -> PIA_PB4 + PIA_PB2) */
static const uint8_t Switch_K_a = PIA_PA3 | PIA_PB4;
static const uint8_t Switch_K_b = PIA_PA3 | PIA_PB2;

/* Key L = Pin 9 -> Pin 4 + Pin 1 (PIA_PA3 -> PIA_PB3 + PIA_PB2) */
static const uint8_t Switch_L_a = PIA_PA3 | PIA_PB3;
static const uint8_t Switch_L_b = PIA_PA3 | PIA_PB2;

/* Key ; = Pin 9 -> Pin 5 + Pin 4 (PIA_PA3 -> PIA_PB4 + PIA_PB3) */
static const uint8_t Switch_SEMICOLON_a = PIA_PA3 | PIA_PB4;
static const uint8_t Switch_SEMICOLON_b = PIA_PA3 | PIA_PB3;

/* Key N = Pin 9 -> Pin 7 + Pin 5 (PIA_PA3 -> PIA_PB6 + PIA_PB4) */
static const uint8_t Switch_N_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_N_b = PIA_PA3 | PIA_PB4;

/* Key M = Pin 9 -> Pin 7 + Pin 4 (PIA_PA3 -> PIA_PB6 + PIA_PB3) */
static const uint8_t Switch_M_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_M_b = PIA_PA3 | PIA_PB3;

/* Key , = Pin 9 -> Pin 6 + Pin 4 (PIA_PA3 -> PIA_PB5 + PIA_PB3) */
static const uint8_t Switch_COMMA_a = PIA_PA3 | PIA_PB5;
static const uint8_t Switch_COMMA_b = PIA_PA3 | PIA_PB3;

/* Key . = Pin 9 -> Pin 7 + Pin 6 (PIA_PA3 -> PIA_PB6 + PIA_PB5) */
static const uint8_t Switch_PERIOD_a = PIA_PA3 | PIA_PB6;
static const uint8_t Switch_PERIOD_b = PIA_PA3 | PIA_PB5;

/* Key / = Pin 9 -> Pin 6 + Pin 5 (PIA_PA3 -> PIA_PB5 + PIA_PB4) */
static const uint8_t Switch_FORWARDSLASH_a = PIA_PA3 | PIA_PB5;
static const uint8_t Switch_FORWARDSLASH_b = PIA_PA3 | PIA_PB4;

/* Key RIGHT ARROW = Pin 10 -> Pin 8 (PIA_PA2 -> PIA_PB7) */
static const uint8_t Switch_RIGHT = PIA_PA2 | PIA_PB7;

/* Key SPACE = Pin 10 -> Pin 4 + Pin 1 (PIA_PA2 -> PIA_PB3 + PIA_PB2) */
static const uint8_t Switch_SPACE_a = PIA_PA2 | PIA_PB3;
static const uint8_t Switch_SPACE_b = PIA_PA2 | PIA_PB2;

/*
 * CreatiVision Left Controller Joystick
 */

/* Up = Pin 2 -> Pin 6 (PIA_PA0 -> PIA_PB3) */
static const uint8_t Switch_JoyL_Up = PIA_PA0 | PIA_PB3;

/* Down = Pin 2 -> Pin 4 (PIA_PA0 -> PIA_PB1) */
static const uint8_t Switch_JoyL_Down = PIA_PA0 | PIA_PB1;

/* Left = Pin 2 + Pin 10 (PIA_PA0 -> PIA_PB5) */
static const uint8_t Switch_JoyL_Left = PIA_PA0 | PIA_PB5;

/* Right = Pin 2 -> Pin 5 (PIA_PA0 -> PIA_PB2) */
static const uint8_t Switch_JoyL_Right = PIA_PA0 | PIA_PB2;

/* Up Left Extra = Pin 2 -> Pin 7 (PIA_PA0 -> PIA_PB4) */
static const uint8_t Switch_JoyL_UpLeft_Extra = PIA_PA0 | PIA_PB4;

/* Up Right & Down Left Extra = Pin 2 -> Pin 9 (PIA_PA0 -> PIA_PB6) */
static const uint8_t Switch_JoyL_UpRightDownLeft_Extra = PIA_PA0 | PIA_PB6;

/* Down Right Extra = Pin 2 -> Pin 3 (PIA_PA0 -> PIA_PB0) */
static const uint8_t Switch_JoyL_DownRight_Extra = PIA_PA0 | PIA_PB0;

/* Button 1 = Pin 2 -> Pin 8 (PIA_PA0 -> PIA_PB7) */
static const uint8_t Switch_JoyL_Button1 = PIA_PA0 | PIA_PB7;

/* Button 2 = Pin 1 -> Pin 8 (PIA_PA1 -> PIA_PB7) */
static const uint8_t Switch_JoyL_Button2 = PIA_PA1 | PIA_PB7;

/**
 * CreatiVision Right Controller Joystick
 */

/* Up = Pin 10 -> Pin 4 (PIA_PA2 -> PIA_PB3) */
static const uint8_t Switch_JoyR_Up = PIA_PA2 | PIA_PB3;

/* Down = Pin 10 -> Pin 2 (PIA_PA2 -> PIA_PB1) */
static const uint8_t Switch_JoyR_Down = PIA_PA2 | PIA_PB1;

/* Left = Pin 10 -> Pin 6 (PIA_PA2 -> PIA_PB5) */
static const uint8_t Switch_JoyR_Left = PIA_PA2 | PIA_PB5;

/* Right = Pin 10 -> + Pin 1 (PIA_PA2 -> PIA_PB2) */
static const uint8_t Switch_JoyR_Right = PIA_PA2 | PIA_PB2;

/* Up Left Extra = Pin 10 -> Pin 5 (PIA_PA2 -> PIA_PB4) */
static const uint8_t Switch_JoyR_UpLeft_Extra = PIA_PA2 | PIA_PB4;

/* Up Right & Down Left Extra = Pin 10 -> Pin 7 (PIA_PA2 -> PIA_PB6) */
static const uint8_t Switch_JoyR_UpRightDownLeft_Extra = PIA_PA2 | PIA_PB6;

/* Down Right Extra = Pin 10 -> Pin 3 (PIA_PA2 -> PIA_PB0) */
static const uint8_t Switch_JoyR_DownRight_Extra = PIA_PA2 | PIA_PB0;

/** Button 1 = Pin 10 -> Pin 8 (PIA_PA2 -> PIA_PB7) */
static const uint8_t Switch_JoyR_Button1 = PIA_PA2 | PIA_PB7;

/** Button 2 - Pin 9 + Pin 8 (PIA_PA3 -> PIA_PB7) */
static const uint8_t Switch_JoyR_Button2 = PIA_PA3 | PIA_PB7;

/**
 * MT8816_Switch turns the Addressed Switch ON or OFF (switchState true/false)
 * NOTE: We also address here (in software) the MT8816 illogical truth table!
 *       Specifically, please note the datasheet Address Decode Truth Table:
 *       "* Switch connections are not in ascending order"  
 *       Yep, FFS! What idiot created this Truth Table design? 
 *       So, the switch statement below returns us to logical X0 - X15 mapping.
 */
static void MT8816_Switch(bool switchState, uint8_t switchAddress)
{
    uint8_t switchAddressX = switchAddress & 0x0F;
    uint8_t switchAddressY = switchAddress & 0x30;
    switch(switchAddressX)
    {
        case 6 ... 11 :
             switchAddressX += 2;
             break;
        case 12 ... 13 :
             switchAddressX -= 6;
            break;
    }
    PORTA.OUT = (switchAddressX | switchAddressY);
    
    if (switchState == true)
        PORTA.OUTSET = MT_Data_bm;
    else
        PORTA.OUTCLR = MT_Data_bm;
        
    PORTA.OUTSET = MT_Strobe_bm;
    /* We could just clear the strobe pin but I like to return the port to 0 */
//    PORTA.OUTCLR = MT_Strobe_bm;
    PORTA.OUT = 0;
}

/**
 * MT8816_Reset resets all used Switches to the OFF state
 * NOTE: This should be entirely unnecessary if an appropriate
 *       hardware reset of the MT8816 is in place!
 *       But, to accommodate a software only reset option (no hardware reset),
 *       this is retained. With a hardware reset in place this then becomes
 *       just a "to be sure" reset. Why not? ;)
 */
static inline void MT8816_Reset(void)
{
    for(uint8_t lp1 = 0; lp1 < 4; lp1++ )
        for(uint8_t lp2 = 0; lp2 < 16; lp2++ )
            MT8816_Switch(false, (lp1<<4) | lp2);
}

/**
 *  Left Joystick uses PORTD PIN2 - PIN7
 *  PORTD definitions:
 *  PIN2 = Up
 *  PIN3 = Down
 *  PIN4 = Left
 *  PIN5 = Right
 *  PIN6 = Button 1
 *  PIN7 = Button 2
 *
 * Read Left Joystick
 * Returns 0 if no Joystick actions are engaged (all switches are off)
 * else 0b00BBRLDU
 * Specifically:
 *  Up           = 0b00xxxxxU
 *  Down         = 0b00xxxxDx
 *  Left         = 0b00xxxLxx
 *  Right        = 0b00xxRxxx
 *  Button 1     = 0b00xBxxxx
 *  Button 2     = 0b00Bxxxxx  
 */
static inline uint8_t readJoystick_Left(void)
{
    uint8_t joyValD;
    
    joyValD = ~(PORTD.IN) & 0xFC;
    
    joyValD = joyValD >> 2;
    return joyValD;
}

/**
 *  Right Joystick uses PORTC PIN0 - PIN3 and PORTD PIN0 - PIN1
 *  PORTC definitions:
 *  PIN0 = Up
 *  PIN1 = Down
 *  PIN2 = Left
 *  PIN3 = Right
 * 
 *  PORTD definitions:
 *  PIN0 = Button 1
 *  PIN1 = Button 2
 *
 * Read Right Joystick
 * Returns 0 if no Joystick actions are engaged (all switches are off)
 * else 0b00BBRLDU
 * Specifically:
 *  Up           = 0b00xxxxxU
 *  Down         = 0b00xxxxDx
 *  Left         = 0b00xxxLxx
 *  Right        = 0b00xxRxxx
 *  Button 1     = 0b00xBxxxx
 *  Button 2     = 0b00Bxxxxx  
 */
static inline uint8_t readJoystick_Right(void)
{
    uint8_t joyValC;
    uint8_t joyValD;

    joyValC = ~(PORTC.IN) & 0x0F;
    joyValD = ~(PORTD.IN) & 0x03;
    
    joyValD = joyValD << 4;
    return joyValC | joyValD;
}

/*
 * process_Joystick_Left reads the current Left Joystick input and if changed,
 *  switches On or Off the required switches to facilitate 8-way Joystick 
 *  switch input for the CreatiVision.
 *  Note that we switch Off any switches that might have been previously On,
 *  before turning On any currently detected On switches!
 */
static inline void process_Joystick_Left(void)
{
    static uint8_t joyLeft_prev = 0;

    uint8_t joyLeft = readJoystick_Left();

    if (joyLeft != joyLeft_prev)
    {
        if (joyLeft & 0x10) 
        {
            MT8816_Switch(true, Switch_JoyL_Button1);
        } else 
        {
            MT8816_Switch(false, Switch_JoyL_Button1);
        }   

        if (joyLeft & 0x20) 
        {
            MT8816_Switch(true, Switch_JoyL_Button2);
        } else 
        {
            MT8816_Switch(false, Switch_JoyL_Button2);
        }   

        switch(joyLeft & 0x0F)
        {
            case 0x01: /* Up */
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_Up);
                break;

            case 0x02: /* Down */
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_Down);
                break;

            case 0x04: /* Left */
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_Left);
                break;

            case 0x08: /* Right */
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_Right);
                break;

            case 0x05: /* Up Left */
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(true, Switch_JoyL_Up);
                MT8816_Switch(true, Switch_JoyL_Left);
                break;

            case 0x09: /* Up Right */
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyL_Up);
                MT8816_Switch(true, Switch_JoyL_Right);
                break;

            case 0x0A: /* Down Right */
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_Down);
                MT8816_Switch(true, Switch_JoyL_Right);
                break;

            case 0x06: /* Down Left */ 
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyL_Down);
                MT8816_Switch(true, Switch_JoyL_Left);
                break;

            default: /* No Joystick Switches are On! */ 
                MT8816_Switch(false, Switch_JoyL_Up);
                MT8816_Switch(false, Switch_JoyL_Down);
                MT8816_Switch(false, Switch_JoyL_Left);
                MT8816_Switch(false, Switch_JoyL_Right);
                MT8816_Switch(false, Switch_JoyL_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyL_DownRight_Extra);
                break;
        }        
        
        joyLeft_prev = joyLeft;
    }        
}

/*
 * process_Joystick_Right reads the current Right Joystick input and if changed,
 *  switches On or Off the required switches to facilitate 8-way Joystick 
 *  switch input for the CreatiVision.
 *  Note that we switch Off any switches that might have been previously On,
 *  before turning On any currently detected On switches!
 */
static inline void process_Joystick_Right(void)
{
    static uint8_t joyRight_prev = 0;

    uint8_t joyRight = readJoystick_Right();

    if (joyRight != joyRight_prev)
    {
        if (joyRight & 0x10) 
        {
            MT8816_Switch(true, Switch_JoyR_Button1);
        } else 
        {
            MT8816_Switch(false, Switch_JoyR_Button1);
        }   

        if (joyRight & 0x20) 
        {
            MT8816_Switch(true, Switch_JoyR_Button2);
        } else 
        {
            MT8816_Switch(false, Switch_JoyR_Button2);
        }   

        switch(joyRight & 0x0F)
        {
            case 0x01: /* Up */
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_Up);
                break;

            case 0x02: /* Down */
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_Down);
                break;

            case 0x04: /* Left */
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_Left);
                break;

            case 0x08: /* Right */
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_Right);
                break;

            case 0x05: /* Up Left */  
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(true, Switch_JoyR_Up);
                MT8816_Switch(true, Switch_JoyR_Left);
                break;

            case 0x09: /* Up Right */ 
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyR_Up);
                MT8816_Switch(true, Switch_JoyR_Right);
                break;

            case 0x0A: /* Down Right */ 
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_Down);
                MT8816_Switch(true, Switch_JoyR_Right);
                break;

            case 0x06: /* Down Left */ 
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                MT8816_Switch(true, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(true, Switch_JoyR_Down);
                MT8816_Switch(true, Switch_JoyR_Left);
                break;

            default: /* No Joystick Switches are On! */ 
                MT8816_Switch(false, Switch_JoyR_Up);
                MT8816_Switch(false, Switch_JoyR_Down);
                MT8816_Switch(false, Switch_JoyR_Left);
                MT8816_Switch(false, Switch_JoyR_Right);
                MT8816_Switch(false, Switch_JoyR_UpLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_UpRightDownLeft_Extra);
                MT8816_Switch(false, Switch_JoyR_DownRight_Extra);
                break;
        }        

        joyRight_prev = joyRight;
    }
}


/* 
 * get_PS2_ScanCode gets a Scan Code byte from the PS2_ScanCodeBuffer
 * Returns 0 if Buffer is empty
 * Atomic block is defined to ensure no interrupt occurs (buffer change),
 * while we are extracting a ScanCode from the buffer!
 */
static inline uint8_t get_PS2_ScanCode(void)
{
	uint8_t value;

	ATOMIC_BLOCK(ATOMIC_FORCEON) 
    {
		if (PS2_ScanCodeBuffer_Start == PS2_ScanCodeBuffer_End)
			return 0;

		value = PS2_ScanCodeBuffer[PS2_ScanCodeBuffer_Start];
        
		if (++PS2_ScanCodeBuffer_Start == PS2_ScanCodeBuffer_Size)
			PS2_ScanCodeBuffer_Start = 0;
	}
	return value;
}

/*
 * process_PS2_ScanCode turns On or Off CreatiVision switches based on
 * appropriate scanCode(s) being returned.
 */
static void process_PS2_ScanCode(void)
{
	static uint8_t key_release = 0;
	static uint8_t extended = 0;

/*
 * Initialize Switch values a & b to No Action!
 */
    uint8_t switchValue_a = NO_SWITCH_ACTION;
    uint8_t switchValue_b = NO_SWITCH_ACTION;

    uint8_t scanCode = get_PS2_ScanCode();

    if (scanCode) {
    
/*
 * Process all ScanCodes that are of interest to us
 */
        switch (scanCode)
        {
/*
 * First check for special action ScanCodes and cache as appropriate flags
 */
            case 0xF0: /* Key release */
                key_release = 1;
                break;

            case 0xE0: /* Extended ScanCode */
            case 0xE1: /* Additional Extended ScanCode */     
                extended = 1;
                break;
/*
 * Then check ScanCodes of interest for the Left Controller Keyboard (24 keys)
 */
            case 0x16 : /* '1' key */
                switchValue_a = Switch_1_a;
                switchValue_b = Switch_1_b;
                break;

            case 0x69 :
                if (extended == 0)
                { /* Keypad '1' key */
                    switchValue_a = Switch_1_a;
                    switchValue_b = Switch_1_b;
                }
                break;

            case 0x1E : /* '2' key */
                switchValue_a = Switch_2_a;
                switchValue_b = Switch_2_b;
                break;

            case 0x72 :
                if (extended == 0)
                { /* Keypad '2' key */
                    switchValue_a = Switch_2_a;
                    switchValue_b = Switch_2_b;
                }
                break;

            case 0x26 : /* '3' key */
                switchValue_a = Switch_3_a;
                switchValue_b = Switch_3_b;
                break;

            case 0x7A :
                if (extended == 0)
                { /* Keypad '3' key */
                    switchValue_a = Switch_3_a;
                    switchValue_b = Switch_3_b;
                }
                break;

            case 0x25 : /* '4' key */
                switchValue_a = Switch_4_a;
                switchValue_b = Switch_4_b;
                break;

            case 0x2E : /* '5' key */
                switchValue_a = Switch_5_a;
                switchValue_b = Switch_5_b;
                break;

            case 0x73 : /* Keypad '5' key */
                switchValue_a = Switch_5_a;
                switchValue_b = Switch_5_b;
                break;

            case 0x36 : /* '6' key */
                switchValue_a = Switch_6_a;
                switchValue_b = Switch_6_b;
                break;

            case 0x15 : /* 'Q' key */
                switchValue_a = Switch_Q_a;
                switchValue_b = Switch_Q_b;
                break;

            case 0x1D : /* 'W' key */
                switchValue_a = Switch_W_a;
                switchValue_b = Switch_W_b;
                break;

            case 0x24 : /* 'E' key */
                switchValue_a = Switch_E_a;
                switchValue_b = Switch_E_b;
                break;

            case 0x2D : /* 'R' key */
                switchValue_a = Switch_R_a;
                switchValue_b = Switch_R_b;
                break;

            case 0x2C : /* 'T' key */
                switchValue_a = Switch_T_a;
                switchValue_b = Switch_T_b;
                break;

            case 0x6B :
                if (extended)
                { /* 'LEFT' key */
                    switchValue_a = Switch_LEFT_a;
                    switchValue_b = Switch_LEFT_b;
                } else 
                { /* Keypad '4' key */
                    switchValue_a = Switch_4_a;
                    switchValue_b = Switch_4_b;
                }
                break;

            case 0x66 : /* 'BKSP' key (also mapped to 'LEFT' Key) */
                switchValue_a = Switch_LEFT_a;
                switchValue_b = Switch_LEFT_b;
                break;

            case 0x1C : /* 'A' key */
                switchValue_a = Switch_A_a;
                switchValue_b = Switch_A_b;
                break;

            case 0x1B : /* 'S' key */
                switchValue_a = Switch_S_a;
                switchValue_b = Switch_S_b;
                break;

            case 0x23 : /* 'D' key */
                switchValue_a = Switch_D_a;
                switchValue_b = Switch_D_b;
                break;

            case 0x2B : /* 'F' key */
                switchValue_a = Switch_F_a;
                switchValue_b = Switch_F_b;
                break;

            case 0x34 : /* 'G' key */
                switchValue_a = Switch_G_a;
                switchValue_b = Switch_G_b;
                break;

            case 0x12 :
                if (extended == 0)
                { /* Left 'SHIFT' key */
                    switchValue_a = Switch_SHIFT;
                }
                break;

            case 0x59 : /* Right 'SHIFT' key */
                switchValue_a = Switch_SHIFT;
                break;

            case 0x1A : /* 'Z' key */
                switchValue_a = Switch_Z_a;
                switchValue_b = Switch_Z_b;
                break;

            case 0x22 : /* 'X' key */
                switchValue_a = Switch_X_a;
                switchValue_b = Switch_X_b;
                break;

            case 0x21 : /* 'C' key */
                switchValue_a = Switch_C_a;
                switchValue_b = Switch_C_b;
                break;

            case 0x2A : /* 'V' key */
                switchValue_a = Switch_V_a;
                switchValue_b = Switch_V_b;
                break;

            case 0x32 : /* 'B' key */
                switchValue_a = Switch_B_a;
                switchValue_b = Switch_B_b;
                break;

            case 0x14 : /* Left or Right 'CTRL' key */
                switchValue_a = Switch_CNTL;
                break;
/*
 * Then check ScanCodes of interest for the Right Controller Keyboard (24 keys)
 */
            case 0x3D : /* '7' key */
                switchValue_a = Switch_7_a;
                switchValue_b = Switch_7_b;
                break;

            case 0x6C :
                if (extended == 0)
                { /* Keypad '7' key */
                    switchValue_a = Switch_7_a;
                    switchValue_b = Switch_7_b;
                }
                break;

            case 0x3E : /* '8' key */
                switchValue_a = Switch_8_a;
                switchValue_b = Switch_8_b;
                break;

            case 0x75 :
                if (extended == 0)
                { /* Keypad '8' key */
                    switchValue_a = Switch_8_a;
                    switchValue_b = Switch_8_b;
                }
                break;

            case 0x46 : /* '9' key */
                switchValue_a = Switch_9_a;
                switchValue_b = Switch_9_b;
                break;

            case 0x7D :
                if (extended == 0)
                { /* Keypad '9' key */
                    switchValue_a = Switch_9_a;
                    switchValue_b = Switch_9_b;
                }
                break;

            case 0x45 : /* '0' key */
                switchValue_a = Switch_0_a;
                switchValue_b = Switch_0_b;
                break;

            case 0x70 :
                if (extended == 0)
                { /* Keypad '0' key */
                    switchValue_a = Switch_0_a;
                    switchValue_b = Switch_0_b;
                }
                break;

            case 0x52 : /* ':' key - NOTE: Mapped to PS/2 Keyboard ' key */
                switchValue_a = Switch_COLON_a;
                switchValue_b = Switch_COLON_b;
                break;

            case 0x4E : /* '-' key */
                switchValue_a = Switch_MINUS;
                break;

            case 0x7B : /* Keypad '-' key */
                switchValue_a = Switch_MINUS;
                break;

            case 0x35 : /* 'Y' key */
                switchValue_a = Switch_Y_a;
                switchValue_b = Switch_Y_b;
                break;

            case 0x3C : /* 'U' key */
                switchValue_a = Switch_U_a;
                switchValue_b = Switch_U_b;
                break;

            case 0x43 : /* 'I' key */
                switchValue_a = Switch_I_a;
                switchValue_b = Switch_I_b;
                break;

            case 0x44 : /* 'O' key */
                switchValue_a = Switch_O_a;
                switchValue_b = Switch_O_b;
                break;

            case 0x4D : /* 'P' key */
                switchValue_a = Switch_P_a;
                switchValue_b = Switch_P_b;
                break;

            case 0x5A : /* Keypad or regular 'ENTER' key */
                switchValue_a = Switch_RETN_a;
                switchValue_b = Switch_RETN_b;
                break;

            case 0x33 : /* 'H' key */
                switchValue_a = Switch_H_a;
                switchValue_b = Switch_H_b;
                break;

            case 0x3B : /* 'J' key */
                switchValue_a = Switch_J_a;
                switchValue_b = Switch_J_b;
                break;

            case 0x42 : /* 'K' key */
                switchValue_a = Switch_K_a;
                switchValue_b = Switch_K_b;
                break;

            case 0x4B : /* 'L' key */
                switchValue_a = Switch_L_a;
                switchValue_b = Switch_L_b;
                break;

            case 0x4C : /* ';' key */
                switchValue_a = Switch_SEMICOLON_a;
                switchValue_b = Switch_SEMICOLON_b;
                break;

            case 0x31 : /* 'N' key */
                switchValue_a = Switch_N_a;
                switchValue_b = Switch_N_b;
                break;

            case 0x3A : /* 'M' key */
                switchValue_a = Switch_M_a;
                switchValue_b = Switch_M_b;
                break;

            case 0x41 : /* ',' key */
                switchValue_a = Switch_COMMA_a;
                switchValue_b = Switch_COMMA_b;
                break;

            case 0x49 : /* '.' key */
                switchValue_a = Switch_PERIOD_a;
                switchValue_b = Switch_PERIOD_b;
                break;

            case 0x71 :
                if (extended == 0)
                { /* Keypad '.' key */
                    switchValue_a = Switch_PERIOD_a;
                    switchValue_b = Switch_PERIOD_b;
                }
                break;

            case 0x4A : /* Keypad or regular '/' key */
                switchValue_a = Switch_FORWARDSLASH_a;
                switchValue_b = Switch_FORWARDSLASH_b;
                break;

            case 0x74 :
                if (extended)
                { /* 'RIGHT' key */
                    switchValue_a = Switch_RIGHT;
                } else 
                { /* Keypad '6' key */
                    switchValue_a = Switch_6_a;
                    switchValue_b = Switch_6_b;
                }
                break;

            case 0x29 : /* 'SPACE' key */
                switchValue_a = Switch_SPACE_a;
                switchValue_b = Switch_SPACE_b;
                break;
/*
 * Just ignore all other key ScanCodes that aren't of interest to us!
 * But, we still need to clear the flags (for any other key)
 */
            default:
                key_release = 0;
                extended = 0;
        }
        
/* 
 * Did we have a key press (or release) ScanCode of interest?
 */
        if ((switchValue_a != NO_SWITCH_ACTION) || (switchValue_b != NO_SWITCH_ACTION))
        {
            if (switchValue_a != NO_SWITCH_ACTION)
            {
                if (key_release)
                {
                    MT8816_Switch(false, switchValue_a);
                }
                else
                {
                    MT8816_Switch(true, switchValue_a);
                }    
            }    
            if (switchValue_b != NO_SWITCH_ACTION)
            {
                if (key_release)
                {
                    MT8816_Switch(false, switchValue_b);
                }    
                else
                {
                    MT8816_Switch(true, switchValue_b);
                }    
            }

            /* After a valid key press ScanCode, we can clear the flags! */
            key_release = 0;
            extended = 0;
        }
    }
}                                               

/*
 * PS2 Keyboard Input - INTERRUPT SERVICE ROUTINE!
 * Interrupt is called on falling edge of PS/2 Clock signal
 */
void PS2_Interrupt(void)
{ 
	static uint8_t startBit = 0;
	static uint8_t data;
    static uint8_t parityCount = 0;
	static uint8_t stopBit = 0;
	static uint8_t bitCount = 0;

    uint8_t thisBit = PORTF.IN & PS2_Data_bm;

    bitCount++;
    switch (bitCount) 
    {
        case 1:
            startBit = thisBit;
            break;

        case 2 ... 9 :
    		data = data >> 1;      
        	if (thisBit)    /* read PS2_Data Pin */
            {    
    			data = data | 0x80; /* Set bit (1) */
                parityCount++;
            }    
            else
    			data = data & 0x7f; /* Clear bit (0) */
            break;

        case 10:
    		if (thisBit)    /* read PS2_Data Pin */
                parityCount++;
            break;

        case 11:
            stopBit = thisBit;
            break;
    }

	if (bitCount > 10) 
    {
		/* If all bits now received, check valid start, stop and parity bits */
        if ((parityCount % 2) && !(startBit) && (stopBit))
        {
    		/* If valid ScanCode, add to Buffer */
            PS2_ScanCodeBuffer[PS2_ScanCodeBuffer_End] = data;

            if (++PS2_ScanCodeBuffer_End == PS2_ScanCodeBuffer_Size)
                PS2_ScanCodeBuffer_End = 0;

            /* If buffer is now full, drop oldest value */
            if ((PS2_ScanCodeBuffer_End == PS2_ScanCodeBuffer_Start)
                && (++PS2_ScanCodeBuffer_Start == PS2_ScanCodeBuffer_Size))
                    PS2_ScanCodeBuffer_Start = 0;
        }
        parityCount = 0;
		bitCount = 0;
	}
}

/*
 * Main Application
 */
int main(void)
{
  
    /* MCC defined System Setup (initialize) */
    SYSTEM_Initialize();

    /* Software Reset all the MT8816 switches to OFF */
    MT8816_Reset();   
    
    /* Setup PS/2 Keyboard Interrupt handler routine */
    IO_PF0_SetInterruptHandler(PS2_Interrupt);
    
    /* Let's do this forever! */
    while(1)
    {

        process_Joystick_Left();
        
        process_Joystick_Right();
 
        process_PS2_ScanCode();

        /* Yep, that's it. :) */
        
    }
    return 0;
}