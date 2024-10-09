/*===================================CPEG222====================================
 * Program: Proj2_Template.c
 * Authors: Raphael Daluz, Mekhai Waples
 * Date:  10/4/2024
 * Description: This template uses an on-board BTN to control
 * the alarm, turning it on and off, and the SSD, which will display
 * the number of times btnC is pressed.
 * Output: All LEDs are turned on and off.
==============================================================================*/
/*---- Board system settings. PLEASE DO NOT MODIFY THIS PART FOR PROJECT 1 ---*/
#ifndef _SUPPRESS_PLIB_WARNING //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider 
                                //(PLL Divide by 1)
#pragma config FNOSC = PRIPLL   // Oscillator Selection Bits (Primary Osc w/PLL 
                                //(XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF    // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT     // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8   // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#include <xc.h>     //Microchip XC processor header which links to the 
                    //PIC32MX370512L header
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // NOTE: utils.c and utils.h must also be in your project 
                    //to use lcd.c
#include"ssd.h"
/* --------------------------- Forward Declarations-------------------------- */
void initialize_ports();
void initialize_output_states();
void handle_button_presses();
void delay_ms(int milliseconds);
void turnOnAlarm(void) ;
void turnOffAlarm(void) ;
void set_time(void);
void run_clock(void);
void setting_algorithm(void);
void ticking_algorithm(void);
void set_alarm();
/* ------------------------ Constant Definitions ---------------------------- */
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))
#define BtnC_RAW PORTFbits.RF0
#define BtnL_RAW PORTBbits.RB0
#define BtnR_RAW PORTBbits.RB8
#define BtnU_RAW PORTBbits.RB1
#define BtnD_RAW PORTAbits.RA15
#define TRUE 1
#define FALSE 0
#define BUTTON_DEBOUNCE_DELAY_MS 20

/***** This section contains variables for the speaker ******/
#define TMR_FREQ_SINE   48000 // 48 kHz
// This array contains the values that implement one syne period, over 25 samples. 
// They are generated using this site: 
// http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
unsigned short rgSinSamples [] = {
256,320,379,431,472,499,511,507,488,453,
406,350,288,224,162,106, 59, 24,  5,  1,
 13, 40, 81,133,192};

#define RGSIN_SIZE  (sizeof(rgSinSamples) / sizeof(rgSinSamples[0]))

// the array of samples, to be stored when recording (Mode 2) and to be played when playing back (Mode 3).
unsigned short *pAudioSamples;

// global variables that store audio buffer position and size
int cntAudioBuf, idxAudioBuf;
/***** End of speaker declararions  ******/  // shouldn't touch

//Mode 1 = SET_TIMER, Mode 3 = TIMER_TICKS
typedef enum {SET_TIMER, SET_ALARM, TIMER_TICKS, ALARMING} eModes ;

/* -------------------- Global Variable Declarations ------------------------ */
char buttonsLockedC = FALSE;
char pressedUnlockedBtnC = FALSE;
char buttonsLockedU = FALSE;
char pressedUnlockedBtnU = FALSE;
char buttonsLockedD = FALSE;
char pressedUnlockedBtnD = FALSE;
char buttonsLockedL = FALSE;
char pressedUnlockedBtnL = FALSE;
char buttonsLockedR = FALSE;
char pressedUnlockedBtnR = FALSE;
eModes mode = SET_TIMER ;
char min1 = 0;
char min2 = 0;
char sec1 = 0;
char sec2 = 0;
int selection = 1;
int counter=0;
/* ----------------------------- Main --------------------------------------- */
int main(void)
{
    /*-------------------- Port and State Initialization -------------------------
*/
    initialize_ports();
    initialize_output_states();
    
    while (TRUE)
    {
        /*-------------------- Main logic and actions start 
--------------------------*/
        handle_button_presses();
        //Mode 1, Set the start time.
        if (mode==SET_TIMER) {
            set_time();
            
            if (pressedUnlockedBtnC) {
                mode = SET_ALARM;
            }
        }
        else if (mode==SET_ALARM) {
            set_alarm();
            
        }
        //Mode 3, Clock is counting up.
        else if (mode==TIMER_TICKS) {
            run_clock();
            if (pressedUnlockedBtnC) {
                mode = SET_TIMER;
            }    
        }
        SSD_WriteDigits(sec1,sec2,min1,min2,0,0,0,0);//think about how to edit this
    }
}
void initialize_ports()
{
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO

    /* The following line sets the tristate of Port F bit 0 to 1.
     *  BtnC is connected to that pins. When the tristate of a pin is set high,
     *  the pin is configured as a digital input. */
    TRISFbits.TRISF0 = 1;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB8 = 1;
    TRISAbits.TRISA15 = 1;
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB8 = 0;
    
    LCD_Init(); // A library function provided by Digilent
    SSD_Init();    //SSD Init

    // the following lines configure interrupts to control the speaker
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
        /* The following code sets up the alarm timer and interrupts */
    tris_A_OUT = 0;    
    rp_A_OUT = 0x0C; // 1100 = OC1
        // disable analog (set pins as digital)
    ansel_A_OUT = 0;
    
    T3CONbits.TCKPS = 0;     //1:1 prescale value
    T3CONbits.TGATE = 0;     //not gated input (the default)
    T3CONbits.TCS = 0;       //PCBLK input (the default)
    
    OC1CONbits.ON = 0;       // Turn off OC1 while doing setup.
    OC1CONbits.OCM = 6;      // PWM mode on OC1; Fault pin is disabled
    OC1CONbits.OCTSEL = 1;   // Timer3 is the clock source for this Output Compare module
    
    IPC3bits.T3IP = 7;      // interrupt priority
    IPC3bits.T3IS = 3;      // interrupt subpriority
    
    macro_enable_interrupts();  // enable interrupts at CPU

}
void initialize_output_states()
{

    LCD_WriteStringAtPos("    GROUP 8     ", 0, 0); //Display "Welcome" at line 0,
    if (mode==SET_TIMER) {
        LCD_WriteStringAtPos("   Set Time?   ", 1, 0); //Display "Set Time?" at line 1
    }
    
    else if (mode==TIMER_TICKS){
        LCD_WriteStringAtPos("   Display Time   ", 1, 0); //Display "Press BtnC" at line 1
    }
    
    else {
        LCD_WriteStringAtPos("   sum ting wong   ", 1, 0);
    }
 
    turnOffAlarm();
}

/* This below function turns on the alarm*/
void turnOnAlarm()
{
    //set up alarm
    PR3 = (int)((float)((float)PB_FRQ/TMR_FREQ_SINE) + 0.5);               
    idxAudioBuf = 0;
    cntAudioBuf = RGSIN_SIZE;
    pAudioSamples = rgSinSamples;

    // load first value
    OC1RS = pAudioSamples[0];
    TMR3 = 0;

    T3CONbits.ON = 1;        //turn on Timer3
    OC1CONbits.ON = 1;       // Start the OC1 module  
    IEC0bits.T3IE = 1;      // enable Timer3 interrupt    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}
/* This below function turns off the alarm*/
void turnOffAlarm()
{
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
}
/* The below function only handles BtnC presses. Think about how it could be
 copied to handle all button presses.*/ 
void handle_button_presses()
{
    pressedUnlockedBtnC = FALSE;
    pressedUnlockedBtnU = FALSE;
    pressedUnlockedBtnD = FALSE;
    pressedUnlockedBtnL = FALSE;
    pressedUnlockedBtnR = FALSE;
//Handles Button C
    if (BtnC_RAW && !buttonsLockedC)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = TRUE;
        pressedUnlockedBtnC = TRUE;
    }
    else if (!BtnC_RAW && buttonsLockedC)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = FALSE;
    }
//Handles Button U
    else if (BtnU_RAW && !buttonsLockedU)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedU = TRUE;
        pressedUnlockedBtnU = TRUE;
    }
    else if (!BtnU_RAW && buttonsLockedU)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedU = FALSE;
    }
//Handles Button D
    else if (BtnD_RAW && !buttonsLockedD)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedD = TRUE;
        pressedUnlockedBtnD = TRUE;
    }
    else if (!BtnD_RAW && buttonsLockedD)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedD = FALSE;
    }
//Handles Button L
    else if (BtnL_RAW && !buttonsLockedL)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedL = TRUE;
        pressedUnlockedBtnL = TRUE;
    }
    else if (!BtnL_RAW && buttonsLockedL)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedL = FALSE;
    }
//Handles Button R
    else if (BtnR_RAW && !buttonsLockedR)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedR = TRUE;
        pressedUnlockedBtnR = TRUE;
    }
    else if (!BtnR_RAW && buttonsLockedR)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedR = FALSE;
    }
}
void delay_ms(int milliseconds)
{
    int i;
    for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++) 
    {}
}

/* ------------------------------------------------------------ */
/***	Timer3ISR
**
**	Description:
**		This is the interrupt handler for Timer3. According to each mode, it is called at specific frequencies, as initialized in AUDIO_Init.
    Mode 0 (Generate sound using sine) - Advance current index in the sine definition buffer, initialize OC1 with the current sine definition value.         
*/
void __ISR(_TIMER_3_VECTOR, IPL7AUTO) Timer3ISR(void) 
{  
   // play sine
    // load sine value into OC register
    OC1RS = 4*pAudioSamples[(++idxAudioBuf) % cntAudioBuf];
    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}

/**In this template no RGB code is provided. But the steps to use it are the same as the steps to use 
 * another aspect of the board which we do provide. Check rgb.c and rgb.h as well as the functions we give you
 * to see what you need to add to use the RGB.
**/

void setting_algorithm(void) { //Algorithm that causes digits to flip when necessary
    //maximum seconds
    if (sec2>5) {
        sec2=0;
        sec1=0;
    }
    //minimum seconds
    if (sec2<0) {
        sec2=5;
        sec1=9;
    }
    //maximum minutes
    if (min2==2 && min1>3) {
        min2=0;
        min1=0;
    }
    //minimum minutes
    if (min2<0) {
        min2=2;
        min1=3;
    }
    //minutes wrap up
    if (min1>9){
        min1=0;
        min2++;
    }
    //minutes wrap down
    if (min1<0) {
        min1=9;
        min2--;
    }
    //seconds wrap up
    if (sec1>9) {
        sec1=0;
        sec2++;
    }
    //seconds wrap down
    if (sec1<0) {
        sec1=9;
        sec2--;
    }
};

void ticking_algorithm(void) {
    //resets if clock hit maximum, 24:00 -> 00:00
    if (min2 == 2 && min1 == 4) {
        min2 = 0;
        min1 = 0;
        sec2 = 0;
        sec1 = 0;
    }
    //seconds count up
    if (sec1 > 9) {
        sec1 = 0;
        sec2++ ;
    }  
    if (sec2 > 5) {
        sec2 = 0;
        min1++ ;
    }
        
    //minutes count up
    if (min1 > 9) {
        min1 = 0;
        min2++ ;
    }

}

void set_time(void) {
    LCD_WriteStringAtPos("   Set Time?   ", 1, 0); //Display "Set Time?" at line 1
    //Handles "SET_TIMER" mode
    //change selection, 1 is seconds, 2 is minutes
    if (pressedUnlockedBtnL) {
        if (selection == 1) {
            selection = 2;
        }
        else if (selection == 2) {
            selection = 1;
        }
    }
    //reset button
    if (pressedUnlockedBtnR) {
        sec1 = 0;
        sec2 = 0;
        min1 = 0;
        min2 = 0;
    }
    //check selection
    if (selection == 1) {
        if(pressedUnlockedBtnU) {
            sec1++;
        }
        else if (pressedUnlockedBtnD) {
            sec1--;
        }
    }
    else if (selection == 2) {
        if(pressedUnlockedBtnU) {
            min1++;
        }
        else if (pressedUnlockedBtnD) {
            min1--;
        }
    }
    setting_algorithm();
}
    
void run_clock(void) {
    //Handles "TIMER_TICKS" mode
    LCD_WriteStringAtPos("   Display Time   ", 1, 0); //Display "Press BtnC" at line 1
    ticking_algorithm();
    if (counter>=180) {
        sec1++;
        counter=0;
    }
    counter++;
}

void set_alarm(void) {
    //sets time values that alarm should beep on
}