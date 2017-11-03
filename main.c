/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "temp.h"
static uint8_t barPos = 2;

volatile uint32_t msTicks = 0;
uint8_t oled_disp[40] = {};
int myintsw3 = 0;

//button for switching mode
volatile int MODE_TOGGLE= 0, MODE_TOGGLE1 = 0;

//SYS Modes
typedef enum {
	MODE_STAT, MODE_FOR, MODE_REV
} system_mode_t;
volatile system_mode_t mode;

static void moveBar(uint8_t steps, uint8_t dir)
{
    uint16_t ledOn = 0;

    if (barPos == 0)
        ledOn = (1 << 0) | (3 << 14);
    else if (barPos == 1)
        ledOn = (3 << 0) | (1 << 15);
    else
        ledOn = 0x07 << (barPos-2);

    barPos += (dir*steps);
    barPos = (barPos % 16);

    pca9532_setLeds(ledOn, 0xffff);
}


static void drawOled(uint8_t joyState)
{
    static int wait = 0;
    static uint8_t currX = 48;
    static uint8_t currY = 32;
    static uint8_t lastX = 0;
    static uint8_t lastY = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_clearScreen(OLED_COLOR_BLACK);
        return;
    }

    if (wait++ < 3)
        return;

    wait = 0;

    if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
        currY--;
    }

    if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
        currY++;
    }

    if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
        currX++;
    }

    if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
        currX--;
    }

    if (lastX != currX || lastY != currY) {
        oled_putPixel(currX, currY, OLED_COLOR_WHITE);
        lastX = currX;
        lastY = currY;
    }
}


#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);




static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;

    /* number of ms */

    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}

static uint8_t * song = (uint8_t*) "E2,D2,C2,D2,E2,E2,E4.D2,D2,D4,E2,G2,G4.E2,D2,C2,D2,E2,E2,E3.E2,D2,D2,E2,D2,C4,";
		//"C2.C2,D4,C4,F4,E8,";
        //(uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";



static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);


	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// Initialize button
	/* Line 261 */	PINSEL_CFG_Type PinCfg;
	/* Line 262 */
						//Initialize SW4
	/* Line 263 */	    PinCfg.Funcnum = 0;
	/* Line 264 */	    PinCfg.OpenDrain = 0;
	/* Line 265 */	    PinCfg.Pinmode = 0;
//	/* Line 266 */	    PinCfg.Portnum = 1;
//	/* Line 267 */	    PinCfg.Pinnum = 31;
//	/* Line 269 */		PINSEL_ConfigPin(&PinCfg);
//						GPIO_SetDir(1, 1 << 31, 0);
	/* Line 271 */

	//Initialize button sw3 (Interrupt)
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 4;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<4, 0);

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);//rotary switch
	GPIO_SetDir(0, 1 << 25, 0);
}

void SysTick_Handler (void){
	msTicks++;

}
uint32_t getTicks(void){
	return msTicks;
}

int Timer(uint32_t startTicks, int delayInMs){
	return (getTicks() - startTicks) >= delayInMs;
}

void EINT3_IRQHandler (void){
	//printf("EINT3\n");
//	myintsw3 = 1;
//	printf("%d\n", myintsw3);
	if((LPC_GPIOINT->IO0IntStatF)>>4 & 1){
		MODE_TOGGLE= 1;
//		if (MODE_TOGGLE) {
//			MODE_TOGGLE1 = 1;
//		}
		printf("sw3EINT\n");
		LPC_GPIOINT->IO0IntClr = 1<<4;
		printf("%d", MODE_TOGGLE);
	}

	if ((LPC_GPIOINT->IO0IntStatF)>>25 & 1){
		printf("P0.25 has been rotated \n");
		LPC_GPIOINT->IO0IntClr = 1<<25;
	}
	printf("Exit OK\n");
}
//void TOGGLE (void){
//	MODE_TOGGLE = MODE_TOGGLE1 + MODE_TOGGLE;
//	MODE_TOGGLE1 = 0;
//			if(MODE_TOGGLE = 3){
//				MODE_TOGGLE = 0;
//			}
//
//}

void ready_uart(void){
	// PINSEL Configuration
	PINSEL_CFG_Type CPin;
	    CPin.OpenDrain = 0;
	    CPin.Pinmode = 0;
	    CPin.Funcnum = 2;
	    CPin.Pinnum = 0;
	    CPin.Portnum = 0;
	PINSEL_ConfigPin(&CPin);
	    CPin.Pinnum = 1;
	    CPin.Portnum = 0;
	PINSEL_ConfigPin(&CPin);

	// Initialise and enable the UART. Not enabling the UART will lead to a hard fault
	UART_CFG_Type UCfg;
	    UCfg.Baud_rate = 115200;
	    UCfg.Databits = UART_DATABIT_8;
	    UCfg.Parity = UART_PARITY_NONE;
	    UCfg.Stopbits = UART_STOPBIT_1;

	// supply power & setup working parameters for UART3
	UART_Init(LPC_UART3, &UCfg);

	// enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);

	// FIFO configuration- For system enhancements only
	//
}

int main (void) {


    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    int8_t x = 0;

    int8_t y = 0;
    int8_t z = 0;
    uint8_t dir = 1;
    uint8_t wait = 0;

    uint8_t state    = 0;

    uint8_t btnSW4 = 1;
    uint8_t btnSW3 = 1;
    uint8_t btn1 = 1;

    uint32_t startTicks = getTicks();
    uint32_t rgbTimer = startTicks, modeTimer = startTicks;


    init_i2c();
    init_ssp();
    init_GPIO();

    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
    led7seg_init();
    light_enable();
    rgb_init();

	SysTick_Config(SystemCoreClock/1000);
    temp_init(&getTicks);

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;

    /* ---- Speaker ------> */

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    /* <---- Speaker ------ */

    moveBar(1, dir);
    oled_clearScreen(OLED_COLOR_BLACK);
	uint32_t my_light_value;
	int32_t my_temp_value;

	/* <---- UART ----- */
	static char* msg = NULL;
	uint8_t data = 0;
	uint32_t len = 0;
	uint8_t line[64];
	uint8_t line_count = 0;

	ready_uart();

	LPC_GPIOINT->IO0IntEnF |= 1 << 4;
	LPC_GPIOINT->IO0IntEnF |= 1 <<25;

	NVIC_EnableIRQ (EINT3_IRQn);

	void reset() {
		led7seg_setChar('{',FALSE); // Clear 7SEG display
		pca9532_setLeds(0, 0xffff); // Clear floodlights
		GPIO_ClearValue( 2, 1); // Clear red
		GPIO_ClearValue( 0, (1<<26) ); // Clear blue
		oled_clearScreen(OLED_COLOR_BLACK); // cls
	}
	rgb_setLeds(RGB_RED);

    while (1)
    {
    	/* <---- UART ----- */
    	// Send a string message from the LPC1769 to the UART terminal
    	msg = "Welcome to EE2024 \r\n";
    	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);



    	// LPC1769 receives a character from the UART terminal, and then sends back that character to the UART terminal
//    	UART_Receive(LPC_UART3, &data, 1, BLOCKING);
//    	UART_Send(LPC_UART3, &data, 1, BLOCKING);



    	// LPC1769 receives a string ended by a return carriage from the UART terminal, and then sends back that string to the UART terminal
//    	do
//    	{
//    		UART_Receive(LPC_UART3, &data, 1, BLOCKING);
//    		if (data != '\r')
//    		{
//    			len++;
//    			line[len-1] = data;
//    		}
//    	}
//    	while ((len<64) && (data != '\r'));
//
//    	line[len]=0;
//    	UART_SendString(LPC_UART3, &line);
//    	printf("--%s--\n", line);
//    	for (line_count = 0; line_count <64; line_count++)
//    	{
//    		line[line_count] = ' ';
//    	}
//    	len = 0;

    	my_temp_value = temp_read();
    	//printf("The temp value is %2.2f \n",my_temp_value/10.0);

        my_light_value = light_read();
        //printf("The light value is %u \n",my_light_value);
    	//printf("%d \n", myintsw3); //test sw3 interrupt

        /* ####### Accelerometer and LEDs  ###### */
        /* # */

        acc_read(&x, &y, &z);
        x = x+xoff;
        y = y+yoff;
        z = z+zoff;

        if (y < 0) {
            dir = 1;
            y = -y;
        }
        else {
            dir = -1;
        }

        if (y > 1 && wait++ > (40 / (1 + (y/10)))) {
            moveBar(1, dir);
            wait = 0;
        }


        /* # */
        /* ############################################# */


        /* ####### Joystick and OLED  ###### */
        /* # */

        state = joystick_read();
        if (state != 0)
            drawOled(state);

        /* # */
        /* ############################################# */

        /* Line 371 */ 	btnSW4 = (GPIO_ReadValue(1) >> 31) & 0x01;
        				btnSW3 = (GPIO_ReadValue(0) >> 4) & 0x01;
        				btn1 = (GPIO_ReadValue(2) >> 10) & 0x01;

        sprintf(oled_disp, "Gforce: %.1f", x/9.8);
        oled_putString(10,10,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        sprintf(oled_disp, "Temp: %2.2f", my_temp_value/10.0 );
        oled_putString(10,20,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        sprintf(oled_disp, "Light: %u", my_light_value);
        oled_putString(10,30,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//
//        /* ############ Trimpot and RGB LED  ########### */
//        /* # */
//
//        //led7seg_setChar('9',FALSE);
//        if (btnSW3 == 0)
//        {
//        	pca9532_setLeds(0xAA00,0xFFFF);
//        	led7seg_setChar('1',FALSE);
//        	//GPIO_SetValue( 2, (1 << 1) );
//        	//rgb_setLeds(RGB_RED);
//        	//GPIO_SetValue( 2, (1 << 1) );
//
//        } else {
//        	pca9532_setLeds(0x00AA,0xFFFF);
//        	//rgb_setLeds(RGB_BLUE);
//        	//GPIO_SetValue( 2, (1 << 1) );
//        	led7seg_setChar('0',FALSE);
//        }

//    	if (Timer(rgbTimer, 333)){
//    		rgb_setLeds(RGB_RED);
//    	}
//    	if (Timer(rgbTimer, 666)){
//    		GPIO_ClearValue(2,1);
//    		rgbTimer=getTicks();
//    	}

    	/* ############ MODES  ########### */
//        if (MODE_TOGGLE) {
//        	if (MODE_TOGGLE1 == 0) {
//        		printf("test%d/n", MODE_TOGGLE1);
//        		MODE_TOGGLE1 = 1;
//        		startTicks = getTicks();
//        	}
//
//        	}
//        }

//        if (Timer(modeTimer, 1000)) {
//        	if(MODE_TOGGLE1){
//        	if(mode == MODE_STAT) {
//        		mode = MODE_REV;
//        	}
//        	//(MODE_TOGGLE) = 0;
//        	}
//        	if(MODE_TOGGLE++){
//        		if(mode == MODE_STAT) {
//        			mode = MODE_FOR;
//        			} else if (mode == MODE_FOR) {
//        				mode = MODE_STAT;
//        				} else if (mode == MODE_REV) {
//        					mode = MODE_STAT;
//        			        	}
//        		}

//        	modeTimer = getTicks();
//        	(MODE_TOGGLE) = 0;
//        }
//
//    	switch (mode) {
//    	case MODE_STAT	:
//    		reset();
//    		break;
//
//    	case MODE_FOR:
//    		rgb_setLeds(RGB_RED);
//    		break;
//
//    	case MODE_REV:
//    		rgb_setLeds(RGB_BLUE);
//    		break;
//
//    	}

//        rgb_setLeds(RGB_RED);

        Timer0_Wait(1);
    }


}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

