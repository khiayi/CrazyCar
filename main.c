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
volatile uint32_t modeTimer = 0;
uint8_t oled_disp[40] = {};
int myintsw3 = 0;

//button for switching mode
volatile int MODE_TOGGLE= 0, MODE_TOGGLE1 = 0;

//SYS Modes
typedef enum {
	MODE_STAT, MODE_FOR, MODE_REV
} system_mode_t;
volatile system_mode_t mode;

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
	if((LPC_GPIOINT->IO0IntStatF)>>4 & 1){	//sw3
		if (MODE_TOGGLE1 == 0) {
			MODE_TOGGLE1 = 1;
			MODE_TOGGLE = 1;
			modeTimer = getTicks();
		} else{
			MODE_TOGGLE = 2;
		}

		LPC_GPIOINT->IO0IntClr = 1<<4;     	//clearing interrupt
	}

	if ((LPC_GPIOINT->IO0IntStatF)>>25 & 1){	//p0.25 rotation
		LPC_GPIOINT->IO0IntClr = 1<<25;
	}
}

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
static char ssg[] = {
        '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};
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
    uint32_t rgbTimer = startTicks;

    //7seg disp initialisation
    uint32_t ForTimer = startTicks, RevTimer = startTicks;
    int ssegdisp=0;
    int RGB = 0;

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
		ssegdisp = 0; //reset display
		pca9532_setLeds(0, 0xffff); // Clear floodlights

		GPIO_ClearValue( 2, 1); // Clear red
		GPIO_ClearValue( 0, (1<<26) ); // Clear blue
		oled_clearScreen(OLED_COLOR_BLACK); // cls
	}

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


//
//        /* ############ Trimpot and RGB LED  ########### */
//        /* # */


    	/* ############ MODES  ########### */

        if (MODE_TOGGLE1 && Timer(modeTimer, 1000)) {
        	if(MODE_TOGGLE == 1) {
        		if(mode == MODE_STAT) {
               		mode = MODE_FOR;
               		reset();
               		ForTimer = getTicks();
        			MODE_TOGGLE1 = 0;
        			MODE_TOGGLE = 0;
        		} else if (mode == MODE_FOR) {
        			mode = MODE_STAT;
        			reset();
               		MODE_TOGGLE1 = 0;
               		MODE_TOGGLE = 0;
        		} else if (mode == MODE_REV){
        			mode = MODE_STAT;
        			reset();
        			MODE_TOGGLE1 = 0;
        			MODE_TOGGLE = 0;
        		}
        	}

        	if(MODE_TOGGLE == 2) {
        		if(mode == MODE_STAT) {
        			mode = MODE_REV;
        			reset();
        			RevTimer = getTicks();
        			MODE_TOGGLE1 = 0;
        			MODE_TOGGLE = 0;
        		} else {
        			mode = MODE_STAT;
        			reset();
        			MODE_TOGGLE1 = 0;
        			MODE_TOGGLE = 0;
        		}
        	}

        	}
//		sprintf(oled_disp, "Gforce: %.1f", x/9.8);
//		oled_putString(10,10,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//		sprintf(oled_disp, "Temp: %2.2f", my_temp_value/10.0 );
//		oled_putString(10,20,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//		sprintf(oled_disp, "Light: %u", my_light_value);
//		oled_putString(10,30,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	switch (mode) {
    	case MODE_STAT	:
    		sprintf(oled_disp, "Stationary");
    		oled_putString(10,10,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    		break;

    	case MODE_FOR:
    		sprintf(oled_disp, "Forward");
    		oled_putString(10,10,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		rgb_setLeds(RGB_RED);
    		if(Timer(rgbTimer,333)){
    			if(RGB == 0){
    				rgb_setLeds(RGB_BLUE);
    				RGB = 1;
    			} else if(RGB == 1){
    				GPIO_ClearValue( 0, (1<<26) );
    				RGB = 0;
    			}
    		}
//    		ssegTimer
			//light and ledarray idle
    		//Every second

    		if(Timer(ForTimer,1000)){
				ForTimer = getTicks();
    			led7seg_setChar(ssg[ssegdisp],FALSE);

			//temp and acc smapled every 1s

				my_temp_value = temp_read();

				if(ssegdisp == 5 || ssegdisp == 10 || ssegdisp == 15){
					sprintf(oled_disp, "Temp: %2.2f", my_temp_value/10.0 );
					oled_putString(10,20,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

				}
			//7seg disp hex value increase by 1 every second
				if(ssegdisp != 15){
					ssegdisp++;
				} else {
					ssegdisp = 0;
				}
    		}

			//blink red if temp exceeeds threshold
			//blink blue if acc exceeds threshold
    		break;

    	case MODE_REV:
    		sprintf(oled_disp, "Reverse");
    		oled_putString(10,10,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		rgb_setLeds(RGB_BLUE);
			//light smapled every 1s
    		if(Timer(RevTimer,1000)){
    			RevTimer = getTicks();
				my_light_value = light_read();
    			sprintf(oled_disp, "Light: %u", my_light_value);
				oled_putString(10,20,(uint8_t *) oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		}
    		if()
    		pca9532_setLeds(0xAA00,0xFFFF);
    		pca9532_setLeds(0x00AA,0xFFFF);
			//ledarray proportional to light sensor value
			//temp and acc idle
    		break;

    	}

    }


}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
