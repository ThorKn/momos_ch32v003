/*
 * ---------------------------------------
 * MoMoS Envelooper - Mossys Modular Synths
 * ---------------------------------------
 * The base of this code is developed from examples 
 * in the ch32v003fun Github repo by cnlohr:
 * https://github.com/cnlohr/ch32v003fun
 * The license is MIT and is contained inside the repo.
 * 
 * Author: Thorsten Knoll - tknl.de
 * Date: 07/2024
 *
 * Inputs:
 * - PD3, PD4: Rotary encoder input
 * - PD6: Rotary encoder button
 *
 * Outputs:
 * - PC1: OLED I2C SDA
 * - PC2: OLED I2C SCL
 * - PD0: PWM envelope output
 */

// what type of OLED
#define SSD1306_128X64

#include "ch32v003fun.h"
#include "ch32v003_GPIO_branchless.h"
#include <stdio.h>
#include "ssd1306_i2c.h"
#include "ssd1306.h"

/******************************************************************************************
 * initialize TIM2 for Encoder Input
 ******************************************************************************************/
void t2encoder_init( void )
{
	// Enable GPIOD, TIM2, and AFIO *very important!*
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_NOREMAP; //set partial remap mode to NOREMAP

	// PD4 is T2CH1_, Input w/ Pullup/down
	GPIOD->CFGLR &= ~(0xf<<(4*4)); //clear old values
	GPIOD->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*4); //set new ones
	// 1 = pull-up, 0 = pull-down
	GPIOD->OUTDR |= 0<<4;

	// PD3 is T2CH2_, Input w/ Pullup/down
	GPIOD->CFGLR &= ~(0xf<<(4*3)); //clear values
	GPIOD->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*3); //set new ones
	// 1 = pull-up, 0 = pull-down
	GPIOD->OUTDR |= 0<<3;

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// SMCFGR: set encoder mode SMS=010b
	TIM2->SMCFGR |= TIM_EncoderMode_TI2;

	// initialize timer
	TIM2->SWEVGR |= TIM_UG;

	// set count to about mid-scale to avoid wrap-around
	TIM2->CNT = 0x8fff;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
};

/******************************************************************************************
 * initialize TIM1 for PWM Out
 ******************************************************************************************/
void t1pwm_init( void )
{
	// Enable GPIOD and TIM1
	RCC->APB2PCENR |= 	RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1;
	
	// PD0 is T1CH1N, 10MHz Output alt func, push-pull
	GPIOD->CFGLR &= ~(0xf<<(4*0));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*0);
			
	// Reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;
	
	// CTLR1: default is up, events generated, edge align
	// SMCFGR: default clk input is CK_INT
	
	// Prescaler 
	TIM1->PSC = 0x0000;
	
	// Auto Reload - sets period
	TIM1->ATRLR = 255;
	
	// Reload immediately
	TIM1->SWEVGR |= TIM_UG;
	
	// Enable CH1N output, positive pol
	TIM1->CCER |= TIM_CC1NE | TIM_CC1NP;
		
	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
	
	// Set the Capture Compare Register value to 50% initially
	TIM1->CH1CVR = 128;
	
	// Enable TIM1 outputs
	TIM1->BDTR |= TIM_MOE;
	
	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;
}

/******************************************************************************************
 * Set pulsewidth (PW) for timer 1
 ******************************************************************************************/
void t1pwm_setpw(uint16_t width)
{
	TIM1->CH1CVR = width;
}

int main()
{
	// Initsystem  with 48MHz internal clock
	SystemInit();
	Delay_Ms( 200 );

	// Say Hello over UART
	printf("\r\n");
	printf("-----------------\n\r");
	printf("Envelooper Module\n\r");
	printf("by MoMoS in 2024\n\r");
	printf("-----------------\n\r");
	printf("\r\n");

	// Init Encoder with Timer 2 on PD3 and PD4
	printf("Initializing Encoder input on GPIO PD3 and PD4.\n\r");
	t2encoder_init();
    Delay_Ms( 100 );
	uint16_t last_count = TIM2->CNT;
    uint16_t count = TIM2->CNT;

	// Init Encoder Button on PD6
	printf("Initializing Encoder Button input on GPIO PD6.\n\r");
	GPIO_port_enable(GPIO_port_D);
	GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_D, 6), GPIO_pinMode_I_pullUp, GPIO_Speed_In);

	// Init PWM output with Timer 1 on PD0
	printf("Initializing PWM output on GPIO PD0\n\r");
	t1pwm_init();
    Delay_Ms( 100 );
	
	// init OLED display with I2C on PC1 and PC2
	printf("Initializing OLED display on GPIO PC1 and PC2\n\r");
	uint8_t err = ssd1306_i2c_init();
	if (err != 0) {
		printf("Init failed\n\r");
	}
	Delay_Ms( 100 );
	err = ssd1306_init();
	if (err != 0) {
		printf("Init failed\n\r");
	}
	Delay_Ms( 100 );

	// Allocate variables
	uint8_t bar = 0;
	uint8_t bar_mode = 0;
	uint8_t bar_out_enable = 0;
	uint8_t pin_d1 = 0;
	uint8_t pin_d1_last = 0;
	uint8_t env[8];
	memset(env, 0, 8);

	printf("Module is up and running.\n\r");

	while(1)
	{
		// Handle the rotary encoder input
		if( count != last_count) {
			if ((count-last_count)<0) {
				// Encoder downwards
				if (bar_mode) {
					// Select the next lower bar
					if (bar > 0) {
						bar--;
					}
				} else {
					// Decrease envelope value of the selected bar
					if (env[bar] > 0) {
						env[bar]--;
					}
				}
			} else {
				// Encoder upwards
				if (bar_mode) {
					// Select the next higher bar
					if(bar < 7) {
						bar++;
					}
				} else {
					// Increase envelope value of the selected bar
					if (env[bar] < 7) {
						env[bar]++;
					}
				}
			}
			count = TIM2->CNT;
			last_count = count;
		}
		count = TIM2->CNT;

		// Handle the Encoder Button
		pin_d1 = !GPIO_digitalRead(GPIOv_from_PORT_PIN(GPIO_port_D, 6));
		if (pin_d1 && !pin_d1_last){
			bar_mode = !bar_mode;
		}
		pin_d1_last = pin_d1;

		// Set the PW of the PWM output, according to the bars
		if (bar_out_enable > 0) {
			bar_out_enable--;
		} else {
			bar_out_enable = 7;
		}
		t1pwm_setpw((uint16_t) 255-(env[bar_out_enable]*32));

		// Draw the bars onto the OLED display
		ssd1306_setbuf(0);
		ssd1306_drawRect(2,0,10,env[0]*8,1);
		ssd1306_drawRect(18,0,10,env[1]*8,1);
		ssd1306_drawRect(34,0,10,env[2]*8,1);
		ssd1306_drawRect(50,0,10,env[3]*8,1);
		ssd1306_drawRect(66,0,10,env[4]*8,1);
		ssd1306_drawRect(82,0,10,env[5]*8,1);
		ssd1306_drawRect(98,0,10,env[6]*8,1);
		ssd1306_drawRect(114,0,10,env[7]*8,1);
		ssd1306_xorrect(1+bar*16,0,12,64);
		ssd1306_refresh();

		// Delay the loop for the encoder input (no debounce code!)
		Delay_Ms(50);
	}
}