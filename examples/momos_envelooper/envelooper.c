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

// --------------------------
// TIMER DEFINES + VARS
// --------------------------
// mask for the CCxP bits
// when set PWM outputs are held HIGH by default and pulled LOW
// when zero PWM outputs are held LOW by default and pulled HIGH
#define TIM1_DEFAULT 0xff

// --------------------------
// ADC DEFINES + VARS
// --------------------------
#define ADC_NUMCHLS 2
volatile uint16_t adc_buffer[ADC_NUMCHLS];
volatile uint16_t adc_single;

// --------------------------
// SYSTICK DEFINES + VARS
// --------------------------
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

volatile uint32_t systick_cnt;
volatile uint32_t adc_systick;

volatile uint8_t env[8];

uint8_t wavetable[256] = {127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 186, 189, 192, 194, 197, 200, 202, 205, 207, 209, 212, 214, 216, 218, 221, 223, 225, 227, 229, 230, 232, 234, 235, 237, 239, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 253, 253, 254, 253, 253, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 247, 246, 245, 244, 243, 241, 240, 239, 237, 235, 234, 232, 230, 229, 227, 225, 223, 221, 218, 216, 214, 212, 209, 207, 205, 202, 200, 197, 194, 192, 189, 186, 184, 181, 178, 175, 172, 169, 166, 163, 160, 157, 154, 151, 148, 145, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82, 79, 76, 73, 70, 68, 65, 62, 60, 57, 54, 52, 49, 47, 45, 42, 40, 38, 36, 33, 31, 29, 27, 25, 24, 22, 20, 19, 17, 15, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 17, 19, 20, 22, 24, 25, 27, 29, 31, 33, 36, 38, 40, 42, 45, 47, 49, 52, 54, 57, 60, 62, 65, 68, 70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124};


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

/*
 * initialize adc for DMA
 */
void adc_init( void )
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	
	// Enable GPIOD and ADC
	RCC->APB2PCENR |=	RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						RCC_APB2Periph_ADC1;
	
	// PD2 is analog input chl 3
	GPIOD->CFGLR &= ~(0xf<<(4*2));	// CNF = 00: Analog, MODE = 00: Input
	
	// PC4 is analog input chl 2
	GPIOC->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	// Set up four conversions on chl 3 and 2
	ADC1->RSQR1 = (ADC_NUMCHLS-1) << 20;	// four chls in the sequence
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = (3<<(5*0)) | (2<<(5*1));
	
	// set sampling time for chl 3 and 2
	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	ADC1->SAMPTR2 = (3<<(3*3)) | (7<<(3*2));

	// turn on ADC
	ADC1->CTLR2 |= ADC_ADON;
	
	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	
	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
	
	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
	DMA1_Channel1->CNTR  = ADC_NUMCHLS;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC;
	
	// Turn on DMA channel 1
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// enable scanning
	ADC1->CTLR1 |= ADC_SCAN;
	
	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;
	
	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;
}

/*
 * Start up the SysTick IRQ
 */
void systick_init(void)
{
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);

	/* Set the tick interval to 1ms for normal op */
	SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK/1000)-1;

	/* Start at zero */
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}

/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{

	adc_systick = ((adc_buffer[0]+4)*8192);
	SysTick->CMP += adc_systick;

	/* clear IRQ */
	SysTick->SR = 0;

	/* update counter */
	systick_cnt--;
	systick_cnt&=7;
	TIM1->CH1CVR = 255-(env[systick_cnt]*32);
	// TIM1->CH1CVR = (env[systick_cnt]*32);
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

	// Init ADC
	printf("Initializing ADC...");
	adc_init();
    Delay_Ms( 100 );
	printf("done.\n\r");
	
	// Init systick
	printf("initializing systick...");
	systick_init();
    Delay_Ms( 100 );
	printf("done.\n\r");
	
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

	// Variables
	uint8_t bar = 0;
	uint8_t bar_mode = 0;
	uint8_t bar_out_enable = 0;
	uint8_t pin_d1 = 0;
	uint8_t pin_d1_last = 0;

	printf("Module is up and running.\n\r");

	while(1)
	{
		// Handle the rotary encoder input
		if( count != last_count) {
			if ((count-last_count)<0) {
				// Encoder downwards
				if (bar_mode) {
					// Select the next lower bar
					if (bar < 7) {
						bar++;
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
					if(bar > 0) {
						bar--;
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
		Delay_Ms(200);
	}
}