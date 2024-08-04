/*
 * ---------------------------------------
 * MoMoS LFO with PWM on TIMER1 - Mossys Modular Synths
 * ---------------------------------------
 * The base of this code is developed from examples in the
 * ch32v003fun Github repo by cnlohr.
 * https://github.com/cnlohr/ch32v003fun
 * The license is MIT and is contained in the repo.
 */

/*
Inputs:
- PD2 pin: Analog input 1 (Frequency)
Outputs:
- PD0 pin: PWM output (Low Frequency Oscillator)
*/

#include "ch32v003fun.h"
#include <stdio.h>

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

uint8_t wavetable[256] = {127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 186, 189, 192, 194, 197, 200, 202, 205, 207, 209, 212, 214, 216, 218, 221, 223, 225, 227, 229, 230, 232, 234, 235, 237, 239, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 253, 253, 254, 253, 253, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 247, 246, 245, 244, 243, 241, 240, 239, 237, 235, 234, 232, 230, 229, 227, 225, 223, 221, 218, 216, 214, 212, 209, 207, 205, 202, 200, 197, 194, 192, 189, 186, 184, 181, 178, 175, 172, 169, 166, 163, 160, 157, 154, 151, 148, 145, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82, 79, 76, 73, 70, 68, 65, 62, 60, 57, 54, 52, 49, 47, 45, 42, 40, 38, 36, 33, 31, 29, 27, 25, 24, 22, 20, 19, 17, 15, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 17, 19, 20, 22, 24, 25, 27, 29, 31, 33, 36, 38, 40, 42, 45, 47, 49, 52, 54, 57, 60, 62, 65, 68, 70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124};

void print_module_info( void )
{
	printf("\r\r\n\n");
	printf("-----------------------------------------\n\r");
	printf("| MoMoS LFO with PWM on TIMER1          |\n\r");
	printf("| Mossys Modular Synths by T.Knoll 2023 |\n\r");
	printf("-----------------------------------------\n\r");
}

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

	adc_systick = ((adc_buffer[0]+4)*1024);
	SysTick->CMP += adc_systick;

	/* clear IRQ */
	SysTick->SR = 0;

	/* update counter */
	systick_cnt++;
	systick_cnt++;
	systick_cnt&=255;
	TIM1->CH1CVR = wavetable[systick_cnt];
}

/*
 * entry
 */
int main()
{
	SystemInit();
	Delay_Ms( 100 );

	print_module_info();

	// Init TIM1 for PWM
	printf("Initializing PWM...");
	t1pwm_init();
	printf("Done.\n\r");

	// Init ADC
	printf("Initializing ADC...");
	adc_init();
	printf("done.\n\r");
	
	// Init systick
	printf("initializing systick...");
	systick_init();
	printf("done.\n\r");

	printf("Module is running.\n\r");
	while(1)
	{
		Delay_Ms( 100 );
		printf("Still running. ADC ch#1: %u\n\r", ((adc_buffer[0]+4)*1024));
	}
}
