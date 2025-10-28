#include <stdint.h>
#include <stm32f10x.h>

#define FREQ_MIN (1.0f/64.0f)
#define FREQ_MAX (64.0f)
#define FREQ_START (1.0f)

#define BTN_UP_PORT GPIOA
#define BTN_UP_PIN 0
#define BTN_DOWN_PORT GPIOA
#define BTN_DOWN_PIN 1

float curr_freq = FREQ_START;

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__NOP();
	}
}

void delay_blink(float freq){
	float period = 1.0f / freq;
	float half_period = period / 2.0f;

	const uint32_t SysCLK_Hz = 8000000UL;
	uint32_t ticks = (uint32_t)(half_period * (SysCLK_Hz / 8));

	delay(ticks);
}

uint8_t read_button(GPIO_TypeDef* port, uint8_t pin){
	return ((port->IDR & (1 << pin)) == 0);
}

uint8_t debounce(GPIO_TypeDef* port, uint8_t pin){
	if (read_button(port, pin))
	{
		delay(20000);
		if (read_button(port, pin))
		{
			while (read_button(port, pin));
			return 1;
		}
		
	}
	return 0;
	
}

int __attribute((noreturn)) main(void) {
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz

	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOA->CRL |= (GPIO_CRL_CNF0_1);
	GPIOA->ODR |= (1U<<0);

	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
	GPIOA->CRL |= (GPIO_CRL_CNF1_1);
	GPIOA->ODR |= (1U<<1);

    while (1) {
		if (debounce(BTN_UP_PORT, BTN_UP_PIN))
		{
			if (curr_freq < FREQ_MAX)
			{
				curr_freq += 2.0f;
				if (curr_freq > FREQ_MAX)
				{
					curr_freq = FREQ_MAX;
				}
				
			}
			
		}

		if (debounce(BTN_DOWN_PORT, BTN_DOWN_PIN))
		{
			if (curr_freq > FREQ_MIN)
			{
				curr_freq -= 2.0f;
				if (curr_freq < FREQ_MIN)
				{
					curr_freq = FREQ_MIN;
				}
				
			}
			
		}

	    GPIOC->ODR ^= (1U<<13U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay_blink(curr_freq);
    }
}
