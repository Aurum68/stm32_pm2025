#include <stdint.h>
#include <stm32f10x.h>

#define FREQ_MIN (1.0f/64.0f)
#define FREQ_MAX (64.0f)
#define FREQ_START (1.0f)

#define BTN_UP_PORT GPIOA
#define BTN_UP_PIN 0
#define BTN_DOWN_PORT GPIOB
#define BTN_DOWN_PIN 0

#define SYS_CLK_FREQ 8000000.0f

float curr_freq = FREQ_START;

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__NOP();
	}
}

uint8_t read_button(GPIO_TypeDef* port, uint8_t pin){
	return ((port->IDR & (1 << pin)) == 0);
}

uint8_t debounce(GPIO_TypeDef* port, uint8_t pin){
	if (read_button(port, pin)) {
		delay(20000); // Небольшая задержка для антидребезга
		if (read_button(port, pin)) {
			while (read_button(port, pin)); // Ждем, пока кнопка будет отпущена
			return 1;
		}
	}
	return 0;
}


void update_timer_freq(float freq) {
    float interrupt_freq = freq * 2.0f;

    uint16_t arr_value = (uint16_t)((1000.0f / interrupt_freq) - 1.0f);

    TIM2->ARR = arr_value;
}

void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 7999;

    update_timer_freq(curr_freq);

    TIM2->DIER |= TIM_DIER_UIE;

    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        GPIOC->ODR ^= (1U << 13U);
    }
}

void SPI1_Init(void)
{
    // Включаем тактирование SPI1 и портов
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

    // Настроим PA5 (SCK), PA7 (MOSI), PA4 (NSS/CS) - на вывод
    // SCK & MOSI: Альтернативная функция, push-pull, 50MHz
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7 | GPIO_CRL_MODE5 | GPIO_CRL_MODE7);
    GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5) | (GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7);

    // PA4 (CS) - General purpose output push-pull
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
    GPIOA->CRL |= (GPIO_CRL_MODE4_1); // 2 МГц

    // PA1 (DC), PA2 (RESET) то же самое
    GPIOA->CRL &= ~((GPIO_CRL_CNF1 | GPIO_CRL_MODE1) | (GPIO_CRL_CNF2 | GPIO_CRL_MODE2));
    GPIOA->CRL |= (GPIO_CRL_MODE1_1) | (GPIO_CRL_MODE2_1);

    // Снимаем CS, RESET, DC
    GPIOA->ODR |= (1<<4);
    GPIOA->ODR |= (1<<2);
    GPIOA->ODR &= ~(1<<1);

    // Настройка SPI1 (Мастер, SCK low, фаза 1, 8 бит, ... макс делитель)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 | SPI_CR1_SPE;
}

void SPI1_Write(uint8_t data) 
{
    // Ждем свободности передатчика
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    // Ждем окончания передачи
    while(SPI1->SR & SPI_SR_BSY);
}

uint8_t SPI1_Read(void)
{
    // Для чтения нужно отправить dummy byte
    SPI1_Write(0xFF);
    while(!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void SSD1306_WriteCommand(uint8_t cmd)
{
    GPIOA->ODR &= ~(1<<1);    // DC=0
    GPIOA->ODR &= ~(1<<4);    // CS=0
    SPI1_Write(cmd);
    GPIOA->ODR |= (1<<4);     // CS=1
}

void SSD1306_WriteData(uint8_t data)
{
    GPIOA->ODR |= (1<<1);     // DC=1
    GPIOA->ODR &= ~(1<<4);    // CS=0
    SPI1_Write(data);
    GPIOA->ODR |= (1<<4);     // CS=1
}

void SSD1306_Reset(void)
{
    GPIOA->ODR &= ~(1<<2);    // RES=0
    for (volatile uint32_t i=0; i<10000; i++);
    GPIOA->ODR |= (1<<2);     // RES=1
}

void SSD1306_Init(void)
{
    SSD1306_Reset();

    SSD1306_WriteCommand(0xAE); // Off
    SSD1306_WriteCommand(0x20); SSD1306_WriteCommand(0x00); // Horizontal Addr Mode
    SSD1306_WriteCommand(0xB0); // Page 0
    SSD1306_WriteCommand(0xC8); // Scan Direction
    SSD1306_WriteCommand(0x00); // Low col
    SSD1306_WriteCommand(0x10); // High col
    SSD1306_WriteCommand(0x40); // Start line
    SSD1306_WriteCommand(0x81); SSD1306_WriteCommand(0xFF); // Contrast
    SSD1306_WriteCommand(0xA1); // Segment remap
    SSD1306_WriteCommand(0xA6); // Normal display
    SSD1306_WriteCommand(0xA8); SSD1306_WriteCommand(0x3F); // Multiplex (64-1)
    SSD1306_WriteCommand(0xA4); // Output follows RAM
    SSD1306_WriteCommand(0xD3); SSD1306_WriteCommand(0x00); // Display offset
    SSD1306_WriteCommand(0xD5); SSD1306_WriteCommand(0x80); // clock
    SSD1306_WriteCommand(0xD9); SSD1306_WriteCommand(0xF1); // pre-charge
    SSD1306_WriteCommand(0xDA); SSD1306_WriteCommand(0x12); // compins
    SSD1306_WriteCommand(0xDB); SSD1306_WriteCommand(0x40); // vcom deselect
    SSD1306_WriteCommand(0x8D); SSD1306_WriteCommand(0x14); // charge pump
    SSD1306_WriteCommand(0xAF); // Display ON
}

void SSD1306_DrawChessBoard(void)
{
    for (uint8_t page=0; page<8; page++) {
        SSD1306_WriteCommand(0xB0 | page);      // Set page
        SSD1306_WriteCommand(0x00);             // Set low column address
        SSD1306_WriteCommand(0x10);             // Set high column address
        for (uint8_t col=0; col<128; col++) {
            uint8_t val = 0x00;
            // 16x16 px клетки (64/8=8 по вертикали, 128/16=8 по горизонтали)
            if ( ((page/2)%2) ^ ((col/16)%2) ) val = 0xFF;
            SSD1306_WriteData(val);
        }
    }
}

int __attribute((noreturn)) main(void) {
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

	GPIOC->CRH &= ~GPIO_CRH_CNF13;
	GPIOC->CRH |= GPIO_CRH_MODE13_0;

	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // Сброс битов
	GPIOA->CRL |= GPIO_CRL_CNF0_1;                  // CNF_1 = 1, CNF_0 = 0 -> Input with pull-up/pull-down
	GPIOA->ODR |= (1U << BTN_UP_PIN);               // Включаем внутренний pull-up резистор

	GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // Сброс битов
	GPIOB->CRL |= GPIO_CRL_CNF0_1;                  // CNF_1 = 1, CNF_0 = 0 -> Input with pull-up/pull-down
	GPIOB->ODR |= (1U << BTN_DOWN_PIN);                   

    SPI1_Init();
    SSD1306_Init();
    SSD1306_DrawChessBoard();

    timer_init();

    while (1) {
		if (debounce(BTN_UP_PORT, BTN_UP_PIN)) {
			if (curr_freq < FREQ_MAX) {
				curr_freq *= 2.0f; // Умножаем для более заметного эффекта
				if (curr_freq > FREQ_MAX) {
					curr_freq = FREQ_MAX;
				}
                update_timer_freq(curr_freq); // Обновляем таймер!
			}
		}

		if (debounce(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
			if (curr_freq > FREQ_MIN) {
				curr_freq /= 2.0f; // Делим для более заметного эффекта
				if (curr_freq < FREQ_MIN) {
					curr_freq = FREQ_MIN;
				}
                update_timer_freq(curr_freq); // Обновляем таймер!
			}
		}
    }
}

