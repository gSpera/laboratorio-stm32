#define REG(ADDR) (*(volatile unsigned int *)(ADDR))
#define EXTRACT_BIT(VALUE, BIT_POS) (((VALUE) >> (BIT_POS)) & 1)

#define GPIOA_BASE 0x48000000
#define GPIOC_BASE 0x48000800
#define GPIOE_BASE 0x48001000
#define GPIO_MODER 0
#define GPIO_ODR 0x14
#define GPIO_AFRL 0x20
#define RCC_BASE 0x40021000
#define RCC_AHBENR 0x14
#define RCC_APB1ENR 0x1C
#define RCC_APB2ENR 0x18
#define RCC_CFGR2 0x2C
#define TIM6_BASE 0x40001000
#define TIM_CR1 0x0
#define TIM_CNT 0x24
#define TIM_PSC 0x28
#define TIM_ARR 0x2C
#define TIM_SR 0x10
#define TIM_DIER 0x0C
#define NVIC_BASE 0xE000E004
#define NVIC_ISER 0xFC

#define ADC1_BASE 0x50000000
// ADCx, x = 1,2,3,4
#define ADC_CR 0x8
#define ADC_ISR 0x0
#define ADC_SQR1 0x30
#define ADC_CFGR 0x0C
#define ADC_DR 0x40
#define ADC_SMPR1 0x14

// ADCCx, x=12,34
#define ADCC12_BASE (ADC1_BASE + 0x300)
#define ADCC_CCR 0x8

#define USART1_BASE 0x40013800
#define USART_CR1 0x00
#define USART_CR2 0x04
#define USART_BRR 0x0C
#define USART_ISR 0x1C
#define USART_RDR 0x24
#define USART_TDR 0x28

#define SYS_CLK 8000000  // 8MHz
#define BAUD_RATE 9600

void isr_tim6dacunder() {}

#define BUFFER_SIZE 256
int buffer[BUFFER_SIZE];
#define PI 3.1415

#define SIN(t) \
    (t - (t * t * t) / (3 * 2) + (t * t * t * t * t) / (5 * 4 * 3 * 2))

float f(float t) {
    if (t <= BUFFER_SIZE / 4) {
        float x = t / BUFFER_SIZE * 2 * PI;
        return SIN(x);
    }
    if (t <= BUFFER_SIZE / 2) {
        return f(BUFFER_SIZE / 4 - (t - BUFFER_SIZE / 4));
    }

    return -f(BUFFER_SIZE / 2 - (t - BUFFER_SIZE / 2));
}

void generate_buffer() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer[i] = ((f((float)i) + 1) / 2) * 4096;
        if (buffer[i] > 4095) buffer[i] = 4095;
        if (buffer[i] < 0) buffer[i] = 0;
    }
}

int freq = 1000;  // 1kHz

extern const unsigned int DATA_START;
extern const unsigned int DATA_LEN;
extern const unsigned int DATA_LOAD_ADDR;

void uart_putc(char ch) {
    // Wait for TX clear
    while (EXTRACT_BIT(REG(USART1_BASE | USART_ISR), 7) == 0)
        ;
    REG(USART1_BASE | USART_TDR) = ch;
}

void uart_put_dec(int v) {
    char buff[10];
    int n;
    for (n = 0; n < 10; n++) {
        buff[n] = (v % 10) + '0';
        v /= 10;
        if (v == 0) break;
    }

    for (int i = n; i >= 0; i--) {
        uart_putc(buff[i]);
    }
}

int main() {
    // Copy .data from flash to ram
    unsigned int *from = (unsigned int *)&DATA_LOAD_ADDR;
    unsigned int *to = (unsigned int *)&DATA_START;
    for (unsigned int i = 0; i < ((unsigned int)&DATA_LEN); i++) {
        to[i] = from[i];
    }

    // generate_buffer();

    // Clock
    // ADC12|A|C|E
    REG(RCC_BASE | RCC_AHBENR) |= (1 << 28) | (1 << 17) | (1 << 19) | (1 << 21);
    REG(RCC_BASE | RCC_APB1ENR) |= 1 << 4;
    REG(RCC_BASE | RCC_APB2ENR) |= 1 << 14;  // USART1

    // GPIO
    REG(GPIOA_BASE | GPIO_MODER) |= (0b11 << 0);
    REG(GPIOC_BASE | GPIO_MODER) |= (0b10 << 8) | (0b10 << 10);
    REG(GPIOC_BASE | GPIO_AFRL) |= (0b0111 << 16) | (0b0111 << 20);
    REG(GPIOE_BASE | GPIO_MODER) |= 0x5555 << 16;

    // UART
    REG(USART1_BASE | USART_CR1) = (1 << 3) | (1 << 2);  // TE | RE
    REG(USART1_BASE | USART_CR2) = (0b00 << 12);         // 1 Stop Bit
    REG(USART1_BASE | USART_BRR) = SYS_CLK / BAUD_RATE;
    REG(USART1_BASE | USART_CR1) |= 1 << 0;  // Enable

    uart_putc('A');
    uart_putc('B');
    uart_put_dec(785);
    uart_putc('X');

    // Freq = SYS_CLOCK / (PSC + 1) = freq
    // SYS_CLOCK / freq -1 = PSC
    REG(TIM6_BASE | TIM_PSC) = 8000 - 1;  // Freq: 1kHz
    REG(TIM6_BASE | TIM_ARR) = 1000;      // Freq: freq
    REG(TIM6_BASE | TIM_CR1) |= 1 << 0;

    // ADC
    // Voltage Regulator
    REG(ADCC12_BASE | ADCC_CCR) |=
        (1 << 23) | (0b01 << 16);              // TSEN | CKMODE=01
    REG(ADC1_BASE | ADC_CR) &= ~(0b11 << 28);  // ADVREGEN = 00
    REG(ADC1_BASE | ADC_CR) |= (0b01 << 28);   // ADVREGEN = 01

    // Wait 10us
    for (volatile int i = 0; i < 80; i++) i = i;

    // Calibrate
    REG(ADC1_BASE | ADC_CR) |= 0b1 << 31;  // ADCAL
    while (EXTRACT_BIT(REG(ADC1_BASE | ADC_CR), 31) == 1)
        ;

    // Enable ADC
    REG(ADC1_BASE | ADC_CR) |= 0b1 << 0;
    while (EXTRACT_BIT(REG(ADC1_BASE | ADC_ISR), 0) == 0)
        ;

    // Channel Selection
    REG(ADC1_BASE | ADC_SQR1) = (1 << 6) | (0 << 0);  // SQ1=ADC1_IN1, L=1
    REG(ADC1_BASE | ADC_SMPR1) = 0b101 << 3;          // Sample Time
    REG(ADC1_BASE | ADC_CFGR) |= 0b00 << 3;           // 12 Bit

    while (1) {
        // ADC Start
        REG(ADC1_BASE | ADC_CR) |= 0b1 << 2;

        while (EXTRACT_BIT(REG(ADC1_BASE | ADC_ISR), 2) == 0)
            ;
        int value = REG(ADC1_BASE | ADC_DR);
        uart_put_dec(value);
        uart_putc('\n');
    }

    while (1)
        ;
}
