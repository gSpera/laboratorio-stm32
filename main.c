#define REG(ADDR) (*(volatile unsigned int *)(ADDR))

#define GPIOA_BASE 0x48000000
#define GPIOE_BASE 0x48001000
#define GPIO_MODER 0
#define GPIO_ODR 0x14
#define RCC_BASE 0x40021000
#define RCC_AHBENR 0x14
#define RCC_APB1ENR 0x1C
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

#define SYS_CLK 8000000  // 8MHz

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

extern unsigned int DATA_START;
extern unsigned int DATA_END;
extern unsigned int DATA_LOAD_ADDR;

int main() {
    // Copy .data from flash to ram
    for (unsigned int addr = DATA_START; addr < DATA_END; addr++) {
        unsigned int *ptr = (unsigned int *)(DATA_LOAD_ADDR + addr);
        *ptr = *(unsigned int *)addr;
    }

    generate_buffer();

    // Clock
    REG(RCC_BASE | RCC_AHBENR) |= 1 << 21;
    REG(RCC_BASE | RCC_APB1ENR) |= 1 << 4;

    // GPIO
    REG(GPIOE_BASE | GPIO_MODER) |= 0x5555 << 16;

    // Freq = SYS_CLOCK / (PSC + 1) = freq
    // SYS_CLOCK / freq -1 = PSC
    REG(TIM6_BASE | TIM_PSC) = 8000 - 1;  // Freq: 1kHz
    REG(TIM6_BASE | TIM_ARR) = 1000;      // Freq: freq
    REG(TIM6_BASE | TIM_CR1) |= 1 << 0;

    int i = 0;
    while (1) {
        REG(GPIOE_BASE | GPIO_ODR) |= buffer[i] << 8;

        while (1) {
            int sr = REG(TIM6_BASE | TIM_SR);
            if (sr != 0) break;
        }
        REG(TIM6_BASE | TIM_SR) = 1;
        i++;
        i %= 4096;
    }

    while (1)
        ;
}
