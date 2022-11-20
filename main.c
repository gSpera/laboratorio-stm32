#define GPIOA_BASE 0x4800000
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

int leds = 0;
void isr_tim6dacunder() {
    unsigned int *ptr = (unsigned int *)(TIM6_BASE + TIM_SR);
    *ptr = 0;
}

int main() {
    // Clock
    volatile unsigned int *ptr = (unsigned int *)(RCC_BASE + RCC_AHBENR);
    *ptr |= (1 << 17) | (1 << 21) | (1 << 28);  // GPIOA | GPIOE | ADC12
    ptr = (unsigned int *)(RCC_BASE + RCC_APB1ENR);
    *ptr |= 1 << 4;
    ptr = (unsigned int *)(RCC_BASE + RCC_CFGR2);
    *ptr |= 0b10000 << 4;  // PLL / 1

    // NVIC
    ptr = (unsigned int *)NVIC_BASE + NVIC_ISER;
    ptr[1] = 1 << 22;  // 54

    // TIM6
    ptr = (unsigned int *)(TIM6_BASE + TIM_PSC);
    *ptr = 8000 - 1;  // 8MHz / 8k = 1kHz
    ptr = (unsigned int *)(TIM6_BASE + TIM_ARR);
    *ptr = 1000;
    ptr = (unsigned int *)(TIM6_BASE + TIM_CNT);
    *ptr = 0;
    ptr = (unsigned int *)(TIM6_BASE + TIM_DIER);
    *ptr = 1;

    // Moder
    ptr = (unsigned int *)(GPIOE_BASE + GPIO_MODER);
    *ptr |= 0x5555 << 16;
    // ptr = (unsigned int *)(GPIOA_BASE + GPIO_MODER);
    // *ptr |= 0b11 << 0;

    // ADC
    // Step 1: Voltage Regulator
    ptr = (volatile unsigned int *)(ADCC12_BASE + ADCC_CCR);
    *ptr |= (1 << 23) | (0b01 << 16);  // TSEN | CKMODE = 01
    ptr = (volatile unsigned int *)(ADC1_BASE + ADC_CR);
    *ptr &= ~(0b11 << 28);  // ADVREGEN = 0b00
    *ptr |= 0b01 << 28;     // ADVREGEN = 0b10

    // Wait 10us
    // At 8MHz, 1 clock is 125ns,
    // 10us is 80 clocks
    // Thats a lot more of 80clocks
    // Maybe use a timer??
    for (volatile int i = 0; i < 80; i++) {
        i = i;
    }

    // Step 2: Calibrate
    // ADEN is 0
    // ADCALDIF is 0
    *ptr |= 0b1 << 31;  // ADCAL
    for (;;) {
        int adcal = (*ptr >> 31) & 0b1;
        if (adcal == 0) break;
    }

    // ADCAL is 0

    // Step 3: Enable ADC
    *ptr |= 0b1 << 0;
    // Wait ADRDY
    ptr = (volatile unsigned int *)(ADC1_BASE + ADC_ISR);
    for (;;) {
        int adrdy = (*ptr >> 0) & 0b1;
        if (adrdy == 1) break;
    }

    // ADRDY is 1

    // Step 4: Select channel
    // Temperature Sensor, ADC1_IN16
    // Sequence
    ptr = (unsigned int *)(ADC1_BASE + ADC_SQR1);
    *ptr = (16 << 6) | (0 << 0);  // SQ1 = ADC1_IN16, L = 1
    ptr = (unsigned int *)(ADC1_BASE + ADC_SMPR1);
    *ptr = 0b101 << 3;
    ptr = (unsigned int *)(ADC1_BASE + ADC_CFGR);
    *ptr |= (0b00 << 3);  // 12bit

    ptr = (unsigned int *)(TIM6_BASE + TIM_CR1);
    *ptr |= (1 << 0);

    for (;;) {
        // Step 5: Start ADC
        ptr = (unsigned int *)(ADC1_BASE + ADC_CR);
        *ptr |= 0b1 << 2;

        // Wait for finish
        ptr = (volatile unsigned int *)(ADC1_BASE + ADC_ISR);
        for (;;) {
            int eoc = (*ptr >> 2) & 0b1;
            if (eoc == 1) break;
        }

        ptr = (unsigned int *)(ADC1_BASE + ADC_DR);
        int value = *ptr;

        float v2i = 3.0 / 4096;
        // Should it be 1000??
        value = (1.43 - value * v2i) * 100 / 4.3 + 25;

        ptr = (unsigned int *)(GPIOE_BASE + GPIO_ODR);
        *ptr = value << 8;

        ptr = (unsigned int *)(TIM6_BASE + TIM_SR);
        while ((*ptr) == 0)
            ;
        *ptr = 0;
    }

    while (1)
        ;
}
