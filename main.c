#define GPIOE_BASE 0x48001000
#define GPIO_MODER 0
#define GPIO_ODR 0x14
#define RCC_BASE 0x40021000
#define RCC_AHBENR 0x14
#define RCC_APB1ENR 0x1C
#define TIM6_BASE 0x40001000
#define TIM_CR1 0x0
#define TIM_CNT 0x24
#define TIM_PSC 0x28
#define TIM_ARR 0x2C
#define TIM_SR 0x10
#define TIM_DIER 0x0C
#define NVIC_BASE 0xE000E004
#define NVIC_ISER 0xFC

#define SYS_CLK 8000000  // 8MHz

int leds = 0;
void isr_tim6dacunder() {
    unsigned int *ptr = (unsigned int *)(GPIOE_BASE + GPIO_ODR);
    leds ^= 0xFF << 8;
    *ptr = leds;
    ptr = (unsigned int *)(TIM6_BASE + TIM_SR);
    *ptr = 0;
}

int main() {
    // Clock
    volatile unsigned int *ptr = (unsigned int *)(RCC_BASE + RCC_AHBENR);
    *ptr |= 1 << 21;
    ptr = (unsigned int *)(RCC_BASE + RCC_APB1ENR);
    *ptr |= 1 << 4;

    // NVIC
    ptr = (unsigned int *)NVIC_BASE + NVIC_ISER;
    ptr[1] = 1 << 22;  // 54

    // TIM6
    ptr = (unsigned int *)(TIM6_BASE + TIM_PSC);
    *ptr = 8000 - 1;  // 8MHz / 8k = 1kHz
    ptr = (unsigned int *)(TIM6_BASE + TIM_ARR);
    *ptr = 2000;
    ptr = (unsigned int *)(TIM6_BASE + TIM_CNT);
    *ptr = 0;
    ptr = (unsigned int *)(TIM6_BASE + TIM_DIER);
    *ptr = 1;
    ptr = (unsigned int *)(TIM6_BASE + TIM_CR1);
    *ptr |= (1 << 0);

    // Moder
    ptr = (unsigned int *)(GPIOE_BASE + GPIO_MODER);
    *ptr |= 0x5555 << 16;

    volatile unsigned int *status =
        (volatile unsigned int *)(TIM6_BASE + TIM_SR);
    ptr = (volatile unsigned int *)(GPIOE_BASE + GPIO_ODR);

    while (1)
        ;
}
