void I2C1_init(void)
{
    RCC->AHBENR |= 2;          // Enable GPIOB clock PB8(D15)=SCL,PB9(D14)=SDA.
    RCC->APB1ENR |= (1 << 21); // Enable I2C1_EN clock

    // configures PB8,PB9 to I2C1_EN
    GPIOB->AFR[1] &= ~0x000000FF; // PB8,PB9 I2C1 SCL, SDA. AFRH8 and AFRH9. clear
    GPIOB->AFR[1] |= 0x00000044;  // GPIOx_AFRL p.189,AF4=I2C1(0100 BIN) p.177
    GPIOB->MODER &= ~0x000F0000;  // PB8 and PB9 clear
    GPIOB->MODER |= 0x000A0000;   // Alternate function mode PB8,PB9
    GPIOB->OTYPER |= 0x00000300;  // output open-drain. p.184
    GPIOB->PUPDR &= ~0x000F0000;  // no pull-up resistors for PB8 and PB9 p.185

    I2C1->CR1 = 0x8000;   // software reset I2C1 SWRST p.682
    I2C1->CR1 &= ~0x8000; // stop reset
    I2C1->CR2 = 0x0020;   // peripheral clock 32 MHz

    /*how to calculate CCR
    TPCLK1=1/32MHz=31,25ns
    tI2C_bus=1/100kHz=10us=10000ns
    tI2C_bus_div2=10000ns/2=5000ns
    CCR value=tI2C_bus_div2/TPCLK1=5000ns/31,25ns=160
    p. 692*/
    I2C1->CCR = 160;

    // maximum rise time in sm mode = 1000ns. Equation 1000 ns/TPCK1
    I2C1->TRISE = 33;    // 1000ns/31,25ns=32, p.693
    I2C1->CR1 |= 0x0001; // eripheral enable (I2C1)
}
