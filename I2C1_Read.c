void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t* data)
{
	volatile int tmp;

	while(I2C1->SR2 & 2){}			//wait until bus not busy
	I2C1->CR1 &= ~0x800;			//Acknowledge clear p.682

	I2C1->CR1 |= 0x100;				//generate start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

	I2C1->DR=address << 1;			//transmit slave address
	while(!(I2C1->SR1 & 2)){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->DR = 0x20;
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->DR=command;				//send command
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->CR1 |= 0x100;				//generate repeated start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated
	delay_Ms(10); 					// delay to wait for ack
	I2C1->DR=address << 1|1;		//transmit slave address
	while(!(I2C1->SR1 & 2)){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	I2C1->CR1 |= (1<<10);			//Enable acknowledge p.683

	while(n > 0)					//read data from chip
	{
		while(!(I2C1->SR1 & 0x40)){}	//wait until RXNE flag is set
		(*data++) = I2C1->DR;			//read data from DR
		n--;
	}
	I2C1->CR1 |= (1<<9);			//generate stop p.682
	I2C1->CR1 &= ~(1<<10);			//disable acknowledge p.682
}
