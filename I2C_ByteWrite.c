void I2C1_ByteWrite(uint8_t address, uint8_t command)
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
	I2C1->DR = command;				//send command

	while(!(I2C1->SR1 & 4)){}		//wait until byte transfer finished p.690
	I2C1->CR1 |= (1<<9);			//generate stop
}
