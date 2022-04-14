/*
lab5 solved
*/

/* Includes */
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include "vamk_lcd_bare_metal.h"
/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

void delay_Ms(int delay);
void I2C1_init(void);
void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data);
void I2C1_ByteWrite(uint8_t address, uint8_t command);
void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t* data);
void USART2_Init(void);
void USART2_write(char data);
void I2C1_GetID(uint8_t address, int n, uint8_t* data);
void I2C1_GetFeatureSet(uint8_t address, int n, uint8_t* data);
void I2C1_Read1(uint8_t address, uint8_t command, int n, uint8_t* data);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

	I2C1_init();
	USART2_Init();
//	LCD_Init();
//	LCD_ClearAll();
//	LCD_GoToXY(0,0);
//	LCD_SendString("Value");
//  /* Configure the system clock to 32 MHz and update SystemCoreClock */
//	LCD_ClearAll();
//	LCD_GoToXY(0, 0);
//	LCD_SendString("2");
  SetSysClock();
  SystemCoreClockUpdate();

  /* TODO - Add your application code here */

  RCC->AHBENR|=1; //GPIOA ABH bus clock ON. p154
  GPIOA->MODER|=0x400; //GPIOA pin 5 to output. p184

    uint8_t address_for_write=0x58;	//0x90 When set to a “1” a read operation is selected, when set to a “0” a write operation is selected.
    uint8_t address_for_read=0x58;  	//0x91 When set to a “1” a read operation is selected, when set to a “0” a write operation is selected.
    uint8_t start_conversion=0x03; 	//start conversion 0111 0111
    uint8_t read_last_result=0x08; 	//read last result command 1010 1010
    uint8_t get_baseline=0x15;
    uint8_t set_baseline=0x1e;
    uint8_t raw=0x50;
    uint8_t H2_Eth[6]="";		//data array for temperature
    uint8_t TVOC_CO2[6]="";
    uint8_t ID[8]="";
    uint8_t Feature[3]="";
    uint8_t BaseLine[3] = {0x8E,0x68,0xCD};
  /* Infinite loop */
    uint8_t buf[50]="start";
    int H2 = 0;
    int Ethanol =0 ;
    int TVOC = 0;
    int CO2 = 0;
    for(int i=0;i<10;i++)
    {
    USART2_write(buf[i]);
    }
    I2C1_GetID(address_for_write,8,ID);
    I2C1_GetFeatureSet(address_for_write,2,Feature);
    I2C1_ByteWrite(address_for_write,start_conversion); 			//start conversion
//    I2C1_ByteWrite(address_for_write,set_baseline);
    I2C1_Read(address_for_read,get_baseline,3,BaseLine);
    I2C1_Write(address_for_write,set_baseline,3,BaseLine);
//    char buf_lcd[4]="";
delay_Ms(100);

int count = 0;
  while (1)
  {
	  count++;

	  I2C1_Read(address_for_read,read_last_result,5,TVOC_CO2);
	  I2C1_Read1(address_for_read,raw,5,H2_Eth);
	  CO2 = (TVOC_CO2[0]<<8) + TVOC_CO2[1];
	  TVOC= (TVOC_CO2[3]<<8) + TVOC_CO2[4];
	  H2 = (H2_Eth[0]<<8) + H2_Eth[1];
	  Ethanol = (H2_Eth[3]<<8) + H2_Eth[4];
	  sprintf(buf,"H2:%d Ethanol:%d TVOC:%d CO2:%d \n\r",H2,Ethanol, TVOC , CO2);

	  for(int i=0;i<44;i++)
	  {
	  USART2_write(buf[i]);
	  }

//	  sprintf(buf_lcd,"%d %d",H2,Ethanol);
//	  LCD_GoToXY(0,1);
//	  LCD_SendString(buf_lcd);

	  GPIOA->ODR|=0x20; //0010 0000 set bit 5. p186
	  delay_Ms(500);
	  GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186
	  delay_Ms(500);
	  if(count == 14){
		  count = 0;
//		  I2C1_ByteWrite(address_for_write,start_conversion);
		  I2C1_Read(address_for_read,get_baseline,5,BaseLine);
//		  I2C1_Write(address_for_write,set_baseline,3,BaseLine);

	  }
  }
  return 0;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}

void I2C1_init(void)
{
	RCC->AHBENR |= 2;			//Enable GPIOB clock PB8(D15)=SCL,PB9(D14)=SDA.
	RCC->APB1ENR |= (1<<21);	//Enable I2C1_EN clock

	//configures PB8,PB9 to I2C1_EN
	GPIOB->AFR[1] &= ~0x000000FF;	//PB8,PB9 I2C1 SCL, SDA. AFRH8 and AFRH9. clear
	GPIOB->AFR[1] |= 0x00000044;	//GPIOx_AFRL p.189,AF4=I2C1(0100 BIN) p.177
	GPIOB->MODER &= ~0x000F0000;	//PB8 and PB9 clear
	GPIOB->MODER |= 0x000A0000;		//Alternate function mode PB8,PB9
	GPIOB->OTYPER |= 0x00000300;	//output open-drain. p.184
	GPIOB->PUPDR &= ~0x000F0000;	//no pull-up resistors for PB8 and PB9 p.185

	I2C1->CR1 = 0x8000;				//software reset I2C1 SWRST p.682
	I2C1->CR1 &= ~0x8000;			//stop reset
	I2C1->CR2 = 0x0020;				//peripheral clock 32 MHz

	/*how to calculate CCR
	TPCLK1=1/32MHz=31,25ns
	tI2C_bus=1/10kHz=10us=100000ns
	tI2C_bus_div2=100000ns/2=50000ns
	CCR value=tI2C_bus_div2/TPCLK1=50000ns/31,25ns=160
	p. 692*/
	I2C1->CCR = 1600;

	//maximum rise time in sm mode = 1000ns. Equation 1000 ns/TPCK1
	I2C1->TRISE = 33;				//1000ns/31,25ns=32, p.693
	I2C1->CR1 |= 0x0001;			//eripheral enable (I2C1)
}

void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data)
{
	volatile int tmp;
	int i;

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
	I2C1->DR = command;				//send command
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	//write data
	for(i=0;i<n;i++)
	{
		while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689
		I2C1->DR=*data++;				//send command
	}

	while(!(I2C1->SR1 & 4)){}		//wait until byte transfer finished p.690
	I2C1->CR1 |= (1<<9);			//generate stop
}

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
void I2C1_GetID(uint8_t address, int n, uint8_t* data)
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

	I2C1->DR = 0x36;
	I2C1->DR = 0x82;				//send command
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->CR1 |= 0x100;				//generate repeated start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

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
void I2C1_GetFeatureSet(uint8_t address, int n, uint8_t* data)
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
	I2C1->DR = 0x2f;				//send command
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->CR1 |= 0x100;				//generate repeated start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

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

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}
