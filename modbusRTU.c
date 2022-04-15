/*
MODBUS RTU slave (USART)
04 READ INPUT REGISTER
https://www.modbustools.com/modbus.html#function04

Step 3. USART2 interrupt flowcode implemented and reads 7 bytes
if mFlag == 1 and delay 7 bytes if mFlag == 2.

*/

/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>

#define SLAVE_ADDRESS 0x05
#define MSB_COMMAND_VALUE 0x20
enum Air {CO2,TVOC,H2,Eth};
/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void USART2_write(char data);
void USART2_Init(void);
void timer11_init(void);
char USART2_read();
unsigned short int CRC16(char *nData,unsigned short int wLength);

void I2C1_init(void);
void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t *data);
void I2C1_Command_Init(uint8_t address, uint8_t command);
void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t *data);
void I2C1_GetID(uint8_t address, int n, uint8_t *data);
void I2C1_GetFeatureSet(uint8_t address, int n, uint8_t *data);
void read_sensor(int res[]);
void send_to_UART2(char *buf);
void generate_response(char *buf ,int res);
unsigned short int  crc_calculation(char *received_frame);
char mFlag=0;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
uint8_t address_for_write = 0x58;
	uint8_t address_for_read = 0x58;
	uint8_t init_command = 0x03;
	uint8_t air_quality_command = 0x08;
	uint8_t get_baseline_command = 0x15;
	uint8_t set_baseline_command = 0x1e;
	uint8_t raw_signal_command = 0x50;
int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	I2C1_init();
	USART2_Init();

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();

	/* TODO - Add your application code here */

	USART2->CR1 |= 0x0020;			//enable RX interrupt
	NVIC_EnableIRQ(USART2_IRQn); 	//enable interrupt in NVIC
	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

	RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
	GPIOA->MODER&=~0x00000C00;		//clear (input reset state for PA5). p184
	GPIOA->MODER|=0x400; 			//GPIOA pin 5 to output. p184
	char received_frame[8]={0};
	char i=0;



	uint8_t ID[8] = "";
	uint8_t Feature[3] = "";
	uint8_t BaseLine[6] = {0x8E, 0x68, 0xCD};

	uint8_t buf[50] = "start";


	I2C1_GetID(address_for_write, 8, ID);							  // get ID from slave device
	I2C1_GetFeatureSet(address_for_write, 2, Feature);				  // get Feature set from slave device
	I2C1_Command_Init(address_for_write, init_command);				  // send initial command to in
	I2C1_Read(address_for_read, get_baseline_command, 5, BaseLine);	  // read baseline value from slave device
	I2C1_Write(address_for_write, set_baseline_command, 5, BaseLine); // set current base line to set up new sequence

	int res[4];
	char input_register = 0xff;
	char H2_res[8]={0};
	char CO2_res[8]={0};
	char TVOC_res[8]={0};
	char Eth_res[8]={0};
	unsigned short int rcv_CRC = 0 ; // recived CRC from master frame
	unsigned short int com_CRC = 0 ; // compared  calculation CRC
  /* Infinite loop */
	   while (1)
	  {

	   if(mFlag==1)
	   {
		   while(i<7)
		   {
		   received_frame[i]=USART2_read();
		   i++;
		   }
		   i=0;

		   rcv_CRC = received_frame[6];
		   rcv_CRC = (rcv_CRC << 8) + received_frame[5];

		   // crc 618e
		   com_CRC = crc_calculation(received_frame);

		   if(rcv_CRC == test){
			   input_register = received_frame[4];
			   if(input_register == 0x01){
				   read_sensor(res);
				   generate_response(H2_res, res[H2]);
				   send_to_UART2(H2_res);
			   }
		   }
		   mFlag=0;
		   //GPIOA->ODR|=0x20;				//0010 0000 or bit 5. p186
		   USART2->CR1 |= 0x0020;			//enable RX interrupt
	   }
	   else if(mFlag==2)
	   {
		   USART2->CR1 &= ~0x00000004;		//RE bit. p739-740. Disable receiver
		   delay_Ms(10); 					//time=1/9600 x 10 bits x 7 byte = 7,29 ms
		   USART2->CR1 |= 0x00000004;		//RE bit. p739-740. Enable receiver
		   USART2->CR1 |= 0x0020;			//enable RX interrupt
		   mFlag=0;
	   }
	  }
  return 0;
}
unsigned short int crc_calculation(char *received_frame){
	char temp[8] = {0};
	temp[0]= 0x05;
	for(int i = 1 ; i <6 ; i++){
		temp[i] = received_frame[i-1];
	}
	return CRC16(temp,6);
}
void send_to_UART2(char *buf)
{
	int k = 0;
	while (buf[k] != '\0')
	{
		k++;
	}
	for (int i = 0; i < k; i++)
	{
		USART2_write(buf[i]);
	}
}
void generate_response(char *buf , int res){
	buf[0] = SLAVE_ADDRESS;
	buf[1] = 0x04;
	buf[2] = 0x02;
	buf[3] = res>>8;
	buf[4] = res;
	buf[5] = CRC16(buf,5);
	buf[6] = CRC16(buf,5)>>8;
	buf[7] = '\0';
}
void read_sensor(int res[4]){

	int H21= 0;
	int TVOC1 = 0;
	int Ethanol =0;
	int CO21 =0;
	uint8_t H2_Eth[6] = "";
	uint8_t TVOC_CO2[6] = "";

		for(int j = 0 ; j <10 ; j++){
			I2C1_Read(address_for_read, air_quality_command, 5, TVOC_CO2); // read air quality measurement from slave address
			I2C1_Read(address_for_read, raw_signal_command, 5, H2_Eth);	   // read raw signal from slave address
			CO21 += (TVOC_CO2[0] << 8) + TVOC_CO2[1];	 // convert CO2 value to int
			TVOC1 += (TVOC_CO2[3] << 8) + TVOC_CO2[4]; // convert TVOC value to int
			H21 += (H2_Eth[0] << 8) + H2_Eth[1];		 // convert H2 value to int
			Ethanol += (H2_Eth[3] << 8) + H2_Eth[4];	 // convert Ethanol value to int
		}
		res[0] = CO21/10;
		res[1] = TVOC1/10;
		res[2] = H21/10;
		res[3] = Ethanol/10;
}
void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); 	//measured with oscilloscope
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

void USART2_IRQHandler(void)
{
	char received_slave_address=0;

	if(USART2->SR & 0x0020) 		//if data available in DR register. p737
	{
		received_slave_address=USART2->DR;
	}
	if(received_slave_address==SLAVE_ADDRESS) //if we have right address
	{
		mFlag=1;
		GPIOA->ODR|=0x20;				//0010 0000 or bit 5. p186
	}
	else
	{
		mFlag=2;
		GPIOA->ODR&=~0x20;				//0010 0000 ~and bit 5. p186

	}
	USART2->CR1 &= ~0x0020;			//disable RX interrupt

}

unsigned short int CRC16 (char *nData,unsigned short int wLength)
{
static const unsigned short int wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

unsigned char nTemp;
unsigned short int wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

char USART2_read()
{
	char data=0;
									//wait while RX buffer is data is ready to be read
	while(!(USART2->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART2->DR;			//p739
		return data;
}
void I2C1_init(void)
{
	RCC->AHBENR |= 2;		   // Enable GPIOB clock PB8(D15)=SCL,PB9(D14)=SDA.
	RCC->APB1ENR |= (1 << 21); // Enable I2C1_EN clock

	// configures PB8,PB9 to I2C1_EN
	GPIOB->AFR[1] &= ~0x000000FF; // PB8,PB9 I2C1 SCL, SDA. AFRH8 and AFRH9. clear
	GPIOB->AFR[1] |= 0x00000044;  // GPIOx_AFRL p.189,AF4=I2C1(0100 BIN) p.177
	GPIOB->MODER &= ~0x000F0000;  // PB8 and PB9 clear
	GPIOB->MODER |= 0x000A0000;	  // Alternate function mode PB8,PB9
	GPIOB->OTYPER |= 0x00000300;  // output open-drain. p.184
	GPIOB->PUPDR &= ~0x000F0000;  // no pull-up resistors for PB8 and PB9 p.185

	I2C1->CR1 = 0x8000;	  // software reset I2C1 SWRST p.682
	I2C1->CR1 &= ~0x8000; // stop reset
	I2C1->CR2 = 0x0020;	  // peripheral clock 32 MHz

	/*how to calculate CCR
	TPCLK1=1/32MHz=31,25ns
	tI2C_bus=1/10kHz=10us=100000ns
	tI2C_bus_div2=100000ns/2=50000ns
	CCR value=tI2C_bus_div2/TPCLK1=50000ns/31,25ns=160
	p. 692*/
	I2C1->CCR = 1600;

	// maximum rise time in sm mode = 1000ns. Equation 1000 ns/TPCK1
	I2C1->TRISE = 33;	 // 1000ns/31,25ns=32, p.693
	I2C1->CR1 |= 0x0001; // eripheral enable (I2C1)
}

void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t *data)
{
	volatile int tmp;
	int i;

	while (I2C1->SR2 & 2)
	{
	} // wait until bus not busy

	I2C1->CR1 &= ~0x800; // Acknowledge clear p.682
	I2C1->CR1 |= 0x100;	 // generate start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = 0x20;
	while (!(I2C1->SR1 & 0x80))
	{
	}					// wait until data register empty p.689
	I2C1->DR = command; // send command
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	// write data
	for (i = 0; i < n; i++)
	{
		while (!(I2C1->SR1 & 0x80))
		{
		}					// wait until data register empty p.689
		I2C1->DR = *data++; // send command
	}

	while (!(I2C1->SR1 & 4))
	{
	}					   // wait until byte transfer finished p.690
	I2C1->CR1 |= (1 << 9); // generate stop
}

void I2C1_Command_Init(uint8_t address, uint8_t command)
{
	volatile int tmp;

	while (I2C1->SR2 & 2)
	{
	} // wait until bus not busy

	I2C1->CR1 &= ~0x800; // Acknowledge clear p.682
	I2C1->CR1 |= 0x100;	 // generate start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = 0x20;
	I2C1->DR = command; // send command

	while (!(I2C1->SR1 & 4))
	{
	}					   // wait until byte transfer finished p.690
	I2C1->CR1 |= (1 << 9); // generate stop
}
void I2C1_GetID(uint8_t address, int n, uint8_t *data)
{
	volatile int tmp;

	while (I2C1->SR2 & 2)
	{
	}					 // wait until bus not busy
	I2C1->CR1 &= ~0x800; // Acknowledge clear p.682

	I2C1->CR1 |= 0x100; // generate start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = 0x36;
	I2C1->DR = 0x82; // send command
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->CR1 |= 0x100; // generate repeated start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1 | 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2;		// Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	I2C1->CR1 |= (1 << 10); // Enable acknowledge p.683

	while (n > 0) // read data from chip
	{
		while (!(I2C1->SR1 & 0x40))
		{
		}					  // wait until RXNE flag is set
		(*data++) = I2C1->DR; // read data from DR
		n--;
	}
	I2C1->CR1 |= (1 << 9);	 // generate stop p.682
	I2C1->CR1 &= ~(1 << 10); // disable acknowledge p.682
}
void I2C1_GetFeatureSet(uint8_t address, int n, uint8_t *data)
{
	volatile int tmp;

	while (I2C1->SR2 & 2)
	{
	}					 // wait until bus not busy
	I2C1->CR1 &= ~0x800; // Acknowledge clear p.682

	I2C1->CR1 |= 0x100; // generate start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = 0x20;
	I2C1->DR = 0x2f; // send command
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->CR1 |= 0x100; // generate repeated start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1 | 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2;		// Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	I2C1->CR1 |= (1 << 10); // Enable acknowledge p.683

	while (n > 0) // read data from chip
	{
		while (!(I2C1->SR1 & 0x40))
		{
		}					  // wait until RXNE flag is set
		(*data++) = I2C1->DR; // read data from DR
		n--;
	}
	I2C1->CR1 |= (1 << 9);	 // generate stop p.682
	I2C1->CR1 &= ~(1 << 10); // disable acknowledge p.682
}

void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t *data)
{
	volatile int tmp;

	while (I2C1->SR2 & 2)
	{
	}					 // wait until bus not busy
	I2C1->CR1 &= ~0x800; // Acknowledge clear p.682

	I2C1->CR1 |= 0x100; // generate start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	I2C1->DR = address << 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = MSB_COMMAND_VALUE;
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->DR = command; // send command
	while (!(I2C1->SR1 & 0x80))
	{
	} // wait until data register empty p.689

	I2C1->CR1 |= 0x100; // generate repeated start p.694
	while (!(I2C1->SR1 & 1))
	{
	} // wait until start condition generated

	delay_Ms(10); // delay to wait for ack

	I2C1->DR = address << 1 | 1; // transmit slave address
	while (!(I2C1->SR1 & 2))
	{
	} // wait until end of address transmission p.690

	tmp = I2C1->SR2;		// Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	I2C1->CR1 |= (1 << 10); // Enable acknowledge p.683

	while (n > 0) // read data from chip
	{
		while (!(I2C1->SR1 & 0x40))
		{
		}					  // wait until RXNE flag is set
		(*data++) = I2C1->DR; // read data from DR
		n--;
	}
	I2C1->CR1 |= (1 << 9);	 // generate stop p.682
	I2C1->CR1 &= ~(1 << 10); // disable acknowledge p.682
}
