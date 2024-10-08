#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
  //structure that holds all the RCC registers that are responsible for managing the clock and reset functionalities of peripherals
  //with some of these registers we are able to turn on peripherals that are turned off initially

};



#define RCC ((struct rcc*) 0x40023800)

struct gpio{
	volatile uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2];
};

#define GPIOD ((struct gpio *) 0x40020C00)
#define GPIOB ((struct gpio *) 0x40020400)
#define GPIOA ((struct gpio *) 0x40020000)
#define GPIOE ((struct gpio *) 0x40021000)

struct nvic{
	volatile uint32_t ISER[8U],RESERVED0[24U],ICER[8U], RESERVED1[24U],  ISPR[8U], RESERVED2[24U],
	ICPR[8U], RESERVED3[24U], IABR[8U], RESERVED4[56U], IP[240U], RESERVED5[644U], STIR;
};

#define NVIC ((struct nvic *) 0xE000E100)

struct spi{
	volatile uint32_t CR1,CR2,SR,DR,CRCPR,RXCRCR,TXCRCR,I2SCFGR,I2SPR;
};



#define SPI1 ((struct spi*) 0x40013000)

struct systick{
	volatile uint32_t CTRL, LOAD,VAL,CALIB;
};
#define SYSTICK ((struct systick*) 0xE000E010)


#define accel_addr 0b0011001
volatile uint8_t accel_addr_read = 0x33;
volatile uint8_t accel_addr_write = 0x32;

#define FREQ 16000000 //clock freq


#define WHO_AM_I 0x0F
#define	CTRL_REG1 0x20
#define	CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE_DATACAPTURE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_THS_XH 0x32
#define INT1_THS_XL 0x33
#define INT1_THS_YH 0x34
#define INT1_THS_YL 0x35
#define INT1_THS_ZH 0x36
#define INT1_THS_ZL 0x37
#define INT1_DURATION 0x38








static inline void systick_init(){
	RCC->APB2ENR |= (1UL << 14); //write all the reference manual pages and shit here on comments
	SYSTICK->LOAD = 15999;
	SYSTICK->VAL = 0;
	SYSTICK->CTRL |= (1UL << 0) | (1UL << 1) | (1UL << 2);



}

volatile uint32_t counter;

void SysTick_Handler(void)
{
	//the original way was to increment a counter each time a clock cycle occured once that counter reached a certain number
	//of miliseconds  (16 MHZ means there is 16 million clock cycles per second)
	// the state of an LED changed
	//either rework that do it ur way without the using a expired_timer function
	//OR
	//find a diff way on the internet to use systick to

	// pass a different number of ticks to the initialization systick_init so that every interrupt the LED changes state
	// instead of using processor clock use an external clock that will trigger an interrupt every however wanted seconds
	// make it configurable so it can be easily changable.
	counter++;
}

void delay(uint32_t time) {
	//time is in ms
	counter =0;
	while(counter < time);
}


void gpio_config(){
	RCC->AHB1ENR |=(1U<<0); //enable GPIOA clock



	//PA5 = SCK
	//PA7 = MOSI
	//PA6 = MISO
	//PE3 = CS/NSS

	GPIOA->MODER &=~(3U<<10);
	GPIOA->MODER |=(2U<<10); // SCK alternate function mode

	GPIOA->MODER &=~(3U<<14);
	GPIOA->MODER |=(2U<<14); // MOSI alternate function mode

	GPIOA->MODER &=~(3U<<12);
	GPIOA->MODER |=(2U<<12); // MISO alternate function mode


	RCC->AHB1ENR |=(1U<<4); //enable GPIOE clock

	GPIOE->MODER &=~(3U<<6);
	GPIOE->MODER |=(1U<<6); // CS output mode

	GPIOA->AFR[0] |= (5U<<20); //SCK SPI alternate function
	GPIOA->AFR[0] |= (5U<<28); //MOSI SPI alternate function
	GPIOA->AFR[0] |= (5U<<24); //MISO SPI alternate function




}


void SPI1_config(){
	RCC->APB2ENR |=(1U<<12); //enable SPI1

	SPI1->CR1 = 0;
	SPI1->CR2 = 0;

	SPI1->CR1 |= (2U<<3); //set baud rate to Fpclk/4
	SPI1->CR1 |= (1U << 0); //set CHPA to 1
	SPI1->CR1 &= ~(1U << 1); //set CPOL to 1

	SPI1->CR1 &=~(1U<<11); //clear DFF bit to set data frame format to 8 bit

	SPI1->CR1 &=~(1U<<7); //claer LSBFIRST so the most significant bit is sent first

	SPI1->CR1 |=(1U<<8);
	SPI1->CR1 |=(1U<<9); //turn on manual software slave selection

	SPI1->CR1 |=(1U<<2); //set to master

	SPI1->CR1 |=(1U<<6); //enable SPI

}
void cs_enable(){
	GPIOE->ODR &=~(1U<<3); //pull the cs line low.
	// pin PE3 is connected to the CS line of the gyroscope
	//when the pin is pulled low,
	//when we pull the line low, we select a slave device that is connected to the pin that we want to communicate with

}
void cs_disable(){
	GPIOE->ODR |=(1U<<3); //bring cs high
	//PE3 is connected to CS line of gyroscope
	//High is the default state of the CS line
	//it means that, the communication between a master and a slave connected via this specific line (GPIO)
	//is stopped, until pulled low again.
}

void transmit(uint8_t *data,uint32_t size){
	uint8_t temp; //used to clear OVR flag later
	uint32_t i = 0;
	while(i < size){ //used to send more than 1 byte of data if needed. E.g. address AND value when writing to register
		while(!(SPI1->SR & (1U<<1))); //wait for TXe
		SPI1->DR = data[i]; //set DR to the register address OR value
		i++;
	}
	while(!(SPI1->SR & (1U<<1))); //wait for TXe

	while((SPI1->SR & (1U<<7))) //wait for busy flag to reset

		//clear OVR
	temp = SPI1->DR;
	temp = SPI1->SR;
}


uint8_t receive(){
	SPI1->DR = 0; // dummy read

	while(!(SPI1->SR & (1U<<0))); //wait for RXNe
	//when set, it means the RXNe buffer is NOT EMPTY. meaning data has been received and is ready to be read

	return (SPI1->DR); //read the RXNe buffer

}


uint8_t read(uint8_t addr){

	addr |= 0x80; //0x80 corresponds to 128 in binary. in the datasheet
	//it is stated that inorder to enable reading from a register, the first bit in the byte transferred
	//needs to be set. This is achieved by appending 0x80 to the first bit


	cs_enable(); //pull CS line low
	delay(10); //small delay for the peripheral to finish booting sequence

	transmit(&addr,1); //send JUST the address of the register
	uint8_t data = receive(); //read the value at the register
	cs_disable(); //pull CS line high
	return data; //return the data we got from the register

}

void send(uint8_t addr, uint8_t value){
	uint8_t data[2]; //array to store the address and the value

	data[0] = addr;
	data[1] = value;

	cs_enable(); //pull CS line low
	delay(10); //small delay for the peripheral to finish booting sequence
	transmit(data,2); //send the address first, then the value we want to set the address to.
	cs_disable(); //pull CS line high.
}

void gyro_init(){
	send(CTRL_REG5,0x01); //enable the high-pass filter
	send(CTRL_REG2,0x03); // set high-pass filter cutoff frequency to level 3
	//meaning 1 Hz at operating speed of 100Hz, which is the default, unless changed in CTRL_REG1


	send(CTRL_REG1,0x0F); //bit 0,1,2 are used to enable the x, y and z axis.
	//bit 3 is used to select the operating mode of the device,
	//reset value is sleep mode
	//set value is normal mode


	delay(10);

}

void read_gyro(int16_t *data){



	while(!(read(STATUS_REG) & (1<<3)));//wait for ZYXDA bit to get set
	//this bit signalizes that data is ready to be read from the X,Y,Z data registers

	data[0] = (int16_t)((read(OUT_X_H)<<8) + read(OUT_X_L)); //we want to store the H values in the upper 16 bits
	//we then want to concatenate the H register with the L
	data[1] = (int16_t)((read(OUT_Y_H)<<8) + read(OUT_Y_L));//we want to store the H values in the upper 16 bits
	//we then want to concatenate the H register with the L
	data[2] = (int16_t)((read(OUT_Z_H)<<8) + read(OUT_Z_L));//we want to store the H values in the upper 16 bits
	//we then want to concatenate the H register with the L


}

int main(){
	systick_init(); //used for delay
	RCC->AHB1ENR |=(1U<<3); //enable LED GPIO clock
	GPIOD->MODER |=(1U<<26); //Orange LED to output mode
	gpio_config();
	SPI1_config();



	uint8_t temp = read(WHO_AM_I);

	if(temp == 0xd3) {
		GPIOD->ODR |=(1U<<13); //if the value equals the WHO_AM_I value from the datasheet of the gyroscope,
		//turn on the orange LED
	}
	int16_t data[3];

	gyro_init();

	temp = read(OUT_TEMP);



	while(1){

		while(!(read(STATUS_REG) & (1<<3)));
		read_gyro(data); //loop to read the data registers

	}


}
