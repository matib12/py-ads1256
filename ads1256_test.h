/*
 * ADS1256_test.c:
 *	Very simple program to test the serial port. Expects
 *	the port to be looped back to itself
 *
 */
 
/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/

#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

//CS      -----   SPI_ADC_CS  
//DIN     -----   MOSI
//DOUT    -----   MISO
//SCLK    -----   SCLK
//DRDY    -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset


#define  DRDY           RPI_GPIO_P1_11  //P0
#define  RST            RPI_GPIO_P1_12  //P1
#define	 SPI_ADC_CS	    RPI_GPIO_P1_15	//P3
#define	 SPI_DAC_CS	    RPI_GPIO_P1_16	//P4

#define CS_ADC_1()      bcm2835_gpio_write(SPI_ADC_CS,HIGH)
#define CS_ADC_0()      bcm2835_gpio_write(SPI_ADC_CS,LOW)

#define CS_DAC_1()      bcm2835_gpio_write(SPI_DAC_CS,HIGH)
#define CS_DAC_0()      bcm2835_gpio_write(SPI_DAC_CS,LOW)

#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))

#define RST_1() 	    bcm2835_gpio_write(RST,HIGH);
#define RST_0() 	    bcm2835_gpio_write(RST,LOW);


/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short    
#define uint32_t unsigned long     


typedef enum {FALSE = 0, TRUE = !FALSE} bool;


/* Channel gain */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/* GAIN   2 */
	ADS1256_GAIN_4			= (2),	/* GAIN   4 */
	ADS1256_GAIN_8			= (3),	/* GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN   16 */
	ADS1256_GAIN_32			= (5),	/* GAIN   32 */
	ADS1256_GAIN_64			= (6),	/* GAIN   64 */
}ADS1256_GAIN_E;

/* Sampling speed choice*/
/* 
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* GAIN  */
	ADS1256_DRATE_E DataRate;	/* DATA output  speed*/
	int32_t AdcNow[8];		/* ADC Conversion value */
	uint8_t Channel;		/* The current channel*/
	uint8_t ScanMode;		/* Scanning mode, 0 = Single-ended input (8 channels); 1 = Differential input (4 channels)*/
}ADS1256_VAR_T;

ADS1256_VAR_T g_tADS1256; // Global structure of type ADS1256_VAR_T which carries configuration data of the ADC

/*Register definition Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,  // x1H
	REG_MUX    = 1,  // 01H
	REG_ADCON  = 2,  // 20H
	REG_DRATE  = 3,  // F0H
	REG_IO     = 4,  // E0H
	REG_OFC0   = 5,  // xxH
	REG_OFC1   = 6,  // xxH
	REG_OFC2   = 7,  // xxH
	REG_FSC0   = 8,  // xxH
	REG_FSC1   = 9,  // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition Table 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA    = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC   = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC   = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG     = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG     = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL  = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL  = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY  = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET    = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

void  bsp_DelayUS(uint64_t micros);

/*
*********************************************************************************************************
*	name: bsp_InitADS1256
*	function: Configuration of the STM32 GPIO and SPI interface. The connection ADS1256
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
void bsp_InitADS1256(void);

/*
*********************************************************************************************************
*	name: ADS1256_StartScan
*	function: Configuration DRDY PIN for external interrupt is triggered
*	parameter: _ucDiffMode : 0 = Single-ended input (8 channels); 1 = Differential input (4 channels)
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode);

/*
*********************************************************************************************************
*	name: ADS1256_Send8Bit
*	function: SPI bus to send 8 bit data
*	parameter: _data:  data
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data);

/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain: gain 1-64
*	           _drate: data rate of type ADS1256_DRATE_E
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

/*
*********************************************************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void);

/*
*********************************************************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void);

/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID);

/*
*********************************************************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd);

/*
*********************************************************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void);

/*
*********************************************************************************************************
*	name: ADS1256_SetChannel
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannel(uint8_t _ch);

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannel
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannel(uint8_t _ch);

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void);

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void);

/*
*********************************************************************************************************
*	name: ADS1256_GetAdc
*	function: read ADC value
*	parameter:  channel number 0--7
*	The return value:  ADC vaule (signed number)
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch);

/*
*********************************************************************************************************
*	name: ADS1256_ISR
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ADS1256_ISR(void);

/*
*********************************************************************************************************
*	name: ADS1256_Scan
*	function: Polling of the DRDY pin
*	parameter:NULL
*	The return value:  1
*********************************************************************************************************
*/
uint8_t ADS1256_Scan(void);

/*
*********************************************************************************************************
*	name: Write_DAC8552
*	function:  DAC send data 
*	parameter: channel : output channel number 1 or 2
*			   data : output DAC value 
*	The return value:  NULL
*********************************************************************************************************
*/
void Write_DAC8552(uint8_t channel, uint16_t Data);

/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value 
*	The return value:  NULL
*********************************************************************************************************
*/
uint16_t Voltage_Convert(float Vref, float voltage);

/*
*********************************************************************************************************
* Function written for python wrapper.
*********************************************************************************************************
*/
int  adcStart(int argc, char *par1, char *par2, char *par3);

/*
*********************************************************************************************************
* Function written for python wrapper.
*********************************************************************************************************
*/
long int readChannels(long int *valorCanal);

/*
*********************************************************************************************************
* Function written for python wrapper.
*********************************************************************************************************
*/
long int readChannel(long int ch);

/*
*********************************************************************************************************
* Function written for python wrapper.
*********************************************************************************************************
*/
int adcStop(void);
