#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "ads1256_test.h"

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,	/* Reset the default values */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

void  bsp_DelayUS(uint64_t micros)
{
	bcm2835_delayMicroseconds(micros);
}

void bsp_InitADS1256(void)
{
#ifdef SOFT_SPI
	CS_ADC_1();
	SCK_0();
	DI_0();
#endif
//ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);	/* 1:1, 1KHz */
}

void ADS1256_StartScan(uint8_t _ucScanMode)
{
	g_tADS1256.ScanMode = _ucScanMode;
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for(i = 0; i < 8; i++){
			g_tADS1256.AdcNow[i] = 0;
		}
	}
}

static void ADS1256_Send8Bit(uint8_t _data)
{
	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* Storage ads1256 register configuration parameters */

		/* STATUS register define
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1  enable  calibration
		*/
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);// Enable the internal buffer
		//buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // Original settings
		buf[0] = (0 << 3) | (0 << 2) | (0 << 1);  // The internal buffer is prohibited, auto-cal is disabled

		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		/* MUX register */
		buf[1] = 0x08;

		/* ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 A
				10 = Sensor Detect Current = 2 A
				11 = Sensor Detect Current = 10 A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		
		/* DRATE register */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;

		CS_ADC_0();	/* SPI cs = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* Write command register, send the register address */
		ADS1256_Send8Bit(0x03);		/* Register number 4,Initialize the number  -1*/

		ADS1256_Send8Bit(buf[0]);	/* Set the STATUS register */
		ADS1256_Send8Bit(buf[1]);	/* Set the input MUX channel parameters */
		ADS1256_Send8Bit(buf[2]);	/* Set the ADCON control register,gain */
		ADS1256_Send8Bit(buf[3]);	/* Set the output rate DRATE register */

		CS_ADC_1();	/* SPI cs = 1 */
	}

	bsp_DelayUS(50);
}

static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/* The minimum time delay 6.5us */
}

static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_ADC_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/*Write command register */
	ADS1256_Send8Bit(0x00);		/*Write the register number */

	ADS1256_Send8Bit(_RegValue);	/*send register value */
	CS_ADC_1();	/* SPI   cs = 1 */
}

static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_ADC_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
	ADS1256_Send8Bit(0x00);	/* Write the register number */

	ADS1256_DelayDATA();	/*delay time */

	read = ADS1256_Recive8Bit();	/* Read the register values */
	CS_ADC_1();	/* SPI   cs  = 1 */

	return read;
}

static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_ADC_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_ADC_1();	/* SPI  cs  = 1 */
}

uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

static void ADS1256_SetChannel(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are I don'nt care!)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are I'don't care!)
	*/
	if (_ch > 7){
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection to AINCOM */
}

static void ADS1256_SetDiffChannel(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are I don't care!)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are I don't care!)
	*/
	if (_ch == 0){
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal  AIN0 AIN1 */
	}
	else if (_ch == 1){
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* DiffChannal  AIN2 AIN3 */
	}
	else if (_ch == 2){
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* DiffChannal  AIN4 AIN5 */
	}
	else if (_ch == 3){
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* DiffChannal  AIN6 AIN7 */
	}
}

static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 400000; i++){
		if (DRDY_IS_LOW()){
			break;
		}
	}
	if (i >= 400000){
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		
	}
}

static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	static uint8_t buf[3];

	CS_ADC_0();	/* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */

	ADS1256_DelayDATA();	/*delay time  */

	/*Read the sample results 24bit*/
	buf[0] = ADS1256_Recive8Bit();
	buf[1] = ADS1256_Recive8Bit();
	buf[2] = ADS1256_Recive8Bit();

	read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
	read |= buf[2];

	CS_ADC_1();	/* SPI = 1 */

	/* Extend a signed number*/
	if (read & 0x800000){
		read |= 0xFF000000;
	}

	return (int32_t)read;
}

int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	iTemp = g_tADS1256.AdcNow[_ch];

	return iTemp;
}

void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0){	/*  0  Single-ended input  8 channel 1 Differential input  4 channel */

		ADS1256_SetChannel(g_tADS1256.Channel);	/*Switch channel mode */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0){
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();	
		}
		else{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
		}

		if (++g_tADS1256.Channel >= 8){
			g_tADS1256.Channel = 0;
		}
	}
	else{	/*DiffChannal*/
		
		ADS1256_SetDiffChannel(g_tADS1256.Channel);	/* change DiffChannal */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0){
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();	
		}
		else{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
		}

		if (++g_tADS1256.Channel >= 4){
			g_tADS1256.Channel = 0;
		}
	}
}

uint8_t ADS1256_Scan(void)
{
	if(DRDY_IS_LOW())
	{
		ADS1256_ISR();
		return 1;
	}

	return 0;
}

void Write_DAC8552(uint8_t channel, uint16_t Data)
{
	uint8_t i = 0;

	if(channel == 1) i = 0x10;
	else if(channel == 2) i = 0x24;
	else return;
	
	if(i != 0){
		CS_DAC_1();
		CS_DAC_0();
		bcm2835_spi_transfer(i);
		bcm2835_spi_transfer((Data >> 8));
		bcm2835_spi_transfer((Data & 0xff));  
		CS_DAC_1();
	}
}

uint16_t Voltage_Convert(float Vref, float voltage)
{
	float _Df_;
	uint16_t _D_;

	//printf("%f: %f ", Vref, voltage);
	_Df_ = 65536.0 * voltage / Vref;
	if(_Df_ < 0.0) return 0;
	if(_Df_ > 65535.0) return 65535;

	_D_ = (uint16_t)_Df_;
    
	return _D_;
}


/*
*********************************************************************************************************
*	name: main
*	function:  
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*
*   From this part down, the code was modified by Fabio Franco de Oliveira. Email: fabioti6@gmail.com
*   It was necessary to eliminate the main loop for reading of all the ADCs, being
*   modified to read only 1 ADC at a time by passing parameters specifying which
*   the channel, gain and sample rate. Problems involving synchronization of the SPI protocol needed to be
*   resolved to enable the execution of the "ads1256_test" outside the loop.
*   Last modified: 05/21/2016
*
*********************************************************************************************************
*/

int  adcStart(int argc, char *par1, char *par2, char *par3)
{
	uint8_t id;
	uint8_t i,x,y;

	int ads_gain;
	int ads_channel;
	int ads_sps;


	if (!bcm2835_init())
		return 1;
    
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );     // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);  // The default
	// Configure ADC CS pin
	bcm2835_gpio_fsel(SPI_ADC_CS, BCM2835_GPIO_FSEL_OUTP);//
	bcm2835_gpio_write(SPI_ADC_CS, HIGH);
	// Configure DAC CS pin
	bcm2835_gpio_fsel(SPI_DAC_CS, BCM2835_GPIO_FSEL_OUTP);//
	bcm2835_gpio_write(SPI_DAC_CS, HIGH);
	//
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);    	
    
	id = ADS1256_ReadChipID();
   
	if (id != 3){
		printf("Error, ASD1256 Chip ID = 0x%d\r\n", (int)id);
	}
	else{
		//printf("Ok, ASD1256 Chip ID = 0x%d\r\n", (int)id);    // Line deliberately commented
	}


   
	// It does nothing if you pass a few arguments
	if( argc != 4 ) {                                
		printf("This program requires 3 parameters: channel, gain and datarate.\n");  
	}
	// It only executes if the 3 necessary arguments have been passed
	else{ 
		// Analyzes the first argument: the channel
		if (strcmp((par1), "0") == 0) 
			{  ads_channel=0;  } 
		else if (strcmp((par1), "1") == 0) 
			{  ads_channel=1;  }
		else if (strcmp((par1), "2") == 0) 
		    {  ads_channel=2;  }
		else if (strcmp((par1), "3") == 0) 
			{  ads_channel=3;  }
		else if (strcmp((par1), "4") == 0) 
			{  ads_channel=4;  } 
		else if (strcmp((par1), "5") == 0) 
			{  ads_channel=5;  }
		else if (strcmp((par1), "6") == 0) 
			{  ads_channel=6;  } 
		else if (strcmp((par1), "7") == 0) 
		    {  ads_channel=7;  }

		// If it is not any of these parameters then it has been set wrong thus assigning '666' to the variable ads_channel
		else{ 
			printf ("Incorrectly set channel: %s\n", par1);      
			printf ("CHANNEL setting must be either of the following: 0, 1, 2, 3, 4, 5, 6, 7\n\n");  
			ads_channel=666;  
		}


		// Analyzes the second argument: the gain
		if (strcmp((par2), "1") == 0) 
			{  ads_gain=0;  } 
		else if (strcmp((par2), "2") == 0) 
	    		{  ads_gain=1;  } 
		else if (strcmp((par2), "4") == 0) 
	    		{  ads_gain=2;  } 
		else if (strcmp((par2), "8") == 0) 
			{  ads_gain=3;  }  
		else if (strcmp((par2), "16") == 0) 
			{  ads_gain=4;  }  
		else if (strcmp((par2), "32") == 0) 
	    		{  ads_gain=5;  } 
		else if (strcmp((par2), "64") == 0) 
			{  ads_gain=6;  }  
		
		// If it is not any of these parameters then it has been set wrong thus assigning '666' to the variable ads_gain
		else  
		{ 
			printf ("Incorrectly set GAIN: %s\n", par2);   
			printf ("GAIN setting must be either of the following: 1, 2, 4, 8, 16, 32, 64\n\n");        
			ads_gain=666;  
		}


 	    // Analyze the third argument: the datarate
		if (strcmp((par3), "2d5") == 0) 
			{  ads_sps=15;  } 
		else if (strcmp((par3), "5") == 0) 
	    		{  ads_sps=14;  } 
		else if (strcmp((par3), "10") == 0) 
	    		{  ads_sps=13;  } 
		else if (strcmp((par3), "15") == 0) 
			{  ads_sps=12;  }  
		else if (strcmp((par3), "25") == 0) 
			{  ads_sps=11;  }  
		else if (strcmp((par3), "30") == 0) 
	    		{  ads_sps=10;  } 
		else if (strcmp((par3), "50") == 0) 
			{  ads_sps=9;  }  
		else if (strcmp((par3), "60") == 0) 
			{  ads_sps=8;  }  
		else if (strcmp((par3), "100") == 0) 
			{  ads_sps=7;  }  
		else if (strcmp((par3), "500") == 0) 
			{  ads_sps=6;  }  
		else if (strcmp((par3), "1000") == 0) 
			{  ads_sps=5;  }  
		else if (strcmp((par3), "2000") == 0) 
			{  ads_sps=4;  }  
		else if (strcmp((par3), "3750") == 0) 
			{  ads_sps=3;  }  
		else if (strcmp((par3), "7500") == 0) 
			{  ads_sps=2;  }  
		else if (strcmp((par3), "15000") == 0) 
			{  ads_sps=1;  }  
		else if (strcmp((par3), "30000") == 0) 
			{  ads_sps=0;  } 
		
		// If it is not any of these parameters then it has been set wrong thus assigning '666' to the variable ads_sps
		else  
		{ 
			printf ("Incorrectly set channel sampling rate: %s\n", par3); 
			printf ("Sampling rate setting must be either of the following: 2d5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000, 3750, 7500, 15000, 30000\n\n");     
			ads_sps=666;  
		}


		// If one of the parameters is marked '666', then there are errors and therefore displays the error message.
		if ((ads_channel==666) || (ads_gain==666) || (ads_sps==666)){
			printf ("Please Review ADC settings! Exiting...\n\n");
		}
		else{

			ADS1256_CfgADC(ads_gain, ads_sps);
			ADS1256_StartScan(0);

			// Boot Loop
			for (x = 0; x < 9; x++){
				for (y = 0; y < 8000; y++){
					if (bcm2835_gpio_lev(DRDY)==0){
						ADS1256_ISR();
						break;
					}      
				}
			}

			return 0; // Returns zero to say started ok
		}
	}

	return 1; // Problem, wrong parameters etc ...
}


// Function that the name needs to hit with the wrapper
long int readChannels(long int *valorCanal){
	int i;
	uint32_t adc[8];
	uint8_t buf[3];

	for (i = 0; i < 8; i++){
		while((ADS1256_Scan() == 0));

		adc[i] = ADS1256_GetAdc(i);
		buf[0] = ((uint32_t)adc[i] >> 16) & 0xFF;
		buf[1] = ((uint32_t)adc[i] >> 8) & 0xFF;
		buf[2] = ((uint32_t)adc[i] >> 0) & 0xFF;
		valorCanal[i]=  (long)adc[i]; 
		bsp_DelayUS(1);	
	}
	return 0;
}


long int readChannel(long int ch){
	long int ChValue;
	uint32_t adc[0];
	uint8_t buf[3];

	 
        while((ADS1256_Scan() == 0));

        adc[ch] = ADS1256_GetAdc(ch);
        buf[0] = ((uint32_t)adc[ch] >> 16) & 0xFF;
        buf[1] = ((uint32_t)adc[ch] >> 8) & 0xFF;
        buf[2] = ((uint32_t)adc[ch] >> 0) & 0xFF;
        ChValue =  (long)adc[ch]; 
        bsp_DelayUS(1);	
	 
	return ChValue;
}


int adcStop(void){
	bcm2835_spi_end();
	bcm2835_close();
	return 0;
}
