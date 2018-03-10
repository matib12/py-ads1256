long int  readChannels(long int *);
long int  readChannel(long int);
int       adcStart(int argc, char*, char*, char *);
int       adcStop(void);
void      Write_DAC8552(uint8_t, uint16_t);
uint16_t  Voltage_Convert(float, float);
