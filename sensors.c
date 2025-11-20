// Default clock source is internal 8MHz RC oscillator
#define F_CPU 16000000UL
#define SCL_CLOCK 100000UL // I2C clock frequency (100kHz)

#define UVADDR    0x53
#define BMPADDR   0x77 

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/twi.h>
#include <string.h>
#include "bme280.h"
#include "bme280_defs.h"

uint8_t buffer[128];
static struct bme280_uncomp_data myData;
static struct bme280_calib_data calData;

void i2c_init(void);
uint8_t i2c_write_byte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t i2c_read_byte(uint8_t slaveAddr, uint8_t regAddr);
static int32_t compensateTemp(int32_t adcTemp);
static double compensatePressure(int32_t adcPressure);
static double compensateHumidity(int32_t adcPressure);

double ambient2Lux(uint32_t rawAmbient);
uint8_t calcUVI(uint32_t rawUV);


void uartSend(uint8_t []);

void getTPHdata(uint8_t addr);
uint32_t getUVdata(uint8_t addr);

void getCompParameters(uint8_t addr);


ISR(USART_RX_vect) {
}

void getCompParameters(uint8_t addr) {
	
	uint32_t data;
	
	// Temperature
		
	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_t1 = data;
	calData.dig_t1 = calData.dig_t1 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_t1 |= data;

	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_t2 = data;
	calData.dig_t2 = calData.dig_t2 << 8;	

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_t2 |= data;	
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_t3 = data;
	calData.dig_t3 = calData.dig_t3 << 8;
	
	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_t3 |= data;
	
	// Pressure
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p1 = data;
	calData.dig_p1 = calData.dig_p1 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p1 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p2 = data;
	calData.dig_p2 = calData.dig_p2 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p2 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p3 = data;
	calData.dig_p3 = calData.dig_p3 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p3 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p4 = data;
	calData.dig_p4 = calData.dig_p4 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p4 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p5 = data;
	calData.dig_p5 = calData.dig_p5 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p5 |= data;
		
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p6 = data;
	calData.dig_p6 = calData.dig_p6 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p6 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p7 = data;
	calData.dig_p7 = calData.dig_p7 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p7 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p8 = data;
	calData.dig_p8 = calData.dig_p8 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p8 |= data;
	
	addr+=2;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_p9 = data;
	calData.dig_p9 = calData.dig_p9 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_p9 |= data;

// humidity
	addr+=3; //0xa1

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_h1 = data;
	
	addr = 0xe1;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_h2 = data;
	calData.dig_h2 = calData.dig_h2 << 8;

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_h2 |= data;
	
	addr+=2; // 0xe3

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_h3= data;
	
	addr++; // 0xe4

	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_h4= data;
	calData.dig_h4 = calData.dig_h4 << 4;
	
	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_h4 |= data & 0x0f;
	
	data = i2c_read_byte(BMPADDR, addr+2);
	calData.dig_h5 = data;
	calData.dig_h5 = calData.dig_h5 << 4;

	data = i2c_read_byte(BMPADDR, addr+1);
	calData.dig_h5 |= (data >> 4) & 0x0f;
		
	addr+=3;
	
	data = i2c_read_byte(BMPADDR, addr);
	calData.dig_h6 = data;
	
}


static double compensateHumidity(int32_t adcHumidity) {

	   double humidity;
	   double humidity_min = 0.0;
	   double humidity_max = 100.0;
	   double var1;
	   double var2;
	   double var3;
	   double var4;
	   double var5;
	   double var6;

	   var1 = ((double)calData.t_fine) - 76800.0;
	   var2 = (((double)calData.dig_h4) * 64.0 + (((double)calData.dig_h5) / 16384.0) * var1);
	   var3 = adcHumidity - var2;
	   var4 = ((double)calData.dig_h2) / 65536.0;
	   var5 = (1.0 + (((double)calData.dig_h3) / 67108864.0) * var1);
	   var6 = 1.0 + (((double)calData.dig_h6) / 67108864.0) * var1 * var5;
	   var6 = var3 * var4 * (var5 * var6);
	   humidity = var6 * (1.0 - ((double)calData.dig_h1) * var6 / 524288.0);

	   if (humidity > humidity_max)
	   {
		   humidity = humidity_max;
	   }
	   else if (humidity < humidity_min)
	   {
		   humidity = humidity_min;
	   }

	   return humidity;
}



static double compensatePressure(int32_t adcPressure) {

    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calData.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calData.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calData.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calData.dig_p4) * 65536.0);
    var3 = ((double)calData.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calData.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calData.dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
	    pressure = 1048576.0 - (double) adcPressure;
	    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
	    var1 = ((double)calData.dig_p9) * pressure * pressure / 2147483648.0;
	    var2 = pressure * ((double)calData.dig_p8) / 32768.0;
	    pressure = pressure + (var1 + var2 + ((double)calData.dig_p7)) / 16.0;

	    if (pressure < pressure_min)
	    {
		    pressure = pressure_min;
	    }
	    else if (pressure > pressure_max)
	    {
		    pressure = pressure_max;
	    }
    }
    else /* Invalid case */
    {
	    pressure = pressure_min;
    }

    return pressure;
}


static int32_t compensateTemp(int32_t adcTemp) {
	
	int32_t var1, var2, T;
	
	var1 = (int32_t)((adcTemp / 8) - ((int32_t)calData.dig_t1 * 2));
	var1 = (var1 * ((int32_t)calData.dig_t2)) / 2048;
	var2 = (int32_t)((adcTemp / 16) - ((int32_t)calData.dig_t1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calData.dig_t3)) / 16384;
	calData.t_fine = var1 + var2;
	T = (calData.t_fine * 5 + 128) / 256;
 
	return T; 	
}

double ambient2Lux(uint32_t rawAmbient) {
	
	double lux;
	uint8_t gain = 3;
	double resInt = 1;
	uint8_t Wfac = 1;	//1 for no or clear window, > for tinted window1 
	
	lux = ( (0.6 * rawAmbient) / (gain * resInt) ) * Wfac;
	
	return lux;
}

uint8_t calcUVI(uint32_t rawUV) {
	
	uint16_t UVsensitivity = 2300;
	uint16_t uvi;
	uint16_t Wfac = 1;
	
	uvi = (rawUV / UVsensitivity) * Wfac;
	
	return uvi;
	
}



void i2c_init(void) {
    
    // Set SCL frequency
   TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    // Enable TWI
    TWCR = (1 << TWEN);
}

uint8_t i2c_write_byte(uint8_t slave_address, uint8_t regAddr, uint8_t data) {
	
    // Send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag
    if ((TWSR & 0xF8) != TW_START) return 1; // Check status

    // Send SLA+W (Slave Address + Write bit)
    TWDR = (slave_address << 1) & 0xfe; // Write bit is 0
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_MT_SLA_ACK) return 2;
	
	// Send the register address
    TWDR = regAddr; // Write bit is 0
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
//    if ((TWSR & 0xF8) != TW_MT_SLA_ACK) return 3;
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 7;

    // Send data byte
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 4;

    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    return 0; // Success
	
	_delay_ms(10);
}

uint8_t i2c_read_byte(uint8_t slave_address, uint8_t regAddr) {
	
	uint8_t data;
	
    // Send START condition (similar to write)
//    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWEA);
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_START) return 9;

    // Send SLA+R (Slave Address + write bit)
    TWDR = (slave_address << 1) & 0xfe; 

    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
//    if ((TWSR & 0xF8) != TW_MR_SLA_ACK) return 2;
	
	if ((TWSR & 0xF8) != TW_MT_SLA_ACK) return 2;
	
	// Send the register address
	TWDR = regAddr; 
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
//	if ((TWSR & 0xF8) != TW_MT_SLA_ACK) return 1;
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;
	
	// Send start again
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	if ((TWSR & 0xF8) != TW_REP_START) return 6;
	
    // Send SLA+R (Slave Address + read bit)
    TWDR = (slave_address << 1) | 0x01;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_MR_SLA_ACK) return 4;
	
    // Read data byte (with NACK for single byte read)
    TWCR = (1 << TWINT) | (1 << TWEN); // No ACK for last byte
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_MR_DATA_NACK) return 3; // Expect NACK
    data = TWDR;

    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	
	_delay_ms(1);
	
    return data; // Success
}

uint32_t getUVdata(uint8_t addr) {

	uint32_t data, readData;

	readData = i2c_read_byte(UVADDR, addr+2);
	data = readData << 12;

	readData = i2c_read_byte(UVADDR, addr+1);
	data |= readData << 8;

		
	readData = i2c_read_byte(UVADDR, addr);
	data |= readData;
	
	return data;
	
}

void getTPHdata(uint8_t addr) {
	
	int32_t data, readData;

	readData = i2c_read_byte(BMPADDR, addr);
	data = readData << 12;
	
	addr++;

	readData = i2c_read_byte(BMPADDR, addr);
	data |= readData << 4;
	
	addr++;
	
	readData = i2c_read_byte(BMPADDR, addr);
	data |= (readData >> 4) & 0x0f;
	myData.pressure = data;
	
	addr++;

	readData = i2c_read_byte(BMPADDR, addr);
	data = readData << 12;
	
	addr++;

	readData = i2c_read_byte(BMPADDR, addr);
	data |= readData << 4;
	
	addr++;
	
	readData = i2c_read_byte(BMPADDR, addr);
	data |= (readData >> 4) & 0x0f; 
	myData.temperature = data;

	addr++;

	readData = i2c_read_byte(BMPADDR, addr);
	data = readData << 8;
	
	addr++;
	
	readData = i2c_read_byte(BMPADDR, addr);
	data |= readData;
	myData.humidity = data;

}

void uartSend(uint8_t character[]) {

  int i;
  
  for (i=0; i<=strlen( (char *)character); i++) {
  
    do {
    } while ( !(UCSR0A & 0x20) );
  
    UDR0 = character[i];
  }

}


int main()
{
    uint8_t rslt, chipID;
    int32_t period;
	int32_t temperature;
	uint32_t ambientLight, rawUV, uvi;
	
	
	double pressure; 
	double  humidity;
	double inPressure;
	double lux;
    
    cli();
    DDRB |= (1 << PB5);
    
    // Set up I2C
    i2c_init();
    
    //set up UART0
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    
    // 9600
    UBRR0H = 0x00; 
    UBRR0L = (uint8_t)103;

	// Get the compensation parameters fm the BME280
	getCompParameters(0x88);
	
    chipID = i2c_read_byte(BMPADDR, 0xd0);
        
	sprintf((char*)buffer, "BME280 Chip ID: %x\r\n", chipID);
    uartSend(buffer);
		
    chipID = i2c_read_byte(UVADDR, 0x06);
        
    sprintf((char*)buffer, "LTR390Chip ID: %x\r\n\r\n", chipID);
    uartSend(buffer);

	// BME280 configuration
		
	// Oversampling of humidity data: x8
	i2c_write_byte(BMPADDR, 0xf2, 0x04);
	_delay_ms(10);
	
	// ctrl_meas: oversampling x8 for T and P, forced mode		
	i2c_write_byte(BMPADDR, 0xf4, 0x91);
	_delay_ms(10);
	
	//config
	i2c_write_byte(BMPADDR, 0xf5, 0x40);
	_delay_ms(10);
		
	//LTR390 config
	i2c_write_byte(UVADDR, 0x00, 0x02); //MAIN_CTL reg: ALS mode	
	_delay_ms(10);
		
	i2c_write_byte(UVADDR, 0x04, 0x22); //ALS_UVS_MEAS_RATE
	_delay_ms(10);	
		
	i2c_write_byte(UVADDR, 0x05, 0x01); //ALS_UVS_GAIN
	_delay_ms(10);
			
	getTPHdata(0xf7);
		
	sprintf((char*)buffer, "RAW Temperature: %ld\r\n", myData.temperature);
	uartSend(buffer);
		
	temperature = compensateTemp(myData.temperature);
	sprintf((char*)buffer, "Temperature: %ld\r\n", temperature);
	uartSend(buffer);
	
	sprintf((char*)buffer, "RAW pressure: %ld\r\n", myData.pressure);
	uartSend(buffer);
		
	pressure = compensatePressure(myData.pressure);
		
	inPressure = ( (double)pressure) * 0.0002953;
		
	sprintf((char*)buffer, "Pressure: %.2f\r\n", inPressure );
	uartSend(buffer);
		
	sprintf((char*)buffer, "RAW humidity: %ld\r\n", myData.humidity);
	uartSend(buffer);
		
	humidity = compensateHumidity(myData.humidity);
	sprintf((char*)buffer, "Humidity: %.2f\r\n\r\n\r\n", humidity);
	uartSend(buffer);
		
	// Get Ambient light
	ambientLight = getUVdata(0x0d);
	lux = ambient2Lux(ambientLight);
		
	sprintf((char*)buffer, "RAW Ambient Light: %ld		%.2f lux\r\n", ambientLight, lux);
	uartSend(buffer);
		
	//LTR390 config
	i2c_write_byte(UVADDR, 0x00, 0x0a); //MAIN_CTL reg: UV mode
	_delay_ms(10);
			
	// get UV level
	rawUV = getUVdata(0x10);
	uvi = calcUVI(rawUV);
		
	sprintf((char*)buffer, "RAW UV: %ld		UV index: %ld \r\n", rawUV, uvi);
	uartSend(buffer);
		

    while (1)
    {

		// Force a measurement from the BMP280
		i2c_write_byte(BMPADDR, 0xf4, 0x91);
		_delay_ms(10);
		
		getTPHdata(0xf7);

//		sprintf((char*)buffer, "RAW Temperature: %ld\r\n", myData.temperature);
//		uartSend(buffer);

		temperature = compensateTemp(myData.temperature);
		sprintf((char*)buffer, "Temperature: %.2f deg.C\r\n", (float)(temperature/100.00));
		uartSend(buffer);

//		sprintf((char*)buffer, "RAW pressure: %ld\r\n", myData.pressure);
//		uartSend(buffer);

		pressure = compensatePressure(myData.pressure);
		inPressure = ( (double)pressure) * 0.0002953;

		sprintf((char*)buffer, "Pressure: %.2f inHg\r\n", inPressure );
		uartSend(buffer);

//		sprintf((char*)buffer, "RAW humidity: %ld\r\n", myData.humidity);
//		uartSend(buffer);

		humidity = compensateHumidity(myData.humidity);
		sprintf((char*)buffer, "Humidity: %.2f %%\r\n\r\n", humidity);
		uartSend(buffer);
		
		i2c_write_byte(UVADDR, 0x00, 0x02); //MAIN_CTL reg: ALS mode
		_delay_ms(10);
		
		// Get Ambient light
		i2c_write_byte(UVADDR, 0x00, 0x02); //MAIN_CTL reg: ALS mode
		_delay_ms(100);	
	
		ambientLight = getUVdata(0x0d);
		lux = ambient2Lux(ambientLight);

		sprintf((char*)buffer, "Ambient Light: %.2f lux\r\n", lux);
		uartSend(buffer);
	
		//LTR390 config
		i2c_write_byte(UVADDR, 0x00, 0x0a); //MAIN_CTL reg: UV mode
		_delay_ms(100);
	
		// get UV level
		rawUV = getUVdata(0x10);
		uvi = calcUVI(rawUV);
		
		sprintf((char*)buffer, "UV index: %ld \r\n\r\n\r\n", uvi);
		uartSend(buffer);
		
		// flash the amber LED(L)
        PORTB ^= (1 << PB5);
        _delay_ms(1000);
    }
    return 0;
}
