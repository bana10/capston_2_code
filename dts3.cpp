#include <iostream>
#include <sstream>
#include <iomanip>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cstring>

class Temperature
{
private:
    int16_t iSensor; 
    int16_t iObject;
    double temperature;

    const int SCE = 22;
    const int spi_chn0 = 0;
    const int SPEED_1MHz = 1000000;
    const int SPI_MODE3 = 3;
    const int OBJECT = 0xA0;
    const int SENSOR = 0xA1;
    
    int16_t SPI_COMMAND(uint8_t ADR){
	    uint8_t Data_Buf[3];

	    Data_Buf[0] = ADR;
	    Data_Buf[1] = 0x22;
	    Data_Buf[2] = 0x22;
	
	    digitalWrite(SCE, 0);  				// SCE LOW
	    delayMicroseconds(10);				// delay 10us

	    wiringPiSPIDataRW (spi_chn0, Data_Buf, 1);		// transfer 1st byte.
	    delayMicroseconds(10);				// delay 10us
	    wiringPiSPIDataRW (spi_chn0, Data_Buf+1, 1);	            // transfer 2nd byte
	    delayMicroseconds(10);				// delay 10us
	    wiringPiSPIDataRW (spi_chn0, Data_Buf+2, 1);		// transfer 3rd byte
	    delayMicroseconds(10);				// delay 10us

	    digitalWrite(SCE, 1);  				// SCE HIGH
	    return (Data_Buf[2]*256+Data_Buf[1]);			// High + Lo byte
    }

public: // Constructor
    Temperature(){
        temperature = 0;
        wiringPiSetup();					// Wiring Pi setup
 	    if(wiringPiSetupGpio() == -1) { return; }
	    pinMode(SCE, OUTPUT);				// SCE Port Output
	    digitalWrite(SCE,1);					// SCE high

	    wiringPiSPISetupMode(spi_chn0, SPEED_1MHz, SPI_MODE3); //SPI0, 1Mhz, SPI Mode3 Setting
	    delay(500);					// wait 500ms
    }

public: // public func
    void check() {
        iObject = SPI_COMMAND(OBJECT);
        delay(500);
        std::cout.precision(4);
        std::cout << "Object : " << (double)iObject / 100 << std::endl;
    }
    
    std::string get_result() {
        double object_value = static_cast<double>(iObject) / 100.0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << object_value;
        return oss.str();
    }


};

extern "C" {
    Temperature* Temperature_new(){
        return new Temperature();
    }

    void Temperature_check(Temperature* f){
        f->check();
    }
    const char* Temperature_get_result(Temperature* f) {
        std::string result = f->get_result();
        return strdup(result.c_str()); 
    }
}

int main() {
    Temperature* temp = Temperature_new();
    temp->check();
    delete temp;

    return 0;
}
