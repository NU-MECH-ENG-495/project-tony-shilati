#ifndef _SPI_ENCODER_
#define _SPI_ENCODER_

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>

#define PI 3.1415926535897932384626433832795

class SPI_Encoder: public Sensor{
    public:
        SPI_Encoder(int chip_select = 10, int speed = 10000000, int bit_order = MSBFIRST, int mode = SPI_MODE1)
        {
            cs = chip_select;
            setting = SPISettings(speed, bit_order, mode);
            pinMode(cs,OUTPUT);
            digitalWriteFast(cs,HIGH);
            SPI.begin();
            SPI.beginTransaction(setting);  
        }

        float getSensorAngle(){
            digitalWriteFast(cs, LOW);
            uint16_t nop = SPI.transfer16(read_cmd);
            digitalWriteFast(cs, HIGH);
            delayNanoseconds(400);
            digitalWriteFast(cs, LOW);
            uint16_t raw = SPI.transfer16(read_cmd);
            digitalWriteFast(cs, HIGH);
            return (raw&16383)/16383.0*2*PI;
        }
    private:
        SPISettings setting;
        int cs;
        uint16_t const read_cmd = (0b11<<14) | 0x3FFF;
};
#endif