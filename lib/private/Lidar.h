#include <Arduino.h>
#include <Wire.h>

class Lidar {
    public:
        Lidar(int addr) : _addr(addr) { // constructor
        };
  
        float read(){
          Wire.beginTransmission(_addr); //start bit
          Wire.write(0x00); //tell tfluna which register to read from, this register is low dist
          Wire.endTransmission(); //end bit
          Wire.requestFrom(_addr, 1); //request 1 bit
          return Wire.read(); //read the bit
        }
  
        int get_FPS(){
          Wire.beginTransmission(_addr);
          Wire.write(0x26); //this register is low FPS
          Wire.endTransmission();
          Wire.requestFrom(_addr, 1);
          return Wire.read();
        }
  
        void set_FPS(int fps){
          Wire.beginTransmission(_addr);
          Wire.write(0x26);
          Wire.write(fps);
          Wire.endTransmission();
        }
  
        void change_Address(int newaddr){
          Wire.beginTransmission(_addr); //start bit
          Wire.write(0x22); //this register is the slave address
          Wire.write(newaddr); //write data, which is the new address
          Wire.endTransmission(); //end bit
          _addr = newaddr;
        } 

    private:
        int _addr;
};
