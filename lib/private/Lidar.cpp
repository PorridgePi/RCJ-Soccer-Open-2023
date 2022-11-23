#include "Lidar.h"

LidarPlusI2C::LidarPlusI2C() { }

void LidarPlusI2C::setup(int def_addr, TwoWire &LIDAR_I2C_BUS, int set_addr) {
    tf.Set_Bus(&LIDAR_I2C_BUS);
    delay(50);

    if (set_addr != 0x00) {
        tf.Set_I2C_Addr(set_addr, def_addr);

        def_addr = set_addr;
    }

    tf.Set_Frame_Rate(tfFPS, def_addr);
    tf.Soft_Reset(def_addr);
    tf.Save_Settings(def_addr);
    delay(50);

    _addr = def_addr;
    Serial.println("Lidar setup done.");
}

float LidarPlusI2C::read() {
    //  if (timer >= 4) {
    tf.getData(tfDist, _addr);
    //    timer = 0;
    return tfDist;
    //  }
    //  return -1;
}

//  addr = 0x10;
//  tf.Set_I2C_Addr(addr, 0x11);

/*
#include "Lidar.h"

LidarPlusI2C ld1;
//int new_addr = 0x10;
int def_addr = 0x30;

void setup(){
  Serial.begin(9600);

//  Wire1.setSCL(11);
//  Wire1.setSDA(10);
//  Wire1.begin();
//
//  ld1.setup(def_addr, Wire1, new_addr);

//  Wire.setSCL(21);
//  Wire.setSDA(20);
//  Wire.begin();
//
//  ld1.setup(def_addr, Wire, new_addr);
//  Serial.print("Lidar address set to: "); Serial.println(new_addr);

  Wire.setSCL(21);
  Wire.setSDA(20);
  Wire.begin();

  ld1.setup(def_addr, Wire);
}

void loop() {
  Serial.println(ld1.read());
}
 */
