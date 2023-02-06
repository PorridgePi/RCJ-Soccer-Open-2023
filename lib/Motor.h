#include <Arduino.h>

const float pi = 3.14159265358979323846f; //probably will reduce the precision

float CurrentOrientation = 30; //updated to the IMU readings every frame; Angle of the bot wrt the field
float TargetOrientation = 0;
struct {float X = 0; float Y = 0;} Velocity; //Velocity vector

signed char sign(float A) {
  return (A < 0) ? -1 : ((A > 0) ? 1 : 0);
}
float deltaAngle(float current, float target) { // Gets the smallest difference in 2 angle, including its direction 
  if (abs(current - target) < 180) {
    return target - current;
  } else {
    return target - current - 360;
  }
}

class Motor {
  private:
    int angle = 0;
    float multiplier = 1;

  public:
  float rotate(float X, float Y) {
     float ORIENTATION = (angle + CurrentOrientation) * pi / 180;
  return multiplier * ((cosf(ORIENTATION) * X - sinf(ORIENTATION) * Y) + sign(deltaAngle(CurrentOrientation, TargetOrientation)));
  }
  Motor(float ANGL, float MULT) {
    angle = ANGL;
    multiplier = MULT;    
  }
};
Motor A(-45, 1); Motor B(45, 1); Motor C(135, 1); Motor D(-135, 1); 

/*float rotate(float X, float Y, Motor MOTOR) {
  float ORIENTATION = (MOTOR.Angle + CurrentOrientation) * pi / 180;
  return MOTOR.Multiplier * (cosf(ORIENTATION) * X - sinf(ORIENTATION) * Y);
}*/

void setup() {

//ASSIGN DETAILS TO EACH MOTOR
//Might delete later if its too memory intensive
//A.angle = -45;B.angle = 45;C.angle = 135;D.angle = -135;

Serial.begin(9600);

}



void loop() {
  Serial.print(" A:");
  Serial.print(A.rotate(Velocity.X,Velocity.Y));

  Velocity.Y = Velocity.Y + 0.01;
  Serial.println(Velocity.Y);

  delay(100);

  //Serial.pri (rotate(1,1,pi/4));

  // put your main code here, to run repeatedly:



}
