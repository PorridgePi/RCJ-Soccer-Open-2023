#include "Arduino.h"

const float pi = 3.14159265358979323846f; //probably will reduce the precision

//float CurrentOrientation = 30; //updated to the IMU readings every frame; Angle of the bot wrt the field
//float TargetOrientation = 0;
struct {float X = 0; float Y = 0; float Orientation = 0;} Posori; //Current Position and Orientation; updated by Localisation

//struct {float X = 0; float Y = 0; float Orientation = 0;} targetPosori; //Target Position and Orientation


signed char sign(float A) {
  return (A < 0) ? -1 : ((A > 0) ? 1 : 0);
};

float deltaAngle(float current, float target) { // Gets the smallest difference in 2 angle, including its direction 
  if (abs(current - target) < 180) {
    return target - current;
  } else {
    return target - current - 360;
  }
};

class Motor {
  private:
    const int _angle;
    const float _multiplier;
    const int _pin0;
    const int _pin1;

  public:
    void drive(float targetOrientation){//float velocityX, float velocityY, float currentOrientation, float targetOrientation) {
      float orientation = (_angle + Posori.Orientation) * pi / 180;
      float translationCommand = (cosf(orientation) * Posori.X - sinf(orientation) * Posori.Y);
      float rotationCommand = sign(deltaAngle(Posori.X, targetOrientation));
      float command = _multiplier * (translationCommand + rotationCommand);
      digitalWrite(_pin0, constrain(command, 0, 255));
      digitalWrite(_pin1, abs(constrain(command, -255, 0)));
    };
  Motor(float ANGL, float MULT, int PIN0, int PIN1) : _angle(ANGL), _multiplier(MULT), _pin0(PIN0), _pin1(PIN1) {
    pinMode(_pin0, OUTPUT);
    pinMode(_pin1, OUTPUT);
  } // SETS _angle = ANGL, _multiplier = MULT, etc.
};
