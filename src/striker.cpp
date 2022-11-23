#include "BasePlate.h"
#include "Bluetooth.h"
#include "Camera.h"
#include "Drivebase.h"
#include "IMU.h"
#include "Localisation.h"
#include "Motors.h"
#include <elapsedMillis.h>

elapsedMillis timer;
elapsedMillis kickTimer;

int  kickPin    = 16;
bool kick_state = 0;

Motor A(14, 15);
Motor B(2, 3);
Motor C(26, 22);
Motor D(27, 28);

IMU    imu(19);
Camera cam(Serial2, 4, 5);

Bluetooth bluetooth(Serial1);

Drivebase    db(A, B, C, D);
Localisation local(0x11, 0x10, 0x10, 0x30); // left, right, front, back
// Localisation local(0x30, 0x10, 0x30, 0x10); //left, right, front, back

float ball_x, ball_y;
float spd = DEFAULT_SPEED, dir, rot;

bool kickerEnabled = true, kickState = false;

void setup() {
    Serial.begin(9600);

    local.setup();
    imu.setup();
    cam.setup();

    pinMode(kickPin, OUTPUT);
    digitalWrite(kickPin, HIGH);

    kickTimer = 3000;
}

void updateBotSensors() {
    rot = imu.read(); // ROTATION SET
    cam.read();

    if (kickTimer > 2200 && kickState == true) {
        digitalWrite(kickPin, HIGH);
        kickTimer = 0;
        kickState = false;
    }

    local.calcCoord(rot);
}

bool boundCheck() { // base trigger --> check localisation (if bot is
                    // withinBound && confidence > THRESH) --> ignore trigger
                    // --> else move to centre
    if (local.boundaryCheck()) { // || base.boundaryCheck()){
        return true;
    } else {
        return false;
    }
}

void attack() {
    if (cam.aim()) { // (cam) move to goal
        dir = cam.dir;
    } else { // (local) move to goal
        local.aim();
        dir = local.dir;
    }

    if ((local.coord_x > 60 && local.coord_x < 123) && (local.coord_y > 90)) {
        // kickBall();
    }
    spd = DEFAULT_SPEED;
}

void kickBall() {
    if (kickTimer > 2000 && kickState == false) {
        digitalWrite(kickPin, LOW);
        kickState = true;
    }
}

void balltrack() {
    cam.orbit();
    dir = cam.dir;
    spd = cam.spd;
}

void sendBallData() {
    cam.getDeltaBall();

    ball_x = local.coord_x + cam.delta_ball_x;
    ball_y = local.coord_y + cam.delta_ball_y;

    //  Serial.print(ball_x); Serial.print(" "); Serial.println(ball_y);

    //  bluetooth.write(ball_x, ball_y);
}

void readBallData() {
}

// check if bot in bounds --> else move to centre
// check if bot has the ball --> else if ball in field --> ballTrack --> else
// return centre check if cam can see goal --> move to goal --> else retreat?/
// use localisation to aim and score move to goal & score

void loop() {
    //  kickBall();
    //  int start = millis();
    updateBotSensors();

    if (boundCheck()) {          // if bot in field
        if (cam.ballInField()) { // if ball in field

            //      sendBallData();

            if (cam.ballCap()) { // if bot has the ball
                //        Serial.println("attack");
                attack();
            } else { // else chase ball
                //        Serial.println("ballTrack");
                balltrack();
            }
        } else {
            //      Serial.println("center - no ball");
            local.moveTo(91, 122); // move center
            spd = local.spd;
            dir = local.dir;
        }
    } else {
        //      Serial.println("center - out field");
        local.moveTo(91, 122); // move center
        spd = local.spd;
        dir = local.dir;
    }

    local.boundaryScale(spd, dir);
    local.confidence();
    //  local.debug();
    Serial.println(kickTimer);
    db.drive(constrain(local.spd, 0, 0), local.dir, rot);
}
