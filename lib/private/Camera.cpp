#include "Camera.h"

Camera::Camera(HardwareSerial &CAM_SERIAL, int tx, int rx) :
    _CAM_SERIAL(CAM_SERIAL), _tx(tx), _rx(rx) {
}

void Camera::setup() {
    //  Serial2.setTX(_tx);
    //  Serial2.setRX(_rx);

    _CAM_SERIAL.begin(CAMERA_BAUDRATE);
    delay(1000);
    //  setGoal();
    Serial.println("Camera setup done.");
}

// bool Camera::setGoal() { // SET TIMER OF (1 SEC?) --> IF NO GOAL SET WITHIN
// TIME LIMIT --> IGNORE AND USE LOCALISATION FOR AIMING UNTIL GOAL IS SET
//  while (!blue || !yellow){
//    read();
//  }
// }

void Camera::read() {
    while (_CAM_SERIAL.available()) {
        byte incomByte = _CAM_SERIAL.read();

        if (incomByte != '\r' &&
            incomByte != '\n') { // ANY OTHER ALTERNATIVES AROUND THIS????
            for (int i = packetIdx; i > 0; i--) {
                cameraBuffer[i] = cameraBuffer[i - 1];
            }
            cameraBuffer[0] = incomByte;
            byte flag       = cameraBuffer[packetIdx];

            if (flag == 253) {
                ball_ = true;

                ball_dist = distMap[constrain(
                    cameraBuffer[0] - min_px,
                    0,
                    max_range)]; // add convertion from px to dist {CHANGE THE
                                 // OUTPUT FORMAT OF CAMERA FROM REAL_DIST TO
                                 // PX_DIST THEN ADD THE CONVERSION ARRAY TO
                                 // ARDUINO CODE}
                ball_dir = constrain(360 - cameraBuffer[1] * 1.44, 0, 360);
                orbit();
                //        Serial.print(ball_dir); Serial.print(" ");
                //        Serial.println(ball_dist);
            }

            if (flag == 252) {
                yellow_goal = true;

                yellow_dist = distMap[constrain(
                    cameraBuffer[0] - min_px,
                    0,
                    max_range)]; // add convertion from px to dist
                yellow_dir  = constrain(360 - cameraBuffer[1] * 1.44, 0, 360);
            }

            if (flag == 251) {
                blue_goal = true;

                blue_dist = distMap[constrain(
                    cameraBuffer[0] - min_px,
                    0,
                    max_range)]; // add convertion from px to dist
                blue_dir  = constrain(360 - cameraBuffer[1] * 1.44, 0, 360);
            }

            if (flag == 255) {
                ball_ = false;
            }
            //	    if (flag == 255){
            //        yellow_goal = false;
            //	    }
            //	    if (flag == 254){
            //        blue_goal = false;
            //	    }
            //      for (int i =0; i < 3; i++){
            //        Serial.print(cameraBuffer[i]); Serial.print(" ");
            //      }
            //      Serial.println();
        }
    }
}

void Camera::getDeltaBall() {
    if (ball_dir > 180) {
        ball_dir -= 360;
    }

    delta_ball_x = ball_dist * sin(deg_to_rad((float) ball_dir));
    delta_ball_y = ball_dist * cos(deg_to_rad((float) ball_dir));
}

void Camera::orbit() {
    int orbit_dir;

    //  Serial.print("ball: "); Serial.print(ball_dist); Serial.print(" ");
    //  Serial.println(ball_dir);

    if (ball_dir >= 0 && ball_dir < 90) {
        orbit_dir = ball_dir * (2 - pow(ball_dist / 175.0, 0.8));
    } else if (ball_dir >= 90 && ball_dir < 180) {
        orbit_dir = ball_dir + 90 * (1 - pow(ball_dist / 175.0, 0.6));
    } else if (ball_dir >= 180 && ball_dir < 270) {
        orbit_dir = ball_dir - 90 * (1 - pow(ball_dist / 175.0, 0.6));
    } else if (ball_dir >= 270 && ball_dir < 360) {
        orbit_dir = 360 - (360 - ball_dir) * (2 - pow(ball_dist / 175.0, 0.8));
    }

    dir = orbit_dir; // ball_dir;
    spd = (pow(ball_dir / 360.0, 0.4) + min(ball_dist / 160.0, 1)) /
          2.0; // max(0.3, pow(ball_dir/360.0, 0.4));
    //  Serial.println(spd);
}

bool Camera::aim() {
    if (OFFENSIVE_GOAL == 'y' && yellow_goal) {
        dir = yellow_dir;
        return true;
    } else if (OFFENSIVE_GOAL == 'b' && blue_goal) {
        dir = blue_dir;
        return true;
    }
    return false;
}

bool Camera::ballCap() {
    if (ball_dist < 12 && (ball_dir > 355 || ball_dir < 5)) {
        return true;
    } else {
        return false;
    }
}

bool Camera::ballInField() {
    return ball_;
}

bool Camera::goalInField() {
    return true;
}

// void Camera::updateBot() { //dir, dist, check goal
////	read();
////  if (ball){
////	  orbit();
////  }
//}
