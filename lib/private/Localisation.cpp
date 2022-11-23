#include "Localisation.h"

float correction(float raw, float angle) {
    angle = (angle > 180) ? 360 - angle : angle;
    if (angle < 60) {
        return raw * cos(deg_to_rad(angle));
    } else {
        return raw;
    } // SPIN ON SPOT cos angle too much
    return raw;
}

Localisation::Localisation(int LEFT_ADDR,
                           int RIGHT_ADDR,
                           int FRONT_ADDR,
                           int BACK_ADDR) :
    _LEFT_ADDR(LEFT_ADDR),
    _RIGHT_ADDR(RIGHT_ADDR), _BACK_ADDR(BACK_ADDR), _FRONT_ADDR(FRONT_ADDR) {
}

void Localisation::setup() {
    Wire1.setSCL(11);
    Wire1.setSDA(10);
    Wire1.begin();
4
    Wire.setSCL(21);
    Wire.setSDA(20);
    Wire.begin();

    ld_left.setup(_LEFT_ADDR, Wire1);
    ld_right.setup(_RIGHT_ADDR, Wire1);
    ld_back.setup(_BACK_ADDR, Wire);
    ld_front.setup(_FRONT_ADDR, Wire);
}

bool Localisation::update(float rot) {
    front = correction(ld_front.read() + yf_offset, rot);
    back  = correction(ld_back.read() + yb_offset, rot);
    left  = correction(ld_left.read() + xl_offset, rot);
    right = correction(ld_right.read() + xr_offset, rot);
    //    Serial.println(right);

    if (rot < 60 || rot > 300) {
        stopBot = false;
    } else {
        stopBot = true;
    }

    if ((front * back * left * right) <
        0) { // if ((front*back*left*right) < 0) {
        return 0;
    }
    return true;
}

void Localisation::calcCoord(float rot) {
    if (update(rot)) {
        coord_x = (left + (182 - right)) / 2;

        if (coord_x >= 60 && coord_x <= 122) {
            back += 20;
            front += 20;
        }

        //    if(coord_x >= 121 && coord_x < 122.5){
        //      back += 27;
        //      front += 27;
        //    }

        coord_y = (back + (243 - front)) / 2;
    }
    coord_x = constrain(coord_x, 0, 182);
    coord_y = constrain(coord_y, 0, 243);
}

bool Localisation::moveTo(float x, float y, bool checkConfidence, int rad) {
    float angle = PI + atan2(coord_x - x, coord_y - y);
    dir         = rad_to_deg(angle);

    dist = hypot(x - coord_x, y - coord_y);

    if (checkConfidence) {
        spd = max(0.1, min(pow(dist / MAX_MOVE_DIST, 0.6), 1) * DEFAULT_SPEED);

        if (dist < rad && x_confidence > 0.8) {
            return true;
        }
    } else {
        spd = 0.4;
        if (dist < 2) {
            return true;
        }
    }
    return false;
}

void Localisation::confidence() {
    //  float angle = deg_to_rad(dir);

    if (!stopBot) {
        //    spd_x = spd*sin(angle);
        //    spd_y = spd*cos(angle);

        x_confidence = pow(min(1, (right + left) / 182.0), 4); // ADJUST
        y_confidence = pow(min(1, (front + back) / 243.0), 2); // ADJUST

        x_constrain_max = x_confidence * DEFAULT_SPEED; // CHECK?
        x_constrain_min = -1 * x_constrain_max;

        y_constrain_max = y_confidence * DEFAULT_SPEED; // CHECK?
        y_constrain_min = -1 * y_constrain_max;

        spd_x = constrain(spd_x, x_constrain_min, x_constrain_max);
        spd_y = constrain(spd_y, y_constrain_min, y_constrain_max);

    } else {
        spd_x = 0;
        spd_y = 0;
    }
    spd = hypot(spd_x, spd_y);
    dir = rad_to_deg(atan2(spd_x, spd_y));

    //  Serial.println(spd);

    dir = (dir < 0) ? dir + 360 : dir;
}

void Localisation::boundaryScale(float spd, float dir) {
    float angle = deg_to_rad(dir);
    spd_x       = spd * sin(angle);
    spd_y       = spd * cos(angle);

    //  Serial.print(spd_x); Serial.print(" "); Serial.println(spd_y);

    if (dir < 180) {         // right
        if (coord_x < 145) { // in field
            x_constrain_max = pow((145 - coord_x) / 54, 0.8) * DEFAULT_SPEED;
        } else { // ouside field
            x_constrain_max =
                (-1) * pow((coord_x - 145) / 37, 0.3) * DEFAULT_SPEED;
        }
    } else { // left
             // STARDARDISED-STANDARDISE-STANDARDISE-STANDARDISE-DTANDARDISE-STANDARDISE-STANDARDISE-STANDARDISE-DTANDARDISE-STANDARDISE-STANDARDISE-STANDARDISE-DTANDARDISE-STANDARDISE-STANDARD
        if (coord_x > 45) { // in field
            x_constrain_min =
                (-1) * pow((coord_x - 45) / 46, 1) * DEFAULT_SPEED;
        } else { // ouside field
            x_constrain_min = pow((45 - coord_x) / 45, 0.1) * DEFAULT_SPEED;
        }
    }
    spd_x = constrain(spd_x, x_constrain_min, x_constrain_max);

    if (dir > 90 && dir < 270) { // back
        float min_y =
            (1 / 13.0) *
                (sqrt(-169.0 * sq(coord_x) + 30758.0 * coord_x + 183075.0) -
                 595.0) +
            6;

        if (coord_y > min_y) { // in field
            y_constrain_min = (-1) *
                              pow((coord_y - min_y) / (122.0 - min_y), 0.5) *
                              DEFAULT_SPEED;
            //      Serial.print("spd: "); Serial.println(y_constrain_min);
        } else { // ouside field
            y_constrain_min =
                pow((min_y - coord_y) / min_y, 0.25) * DEFAULT_SPEED;
            //      Serial.print("wot: "); Serial.println(y_constrain_min);
        }
    } else { // front
        float max_y = (1 / 13.0) * (3754 - sqrt(-169.0 * sq(coord_x) +
                                                30758.0 * coord_x + 183075.0)) -
                      6;
        if (coord_y < max_y) { // in field
            y_constrain_max =
                pow((max_y - coord_y) / (max_y - 122.0), 0.5) * DEFAULT_SPEED;
        } else { // ouside field
            y_constrain_max = -1 *
                              pow((coord_y - max_y) / (243.0 - max_y), 0.1) *
                              DEFAULT_SPEED;
            //      Serial.println(y_constrain_max);
        }
    }
    spd_y = constrain(spd_y, y_constrain_min, y_constrain_max);

    //  Serial.print(spd_x); Serial.print(" "); Serial.println(spd_y);
    //  Serial.println();
}

void Localisation::aim() {
    dir = rad_to_deg(PI + atan2(coord_x - GOAL_XCOORD, coord_y - GOAL_YCOORD));
}

bool Localisation::boundaryCheck() {
    if ((coord_x < 25 || coord_x > 157) || (coord_y < 25 || coord_y > 218)) {
        return false;
    }
    return true;
}

void deconstructVectors(float spd, float dir) {
    //  float angle = deg_to_rad(dir);
    //  spd_x = spd*sin(angle);
    //  spd_y = spd*cos(angle);
}

void Localisation::debug() {
    Serial.print(coord_x);
    Serial.print(" ");
    Serial.print(coord_y);
    Serial.print(" ");
    Serial.print("   ");

    Serial.print(front);
    Serial.print(" ");
    Serial.print(back);
    Serial.print(" ");
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print(" ");
    Serial.print("   ");

    Serial.print(spd);
    Serial.print(" ");
    Serial.print(dir);
    Serial.print(" ");
    Serial.println();

    Serial.print(spd_x);
    Serial.print(" ");
    Serial.print(spd_y);
    Serial.print("  ");
    Serial.print(x_confidence);
    Serial.print(" ");
    Serial.println(y_confidence);
}
