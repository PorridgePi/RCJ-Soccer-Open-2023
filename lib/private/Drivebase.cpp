#include "Drivebase.h"

Drivebase::Drivebase(Motor &A, Motor &B, Motor &C, Motor &D) :
    ma(A), mb(B), mc(C), md(D) {
    analogWriteFreq(1000);
}

void Drivebase::drive(double spd, float dir, float rot) {
    // check values (spd done in motor library)
    rot = min(rot, 360);
    dir = min(dir, 360);

    // derived values (dir and spd)
    float ref_dir = deg_to_rad(dir - 45);
    float vx      = abs(spd * cos(ref_dir)),
          vy      = abs(spd * sin(ref_dir)); // Motors B&D: vx, A&C: vy
    float m_rot =
        constrain((rot < 180) ? 3 * (rot / 180) : 3 * ((360 - rot) / 180),
                  0,
                  1);

    // Motors A&C (y-axis)
    if (dir >= 45 && dir < 225) { // y-axis - backwards
        if (rot < 180) {          // anticlockwise rot
            ma.move(-vy);         //(1 - (2*rot_perc))
            mc.move(-vy * (1 - (2 * m_rot)));
        } else { // clockwise rot
            ma.move(-vy * (1 - (2 * m_rot)));
            mc.move(-vy);
        }
    } else {             // y-axis - forwards
        if (rot < 180) { // anticlockwise rot
            ma.move(vy * (1 - (2 * m_rot)));
            mc.move(vy);
        } else { // clockwise rot
            ma.move(vy);
            mc.move(vy * (1 - (2 * m_rot)));
        }
    }

    // Motors B&D (x-axis)
    if (dir >= 135 && dir < 315) { // x-axis - backwards
        if (rot < 180) {           // anticlockwise rot
            mb.move(-vx);
            md.move(-vx * (1 - (2 * m_rot)));
        } else { // clockwise rot
            mb.move(-vx * (1 - (2 * m_rot)));
            md.move(-vx);
        }
    } else {             // x-axis - forwards
        if (rot < 180) { // anticlockwise rot
            mb.move(vx * (1 - (2 * m_rot)));
            md.move(vx);
        } else { // clockwise rot
            mb.move(vx);
            md.move(vx * (1 - (2 * m_rot)));
        }
    }
}

// void Drivebase::activeAIM(float rot){
//
// }
