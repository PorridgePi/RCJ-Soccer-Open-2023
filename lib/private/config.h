#ifndef CONFIG_h
#define CONFIG_h

#define deg_to_rad(deg) ((deg) *M_PI / 180.0)
#define rad_to_deg(rad) ((rad) *180.0 / M_PI)

/*CAM.h*/
#define CAMERA_BAUDRATE 115200
#define CAMERA_POS      254
#define CAMERA_NEG      255

#define CAMERA_NOBALL 246
#define CAMERA_BLOW   247
#define CAMERA_BHIGH  248
#define CAMERA_YLOW   249
#define CAMERA_YHIGH  250
#define CAMERA_BGOAL  251
#define CAMERA_YGOAL  252
#define CAMERA_BALL   253

/*Localisation.h*/
#define DEFAULT_SPEED 0.7
#define MAX_MOVE_DIST 100
#define GOAL_XCOORD   91
#define GOAL_YCOORD   200

#define TFMINIPLUS_BAUDRATE 115200
#define BLUETOOTH_BAUDRATE  115200

// --> TO BE ADDED  //
/*Kicker.h*/
#define KICKER_PIN  16
#define kickerDelay 3000 // ms

/*Dribbler.h*/
#define BALLSENS_THRESH 381
#define DRIBBLER_SPEED  255

/*Drivebase.h*/
// #define maxBallDist 65
#define DEBUG

// struct bot {
//   float spd;
//   float dir;
//   float rot;
// }
#define OFFENSIVE_GOAL 'y'

#endif
