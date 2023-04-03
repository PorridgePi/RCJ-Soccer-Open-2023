#include <bits/stdc++.h>
using namespace std;

#define PI 3.1415926535897932384626433832795

#define SPEED 0.3
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RAD(x) ((x) / 180.0f * (float) PI)

float moveAngle = 225;
int x = 100;
int y = 150;

float speedX, speedY;
float speed = SPEED;

int main() {
    while (x>0) {

        speedX = speed * sin(RAD(moveAngle));
        speedY = speed * cos(RAD(moveAngle));
        cout << x << '\t' << y << '\t';

        cout << speedX << '\t' << speedY << '\t';

        const int borderDist = 12;
        const int borderTolerance = 9;

        float multiplierX;
        int distanceX;
        if (moveAngle >= 0 && moveAngle < 180) { // moving right
            distanceX = (182 - borderDist - borderTolerance) - x;
        } else if (moveAngle >= 180 && moveAngle < 360) { // moving left
            distanceX = x - borderDist - borderTolerance;
        }
        cout << distanceX << "\t";
        multiplierX = constrain(3 * sqrt((float) distanceX/91.0f), 0, 1);
        speedX = constrain(speedX, -SPEED * multiplierX, SPEED * multiplierX);

        float multiplierY;
        int distanceY;
        if (moveAngle >= 90 && moveAngle < 270) { // moving down
            int maxY = max(borderDist, (3 / 80 * (x - 182 / 2)) ^ 4 + 243 - 38); // 38 is wall to goal border
            distanceY = maxY - y;
        } else if (moveAngle >= 270 || moveAngle < 90) { // moving up
            int maxY = max(borderDist, - (3 / 80 * (x - 182 / 2)) ^ 4 + 38);
            distanceY = y - maxY;
        }
        cout << distanceY << "\t";
        multiplierY = constrain(3 * sqrt((float) distanceY/121.5f), 0, 1);
        speedY = constrain(speedY, -SPEED * multiplierY, SPEED * multiplierY);
        x--;
        // y++;
        
        cout << "mul " << multiplierX << '\t' << multiplierY << '\t';
        cout << speedX << '\t' << speedY << '\t';
        cout << '\n';
    }

    return 0;
}
