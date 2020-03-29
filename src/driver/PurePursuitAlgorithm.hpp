#ifndef PURE_PURSUIT_ALGORITHM
#define PURE_PURSUIT_ALGORITHM

#define ROUTE_SIZE 1800

#include <math.h>

class State {
public:
    void init(float newX, float newY, float newYaw, float newV) {
        x = newX;
        y = newY;
        yaw = newYaw;
        v = newV;
    }
    void update(float positionX, float positionY, float angle, float speed) {
        x = positionX;
        y = positionY;
        yaw = angle;
        v = speed;
    }
    float x, y, yaw, v;
};

class PurePursuitAlgorithm {
public:
    void init(float newX, float newY, float newYaw, float newV);
    float PIDControl(float target, float current);
    float update(int pind);
    int calculateTargetIndex();
    float calculateDistance(float pointX, float pointY);
    //angle2を原点として、angle1との差を返す
    float getDifferenceAngle(float angle1, float angle2);
    float convertAngle(float convertedAngle);

    const float k = 0.4;    // look forward gain
    const float Lfc = 0.20; // look-ahead distance
    const float Kp = 1.0;   //speed proportional gain
    const float dt = 0.1;   //[s]
    const float L = 0.9;    //[m] wheel base of vehicle

    int ind;
    State state;
    float cx[ROUTE_SIZE];
    float cy[ROUTE_SIZE];
};

#endif