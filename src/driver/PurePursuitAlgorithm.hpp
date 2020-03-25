#ifndef PURE_PURSUIT_ALGORITHM
#define PURE_PURSUIT_ALGORITHM

#define ROUTE_SIZE 1800

#include <math.h>

class State {
public:
    void init(float newX, float newY, float newYaw, float newV,
              float newDt, float newL) {
        x = newX;
        y = newY;
        yaw = newYaw;
        v = newV;
        dt = newDt;
        L = newL;
    }
    void update(float positionX, float positionY, float angle, float speed) {
        x = positionX;
        y = positionY;
        yaw = angle;
        v = speed;
        rearX = x - ((L / 2) * cosf(angle));
        rearY = y - ((L / 2) * sinf(angle));
    }
    float x, y, yaw, v;
    float dt, L; //dt[s], L[m] wheel base of vehicle
    float rearX, rearY;
};

class PurePursuitAlgorithm {
public:
    void init(float newX, float newY, float newYaw, float newV) {
        state.init(newX, newY, newYaw, newV, dt, L);
        ind = 0;
    }
    float PIDControl(float target, float current) {
        float a = Kp * (target - current);
        return a;
    }
    float update(int pind) {
        ind = calculateTargetIndex();

        if (pind >= ind) {
            ind = pind;
        }

        float tx, ty;

        if (ind < ROUTE_SIZE) {
            tx = cx[ind];
            ty = cy[ind];
        } else {
            tx = cx[ROUTE_SIZE - 1];
            ty = cy[ROUTE_SIZE - 1];
            ind = ROUTE_SIZE - 1;
        }

        float alpha = getDifferenceAngle(atan2f(ty - state.y, tx - state.x), state.yaw);

        return alpha;
    }
    int calculateTargetIndex() {
        float dx = cx[ind] - state.x;
        float dy = cy[ind] - state.y;
        float L = std::hypot(dx, dy);

        float Lf = k * state.v + Lfc;
        while (Lf > L && (ind + 1) < ROUTE_SIZE) {
            dx = cx[ind] - state.x;
            dy = cy[ind] - state.y;
            L = std::hypot(dx, dy);
            ind += 1;
        }
        return ind;
    }
    float calculateDistance(float pointX, float pointY) {
        float dx = state.x - pointX;
        float dy = state.y - pointY;
        return sqrtf(dx * dx + dy * dy);
    }
    //angle2を原点として、angle1との差を返す
    float getDifferenceAngle(float angle1, float angle2) {
        float a1 = convertAngle(angle1);
        float a2 = convertAngle(angle2);

        float tempAngleA = a1 - a2;
        float tempAngleB = (a1 - M_PI * 2.f) - a2;
        float tempAngleC = a1 - (a2 - M_PI * 2.f);

        bool ab = (tempAngleA * tempAngleA < tempAngleB * tempAngleB);
        bool ac = (tempAngleA * tempAngleA < tempAngleC * tempAngleC);
        bool bc = (tempAngleB * tempAngleB < tempAngleC * tempAngleC);

        if (ab == true && ac == true) {
            return tempAngleA;
        }
        if (ab == false && bc == true) {
            return tempAngleB;
        }
        if (ac == false && bc == false) {
            return tempAngleC;
        }

        return 0;
    }
    float convertAngle(float convertedAngle) {
        float a1 = convertedAngle - static_cast<float>(static_cast<int>(convertedAngle / (M_PI * 2.f))) * M_PI * 2.f;
        if (a1 > M_PI) {
            a1 -= M_PI * 2.f;
        } else if (a1 < M_PI * (-1.f)) {
            a1 += M_PI * 2.f;
        }

        return a1;
    }
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