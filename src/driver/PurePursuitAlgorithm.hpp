#ifndef PURE_PURSUIT_ALGORITHM
#define PURE_PURSUIT_ALGORITHM

#define ROUTE_SIZE 1800

#include <math.h>

class State {
public:
    void init(float new_time_stamp, float new_position_x, float new_position_y, float new_yaw) {
        position_x = new_position_x;
        position_y = new_position_y;
        yaw = new_yaw;

        //updateでvを限りなく0にするために大きな値でtimes_stampを引く
        time_stamp = new_time_stamp - 10000.f;
        update(new_time_stamp, position_x, position_y, yaw);
    }
    bool update(float new_time_stamp, float new_position_x, float new_position_y, float new_yaw) {
        float dt = new_time_stamp - time_stamp;
        if (dt <= 0) {
            return false;
        }
        float velocity_x = (new_position_x - position_x) / dt;
        float velocity_y = (new_position_y - position_y) / dt;

        velocity = std::hypot(velocity_x, velocity_y);

        time_stamp = new_time_stamp;
        position_x = new_position_x;
        position_y = new_position_y;
        yaw = new_yaw;

        return true;
    }
    float getPositionX(){return position_x};
    float getPositionY(){return position_y};
    float getYaw(){return yaw};
    float getVelocity(){return velocity};

    float time_stamp, position_x, position_y, yaw, velocity;
};

class PurePursuitAlgorithm {
public:
    void init(float newX, float newY, float newYaw, float newV);
    float PIDControl();
    void update(float time_stamp, float position_x, float position_y, float yaw);
    void setTargetSpeed(float new_target_speed);
    float indexUpdate(int pind);
    //単位はそれぞれ s , m ,  m , rad
    float stateUpdate(float time_stamp, float position_x, float position_y, float yaw);
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

    float target_speed, current_speed;

    int ind, current_ind;
    State state;
    float cx[ROUTE_SIZE];
    float cy[ROUTE_SIZE];
    float angleList[ROUTE_SIZE];
};

#endif