#ifndef PURE_PURSUIT_ALGORITHM
#define PURE_PURSUIT_ALGORITHM

#define ROUTE_SIZE 1800

#include <geometry_msgs/Twist.h>
#include <math.h>

#include <nav_msgs/Path.h>

class State
{
public:
    void init(float new_time_stamp, float new_position_x, float new_position_y, float new_yaw)
    {
        time_stamp = new_time_stamp;
        position_x = new_position_x;
        position_y = new_position_y;
        yaw = new_yaw;
        velocity = 0;
        velocityYaw = 0;
    }
    bool update(float new_time_stamp, float new_position_x, float new_position_y, float new_yaw)
    {
        float dt = new_time_stamp - time_stamp;
        if (dt <= 0) {
            return false;
        }
        float velocity_x = (new_position_x - position_x) / dt;
        float velocity_y = (new_position_y - position_y) / dt;

        velocity = std::hypot(velocity_x, velocity_y);
        velocityYaw = atan2f(velocity_x, velocity_y);

        time_stamp = new_time_stamp;
        position_x = new_position_x;
        position_y = new_position_y;
        yaw = new_yaw;

        return true;
    }
    float getPositionX() { return position_x; }
    float getPositionY() { return position_y; }
    float getYaw() { return yaw; }
    float getVelocity() { return velocity; }
    float getVelocityYaw() { return velocityYaw; }

    float time_stamp, position_x, position_y, yaw, velocity, velocityYaw;
};

class PurePursuitAlgorithm
{
public:
    void init(float newX, float newY, float newYaw, float newV);
    void update(float time_stamp, float position_x, float position_y, float yaw);
    void setTargetSpeed(float new_target_speed);
    void setPath(const nav_msgs::Path& new_path);
    geometry_msgs::Twist getCommandVelocity();
    bool judgeGoal();

private:
    float PIDControl();
    void indexUpdate();
    //単位はそれぞれ s , m ,  m , rad
    float stateUpdate(float time_stamp, float position_x, float position_y, float yaw);
    int calculateTargetIndex();
    //angle2を原点として、angle1との差を返す
    float getDifferenceAngle(float angle1, float angle2);
    float convertAngle(float convertedAngle);
    float calculateDistance(float pointX, float pointY);

    const float k = 0.4;    // look forward gain
    const float Lfc = 0.20; // look-ahead distance
    const float Kp = 1.0;   //speed proportional gain
    float target_speed;
    int ind;
    State state;
    nav_msgs::Path path;
    geometry_msgs::Twist vel;
};

#endif