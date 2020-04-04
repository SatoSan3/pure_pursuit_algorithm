#include "PurePursuitAlgorithm.hpp"

void PurePursuitAlgorithm::init(float time_stamp, float position_x, float position_y, float yaw)
{
    state.init(time_stamp, position_x, position_y, yaw);
    ind = 0;
}
float PurePursuitAlgorithm::PIDControl()
{
    return Kp * (target_speed - state.getVelocity());
}
void PurePursuitAlgorithm::update(float time_stamp, float position_x, float position_y, float yaw)
{
    state.update(time_stamp, position_x, position_y, yaw);
    if (path.poses.size() <= 0) {
        return;
    }
    calculateTargetIndex();

    float ai = PIDControl();
    indexUpdate();
    float di = getDifferenceAngle(atan2f(path.poses[ind].pose.position.y - state.getPositionY(), path.poses[ind].pose.position.x - state.getPositionX()), state.getVelocityYaw());

    float newVelocity = state.getVelocity() + ai;
    float newVelocityYaw = state.getVelocityYaw() - state.getYaw() + di;

    if (ind != ROUTE_SIZE - 1) {
        vel.linear.x = newVelocity * cosf(newVelocityYaw);
        vel.linear.y = newVelocity * sinf(newVelocityYaw);
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        float temp_angle = std::atan2(path.poses[ind].pose.orientation.z, path.poses[ind].pose.orientation.w) * 2;
        vel.angular.z = getDifferenceAngle(temp_angle, state.getYaw());
    }
}
void PurePursuitAlgorithm::setTargetSpeed(float new_target_speed)
{
    target_speed = new_target_speed;
}
void PurePursuitAlgorithm::indexUpdate()
{
    int old_ind = ind;
    ind = calculateTargetIndex();

    if (old_ind >= ind) {
        ind = old_ind;
    }

    float tx, ty;

    if (ind >= path.poses.size()) {
        ind = path.poses.size() - 1;
    }

    //float alpha = getDifferenceAngle(atan2f(ty - state.getPositionY(), tx - state.getPositionX()), state.getVelocityYaw());
}
int PurePursuitAlgorithm::calculateTargetIndex()
{
    float dx = path.poses[ind].pose.position.x - state.getPositionX();
    float dy = path.poses[ind].pose.position.y - state.getPositionY();
    float L = std::hypot(dx, dy);

    float Lf = k * state.getVelocity() + Lfc;
    while (Lf > L && (ind + 1) < ROUTE_SIZE) {
        dx = path.poses[ind].pose.position.x - state.getPositionX();
        dy = path.poses[ind].pose.position.y - state.getPositionY();
        L = std::hypot(dx, dy);
        ind += 1;
    }
    return ind;
}
float PurePursuitAlgorithm::calculateDistance(float pointX, float pointY)
{
    float dx = state.getPositionX() - pointX;
    float dy = state.getPositionY() - pointY;
    return std::hypot(dx, dy);
}
//angle2を原点として、angle1との差を返す
float PurePursuitAlgorithm::getDifferenceAngle(float angle1, float angle2)
{
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
float PurePursuitAlgorithm::convertAngle(float convertedAngle)
{
    float a1 = convertedAngle - static_cast<float>(static_cast<int>(convertedAngle / (M_PI * 2.f))) * M_PI * 2.f;
    if (a1 > M_PI) {
        a1 -= M_PI * 2.f;
    } else if (a1 < M_PI * (-1.f)) {
        a1 += M_PI * 2.f;
    }

    return a1;
}

geometry_msgs::Twist PurePursuitAlgorithm::getCommandVelocity()
{
    return vel;
};

void PurePursuitAlgorithm::setPath(const nav_msgs::Path& new_path)
{
    path = new_path;
}

bool PurePursuitAlgorithm::judgeGoal()
{
    if (ind == (path.poses.size() - 1)) {
        return true;
    }
    return false;
}