#include "PurePursuitAlgorithm.hpp"

void PurePursuitAlgorithm::init(float time_stamp, float position_x, float position_y, float yaw) {
    state.init(time_stamp, position_x, position_y, yaw);
    ind = 0;
}
float PurePursuitAlgorithm::PIDControl() {
    return Kp * (target_speed - state.getVelocity());
}
void PurePursuitAlgorithm::update(float time_stamp, float position_x, float position_y, float yaw) {
    state.update(time_stamp, position_x, position_y, yaw);
    current_ind = calculateTargetIndex();

    float ai = PIDControl();
    float di = indexUpdate();
    current_ind = ind;

    float newVelocity = state.getVelocity() + ai;
    float newVelocityYaw = state.getVelocityYaw() - state.getYaw() + di;

    if (current_ind != ROUTE_SIZE - 1) {
        vel.linear.x = newVelocity * cosf(newVelocityYaw);
        vel.linear.y = newVelocity * sinf(newVelocityYaw);
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = getDifferenceAngle(angleList[current_ind], state.getYaw());

        std::cout << angleList[current_ind] << " , " << state.getYaw() << std::endl;
    }
    // } else {
    //     //最終地点に到達したときのみ 最終地点で停止する
    //     moveTargetPoint(ppa.cx[ROUTE_SIZE - 1], ppa.cy[ROUTE_SIZE - 1], angleList[ROUTE_SIZE - 1]);
    // }
}
void PurePursuitAlgorithm::setTargetSpeed(float new_target_speed) {
    target_speed = new_target_speed;
}
float PurePursuitAlgorithm::indexUpdate() {
    ind = calculateTargetIndex();

    if (current_ind >= ind) {
        ind = current_ind;
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

    float alpha = getDifferenceAngle(atan2f(ty - state.getPositionY(), tx - state.getPositionX()), state.getVelocityYaw());

    return alpha;
}
int PurePursuitAlgorithm::calculateTargetIndex() {
    float dx = cx[ind] - state.getPositionX();
    float dy = cy[ind] - state.getPositionY();
    float L = std::hypot(dx, dy);

    float Lf = k * state.getVelocity() + Lfc;
    while (Lf > L && (ind + 1) < ROUTE_SIZE) {
        dx = cx[ind] - state.getPositionX();
        dy = cy[ind] - state.getPositionY();
        L = std::hypot(dx, dy);
        ind += 1;
    }
    return ind;
}
float PurePursuitAlgorithm::calculateDistance(float pointX, float pointY) {
    float dx = state.getPositionX() - pointX;
    float dy = state.getPositionY() - pointY;
    return sqrtf(dx * dx + dy * dy);
}
//angle2を原点として、angle1との差を返す
float PurePursuitAlgorithm::getDifferenceAngle(float angle1, float angle2) {
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
float PurePursuitAlgorithm::convertAngle(float convertedAngle) {
    float a1 = convertedAngle - static_cast<float>(static_cast<int>(convertedAngle / (M_PI * 2.f))) * M_PI * 2.f;
    if (a1 > M_PI) {
        a1 -= M_PI * 2.f;
    } else if (a1 < M_PI * (-1.f)) {
        a1 += M_PI * 2.f;
    }

    return a1;
}

geometry_msgs::Twist PurePursuitAlgorithm::getCommandVelocity() {
    return vel;
};