#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <string>

#include "PurePursuitAlgorithm.hpp"

namespace {
PurePursuitAlgorithm ppa;

float angleList[ROUTE_SIZE];
float speedVector;
float speedVectorAngle;
float x = 0, y = 0, angle = 0;
} // namespace

class VirtualOmniWheels {
public:
    void init(ros::NodeHandle nh) {
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        positionX = 0;
        positionY = 0;
        machineAngle = 0;
        update();
    }
    void setSpeed(float x, float y, float angle) {
        geometry_msgs::Twist vel;
        vel.linear.x = x;
        vel.linear.y = y;
        vel.angular.z = angle;
        pub.publish(vel);
    }
    //オドメトリの情報を更新する
    void update() {
        //オドメトリ(tf)取得の準備
        static tf::TransformListener ln;
        const std::string target_frame = "odom";

        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id = "base_footprint";
        source_pose.pose.orientation.w = 1.0;

        ros::Time time = ros::Time::now();

        static float oldTime = 0;
        float newTime = time.sec + time.nsec / (1000.f * 1000.f * 1000.f);
        float dt = newTime - oldTime;

        static float oldPositionX = 0;
        static float oldPositionY = 0;

        geometry_msgs::PoseStamped target_pose;
        try {
            ln.waitForTransform(source_pose.header.frame_id, target_frame, time, ros::Duration(1.0));
            ln.transformPose(target_frame, source_pose, target_pose);

            positionX = target_pose.pose.position.x;
            positionY = target_pose.pose.position.y;
            machineAngle = atan2f((target_pose.pose.orientation.x + target_pose.pose.orientation.y + target_pose.pose.orientation.z), target_pose.pose.orientation.w) * 2.f;

            speedVector = std::hypot(positionX - oldPositionX, positionY - oldPositionY) / dt;
            speedVectorAngle = atan2f(positionY - oldPositionY, positionX - oldPositionX);

            oldPositionX = positionX;
            oldPositionY = positionY;
            oldTime = newTime;

        } catch (...) {
            ROS_INFO("tf error");
        }
    }
    float positionX, positionY, machineAngle, speed, speedAngle;

private:
    ros::Publisher pub;
};
namespace {
VirtualOmniWheels ow;
}

float getPositionX() {
    return ow.positionX * cosf(angle) - ow.positionY * sinf(angle) + x;
}

float getPositionY() {
    return ow.positionX * sinf(angle) + ow.positionY * cosf(angle) + y;
}
float getAngle() {
    return angle + ow.machineAngle;
}

void updateSpeedVector() {
    speedVector = ow.speed;
    speedVectorAngle = ow.speedAngle;
}

void updateState() {
    updateSpeedVector();
    ppa.state.update(getPositionX(), getPositionY(), speedVectorAngle, speedVector);
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

void setConvertedSpeed(float speedX, float speedY, float speedAngle) {
    float convertedAngle = convertAngle(angle + ow.machineAngle);
    float convertedSpeedX = speedX * cosf(convertedAngle * (-1.f)) - speedY * sinf(convertedAngle * (-1.f));
    float convertedSpeedY = speedX * sinf(convertedAngle * (-1.f)) + speedY * cosf(convertedAngle * (-1.f));

    ow.setSpeed(convertedSpeedX, convertedSpeedY, speedAngle);
}

//angle1とangle2との差で絶対値が小さい値を返す。単位は[rad]
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

//絶対座標をもとに制御を行う
void moveTargetPoint(float pointX, float pointY, float pointAngle) {
    float convertedX = ow.positionX * cosf(angle) - ow.positionY * sinf(angle) + x;
    float convertedY = ow.positionX * sinf(angle) + ow.positionY * cosf(angle) + y;
    float convertedAngle = convertAngle(angle + ow.machineAngle);

    float tempX = pointX - convertedX;
    float tempY = pointY - convertedY;
    float tempAngle = getDifferenceAngle(pointAngle, convertedAngle);

    //tempAngle = tempAngle - static_cast<int>(tempAngle);
    //float tempAngle = pointAngle - convertedAngle;

    float speedX = tempX * cosf(convertedAngle * (-1.f)) - tempY * sinf(convertedAngle * (-1.f));
    float speedY = tempX * sinf(convertedAngle * (-1.f)) + tempY * cosf(convertedAngle * (-1.f));

    static float iX = 0, iY = 0, iAngle = 0;

    iX = speedX * 0.1 + iX * 0.9;
    iY = speedY * 0.1 + iY * 0.9;
    iAngle = tempAngle * 0.1 + iAngle * 0.9;

    float kp = 1, ki = 1;

    if (tempAngle * tempAngle > 0.000001) {
        ow.setSpeed(speedX, speedY, tempAngle * 0.01);
        //ow.setSpeed(kp * speedX + ki * iX, kp * speedY + ki * iY, kp * tempAngle + ki * iAngle);
    } else {
        ow.setSpeed(kp * speedX + ki * iX, kp * speedY + ki * iY, 0);
    }
}

void updatePurePursuitAlgorithm() {

    //const float targetSpeed = 400; //脱調しやすくなる
    const float targetSpeed = 0.4;
    //static float lastIndex = ROUTE_SIZE - 1;
    //static float time = 0;
    static int targetInd = ppa.calculateTargetIndex();

    updateState();

    float ai = ppa.PIDControl(targetSpeed, speedVector);
    float di = ppa.update(targetInd);
    targetInd = ppa.ind;

    float newSpeed = (speedVector + ai);

    if (targetInd != ROUTE_SIZE - 1) {
        float angleB = getDifferenceAngle(angleList[targetInd], getAngle());
        std::cout << "angleB " << angleB << " getAngle() " << getAngle() << " angleList " << angleList[targetInd] << std::endl;
        setConvertedSpeed(newSpeed * cosf(speedVectorAngle + di),
                          newSpeed * sinf(speedVectorAngle + di), angleB);
    } else {
        //最終地点に到達したときのみ 最終地点で停止する
        std::cout << "stop point" << std::endl;
        moveTargetPoint(ppa.cx[ROUTE_SIZE - 1], ppa.cy[ROUTE_SIZE - 1], angleList[ROUTE_SIZE - 1]);
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "machine_driver2");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //速度制御(cmd_vel)の準備

    //経路追従関連の初期化
    for (int i = 0; i < ROUTE_SIZE; ++i) {
        // if (i < 314 * 2) {
        //     ppa.cy[i] = static_cast<float>(i) / 2.f;
        //     ppa.cx[i] = sin(static_cast<float>(i) / 100.f) * 150;
        //     angleList[i] = 0;
        // } else {
        //     ppa.cy[i] = static_cast<float>(i) / 2.f;
        //     ppa.cx[i] = 0;
        //     angleList[i] = M_PI * 10 / 4.0 / (ROUTE_SIZE - 314 * 2) * (i - 314 * 2);
        //     //angleList[i] = 0;
        // }
        // ppa.cy[i] = sin(static_cast<float>(i) / 100.f) * 500;
        // ppa.cx[i] = cos(static_cast<float>(i) / 100.f * 2.f) * 500;
        // angleList[i] = 0;

        //砂時計経路
        ppa.cy[i] = sin(static_cast<float>(i) / 100.f) * 5;
        ppa.cx[i] = sin(static_cast<float>(i) / 100.f * 2.f) * 5;

        //往復経路
        // ppa.cy[i] = static_cast<float>(i % 100) * 5 - 5.f * 100.f / 2.f;
        // ppa.cx[i] = 0;

        angleList[i] = 0.01 * static_cast<float>(i);
        //angleList[i] = 0.1;
    }
    ppa.init(0, 0, 0, 0);

    ow.init(nh);

    int i = 0;
    while (ros::ok()) {
        ros::spinOnce();
        ow.update();
        updatePurePursuitAlgorithm();

        rate.sleep();
    }

    return 0;
}
