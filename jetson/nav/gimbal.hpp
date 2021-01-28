#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "rover_msgs/ZedGimbalCmd.hpp"
#include "rover_msgs/ZedGimbalPosition.hpp"
#include "pid.hpp"
#include "rapidjson/document.h"

#include <lcm/lcm-cpp.hpp>


class Gimbal{
    private:
        double cur_yaw;
        double target_yaw;
        double yaw_command;
        double MAX_YAW, MIN_YAW;
        rover_msgs::ZedGimbalPosition signal;
        double TOLERANCE;
        

    public:
        Gimbal(double MIN_YAW_IN, double MAX_YAW_IN, double tolerance_in);
        //sets the target yaw of the gimbal
        bool setTargetYaw(double target);

        //returns the current yaw of the gimbal
        double getYaw() const;

        void setYaw(double yaw);

        //returns the LCM message to be sent. Takes in a reference to the LCM object that sends it
        void publishControlSignal(lcm::LCM &lcmObj, const rapidjson::Document& mRoverConfig);

};

#endif // gimbal.hpp