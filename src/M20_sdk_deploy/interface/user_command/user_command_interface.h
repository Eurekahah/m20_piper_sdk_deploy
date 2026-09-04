/**
 * @file user_command_interface.h
 * @brief this file is used for robot's user command input
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include <atomic>
#include "common_types.h"
#include "custom_types.h"
#include "motion_state_feedback.hpp"

using namespace types;

namespace interface{

class UserCommandInterface{
private:
    /* data */
public:
    UserCommandInterface(RobotName robot_name){
        robot_name_ = robot_name;
        usr_cmd_ = new UserCommand();
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }
    virtual ~UserCommandInterface(){
        delete usr_cmd_;
    }

    /**
     * @brief start the thread to process user command
     */
    virtual void Start() = 0;

    /**
     * @brief stop the thread 
     */
    virtual void Stop() = 0;

    /**
     * @brief return your user command
     * @return UserCommand ptr
     */
    virtual UserCommand* GetUserCommand() = 0; 

    /**
     * @brief set the motion state feedback 
     * @param  msfb         motion state feedback
     */
    virtual void SetMotionStateFeedback(MotionStateFeedback* msfb){
        msfb_ = msfb;
    }

    /**
     * @brief Mark an external teleop source (VR) as active.
     *
     * While active, local input devices such as the keyboard must stop
     * overwriting velocity/body fields they do not own, otherwise the keyboard
     * loop would zero/step over VR commands at 200 Hz.
     */
    void SetRemoteActive(bool active) {
        remote_active_.store(active);
    }

    bool IsRemoteActive() const {
        return remote_active_.load();
    }

    MotionStateFeedback *msfb_;
    RobotName robot_name_;
    UserCommand *usr_cmd_;
    std::atomic<bool> remote_active_{false};
};
};
