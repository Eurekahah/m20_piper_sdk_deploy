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

    /**
     * @brief 请求"本地输入源"（键盘）把内部状态归位到默认值。
     *
     * 场景：VR 的复位（左 X/Y）会把机身姿态/速度交还给键盘。这时键盘必须从**默认
     * 底座**重新开始 —— 否则它会在 5 ms 后用复位前的旧 `body_height_` 覆盖掉"回到
     * 默认"的指令，机身就永远回不到默认（2026-10-13 操作员反馈的现象）。
     * 外部源在复位边沿调用 `RequestLocalReset()`，键盘线程在下一拍消费。
     */
    void RequestLocalReset() {
        local_reset_request_.store(true);
    }

    bool ConsumeLocalResetRequest() {
        return local_reset_request_.exchange(false);
    }

    MotionStateFeedback *msfb_;
    RobotName robot_name_;
    UserCommand *usr_cmd_;
    std::atomic<bool> remote_active_{false};
    std::atomic<bool> local_reset_request_{false};
};
};
