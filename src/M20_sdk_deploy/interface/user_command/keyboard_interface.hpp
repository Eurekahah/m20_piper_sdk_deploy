// keyboard_interface.hpp
#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <cctype>
#include <mutex>
#include <vector>

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    std::atomic<bool> running_{false};
    std::thread kb_thread_;
    mutable std::mutex keys_mutex_;

    float max_forward_ = 0.7f;
    float max_side_    = 0.5f;
    float max_yaw_     = 0.7f;

    // Body pose targets (absolute; accumulated while the keys are held).
    // NOTE: 'c' is already used to enter RL control mode, so body height uses
    // h/j instead of the Isaac Lab C/V mapping.
    float body_height_ = 0.513f;   // m, matches the Isaac Lab default base height
    float body_pitch_  = 0.0f;     // rad
    float body_roll_   = 0.0f;     // rad
    const float height_step_ = 0.002f;   // m per repeat
    const float body_pose_step_ = 0.01f; // rad per repeat
    const float body_height_min_ = 0.33f, body_height_max_ = 0.60f;
    const float body_pitch_max_ = 0.35f, body_roll_max_ = 0.25f;

    // Arm EE increments (non-zero while the numpad key is held, NumLock ON)
    float ee_inc_[6] = {0, 0, 0, 0, 0, 0};  // dx, dy, dz, droll, dpitch, dyaw
    const float ee_pos_step_ = 0.005f;  // m
    const float ee_orn_step_ = 0.02f;   // rad
    bool gripper_closed_ = false;

    std::unordered_set<char> held_keys_;
    std::unordered_map<char, double> last_seen_time_;
    
    const std::unordered_set<char> velocity_keys_ = {'w', 's', 'a', 'd', 'q', 'e'};
    const std::unordered_set<char> body_keys_ = {'h', 'j', 'b', 'n', '[', ']'};
    const std::unordered_set<char> ee_keys_ = {'8', '2', '4', '6', '7', '9',
                                               '1', '3', '0', '.', '+', '-'};
    const double key_timeout_ms_ = 500.0;

    void ClipNumber(float& num, float low, float high)
    {
        if (num < low) num = low;
        if (num > high) num = high;
    }

    double GetCurrentTimeStamp()
    {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }

    static void setup_raw_mode()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        termios raw = t;
        raw.c_lflag &= ~(ECHO | ICANON);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    static void restore_terminal()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag |= (ECHO | ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    void compute_velocity_from_held_keys(float& fwd, float& side, float& yaw)
    {
        fwd = 0.0f;
        side = 0.0f;
        yaw = 0.0f;

        std::lock_guard<std::mutex> lock(keys_mutex_);
        
        if (held_keys_.count('w')) fwd += max_forward_;
        if (held_keys_.count('s')) fwd -= max_forward_;
        if (held_keys_.count('a')) side += max_side_;
        if (held_keys_.count('d')) side -= max_side_;
        if (held_keys_.count('q')) yaw += max_yaw_;
        if (held_keys_.count('e')) yaw -= max_yaw_;
        
        ClipNumber(fwd, -max_forward_, max_forward_);
        ClipNumber(side, -max_side_, max_side_);
        ClipNumber(yaw, -max_yaw_, max_yaw_);
    }

    void process_mode_command(char k)
    {
        if (k == 'r') {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            std::cout << "[MODE] Joint Damping\n";
        }
        else if (k == 'z' && (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand
            || msfb_->GetCurrentState() == RobotMotionState::LieDown)) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
            std::cout << "[MODE] Standing Up\n";
        }
        else if (k == 'c' && msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
            std::cout << "[MODE] RL Control\n";
        }
        else if (k == 'x' && (msfb_->GetCurrentState() == RobotMotionState::StandingUp 
            || msfb_->GetCurrentState() == RobotMotionState::RLControlMode)) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
            std::cout << "[MODE] Lie Down\n";
        }
    }

    void clip_body_pose()
    {
        if (body_height_ < body_height_min_) body_height_ = body_height_min_;
        if (body_height_ > body_height_max_) body_height_ = body_height_max_;
        if (body_pitch_ < -body_pitch_max_) body_pitch_ = -body_pitch_max_;
        if (body_pitch_ > body_pitch_max_)  body_pitch_ =  body_pitch_max_;
        if (body_roll_ < -body_roll_max_) body_roll_ = -body_roll_max_;
        if (body_roll_ > body_roll_max_)  body_roll_ =  body_roll_max_;
    }

    void handle_body_key(char k)
    {
        if (k == 'h')      body_height_ += height_step_;
        else if (k == 'j') body_height_ -= height_step_;
        else if (k == 'b') body_pitch_  += body_pose_step_;
        else if (k == 'n') body_pitch_  -= body_pose_step_;
        else if (k == '[') body_roll_   += body_pose_step_;
        else if (k == ']') body_roll_   -= body_pose_step_;
        clip_body_pose();
    }

    void handle_action_key(char k)
    {
        if (k == 'g') {
            gripper_closed_ = !gripper_closed_;
            usr_cmd_->gripper_cmd = gripper_closed_ ? 1.0f : 0.0f;
            std::cout << "[GRIPPER] " << (gripper_closed_ ? "CLOSE" : "OPEN") << "\n";
        } else if (k == 'l') {
            body_height_ = 0.513f;
            body_pitch_  = 0.0f;
            body_roll_   = 0.0f;
            gripper_closed_ = false;
            usr_cmd_->gripper_cmd = 0.0f;
            usr_cmd_->ee_reset = 1;
            std::cout << "[RESET] body pose + arm teleop reset\n";
        }
    }

    void keyboard_loop()
    {
        setup_raw_mode();

        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║      KEYBOARD TELEOP - MULTI-KEY READY         ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Movement:  W/S (forward/back)  A/D (left/right)\n"
                  << "  Rotation:  Q (CCW)  E (CW)\n"
                  << "  Mode:      R (damping)  Z (stand)  C (control)\n"
                  << "  Body pose: H/J (height)  B/N (pitch)  [/] (roll)\n"
                  << "  Arm EE (numpad, NumLock ON):\n"
                  << "    8/2 x, 4/6 y, 7/9 z, 1/3 roll, 0/. pitch, +/- yaw\n"
                  << "  G (gripper toggle)  L (reset body+arm)\n"
                  << "\n";

        char ch;

        while (running_) {
            double now = GetCurrentTimeStamp();
            usr_cmd_->time_stamp = now;

            // Read all available keyboard input
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                char k = std::tolower(static_cast<unsigned char>(ch));

                // Handle mode commands
                if (k == 'r' || k == 'z' || k == 'c' || k == 'x') {
                    process_mode_command(k);
                    continue;
                }

                // Body pose keys (increment while held / repeated)
                if (body_keys_.count(k)) {
                    handle_body_key(k);
                    continue;
                }

                // Action keys
                if (k == 'g' || k == 'l') {
                    handle_action_key(k);
                    continue;
                }

                // Track velocity + EE keys (held-set based, timeout on release)
                if (velocity_keys_.count(k) || ee_keys_.count(k)) {
                    std::lock_guard<std::mutex> lock(keys_mutex_);
                    held_keys_.insert(k);
                    last_seen_time_[k] = now;
                }
            }

            // Remove keys that haven't been seen recently (released)
            {
                std::lock_guard<std::mutex> lock(keys_mutex_);
                std::vector<char> to_remove;
                
                for (char k : held_keys_) {
                    if (now - last_seen_time_[k] > key_timeout_ms_) {
                        to_remove.push_back(k);
                    }
                }
                
                for (char k : to_remove) {
                    held_keys_.erase(k);
                    last_seen_time_.erase(k);
                }
            }

            // Compute velocity from all currently held keys
            float fwd = 0.0f, side = 0.0f, yaw = 0.0f;
            
            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                compute_velocity_from_held_keys(fwd, side, yaw);
            }

            // Arm EE increments from the held numpad keys
            for (int i = 0; i < 6; ++i) ee_inc_[i] = 0;
            {
                std::lock_guard<std::mutex> lock(keys_mutex_);
                if (held_keys_.count('8')) ee_inc_[0] += ee_pos_step_;
                if (held_keys_.count('2')) ee_inc_[0] -= ee_pos_step_;
                if (held_keys_.count('4')) ee_inc_[1] += ee_pos_step_;
                if (held_keys_.count('6')) ee_inc_[1] -= ee_pos_step_;
                if (held_keys_.count('7')) ee_inc_[2] += ee_pos_step_;
                if (held_keys_.count('9')) ee_inc_[2] -= ee_pos_step_;
                if (held_keys_.count('1')) ee_inc_[3] += ee_orn_step_;
                if (held_keys_.count('3')) ee_inc_[3] -= ee_orn_step_;
                if (held_keys_.count('0')) ee_inc_[4] += ee_orn_step_;
                if (held_keys_.count('.')) ee_inc_[4] -= ee_orn_step_;
                if (held_keys_.count('+')) ee_inc_[5] += ee_orn_step_;
                if (held_keys_.count('-')) ee_inc_[5] -= ee_orn_step_;
            }
            
            usr_cmd_->forward_vel_scale  = fwd;
            usr_cmd_->side_vel_scale     = side;
            usr_cmd_->turnning_vel_scale = yaw;
            usr_cmd_->body_height = body_height_;
            usr_cmd_->body_pitch  = body_pitch_;
            usr_cmd_->body_roll   = body_roll_;
            for (int i = 0; i < 6; ++i) usr_cmd_->ee_inc[i] = ee_inc_[i];

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        restore_terminal();
        std::cout << "\n[KEYBOARD] Stopped.\n";
    }

public:
    KeyboardInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
        std::cout << "[KeyboardInterface] Initialized with multi-key support\n";
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
        usr_cmd_->body_height = body_height_;
        usr_cmd_->gripper_cmd = 0.0f;
    }

    ~KeyboardInterface() 
    { 
        Stop(); 
    }

    void Start() override
    {
        if (running_) return;
        running_ = true;
        kb_thread_ = std::thread(&KeyboardInterface::keyboard_loop, this);
    }

    void Stop() override
    {
        running_ = false;
        if (kb_thread_.joinable()) {
            kb_thread_.join();
        }
        
        std::lock_guard<std::mutex> lock(keys_mutex_);
        held_keys_.clear();
        last_seen_time_.clear();
        
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override 
    { 
        return usr_cmd_; 
    }

    void set_max_velocities(float fwd, float side, float yaw)
    {
        max_forward_ = std::abs(fwd);
        max_side_    = std::abs(side);
        max_yaw_     = std::abs(yaw);
        std::cout << "[CONFIG] Max velocities: fwd=" << max_forward_ 
                  << " side=" << max_side_ 
                  << " yaw=" << max_yaw_ << "\n";
    }
};
