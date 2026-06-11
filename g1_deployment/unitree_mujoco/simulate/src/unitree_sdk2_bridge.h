#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <iostream>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    // Update model and data pointers (e.g., after model reload)
    void setPointerReferences(mjModel** model_ptr, mjData** data_ptr) {
        mj_model_ = *model_ptr;
        mj_data_ = *data_ptr;
        _check_sensor();  // Re-initialize sensor addresses with new model
    }

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    // Foot contact sensors
    int left_foot_contact_adr_ = -1;
    int right_foot_contact_adr_ = -1;

    // Foot frame sensors
    int left_foot_bottom_linvel_adr_ = -1;
    int right_foot_bottom_linvel_adr_ = -1;

    // Contact debounce counters: hold contact state for a few ticks to
    // prevent transient false negatives from physics/bridge timing jitter.
    static constexpr int CONTACT_HOLD_TICKS = 5;  // 5ms at 1000Hz bridge rate
    int left_contact_hold_ = 0;
    int right_contact_hold_ = 0;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Foot contact sensors
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "left_foot_contact");
        if (sensor_id >= 0) {
            left_foot_contact_adr_ = mj_model_->sensor_adr[sensor_id];
            std::cout << "Found left_foot_contact sensor: sensor_id=" << sensor_id 
                      << ", adr=" << left_foot_contact_adr_ << std::endl;
        } else {
            std::cout << "WARNING: left_foot_contact sensor not found!" << std::endl;
        }
        
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "right_foot_contact");
        if (sensor_id >= 0) {
            right_foot_contact_adr_ = mj_model_->sensor_adr[sensor_id];
            std::cout << "Found right_foot_contact sensor: sensor_id=" << sensor_id 
                      << ", adr=" << right_foot_contact_adr_ << std::endl;
        } else {
            std::cout << "WARNING: right_foot_contact sensor not found!" << std::endl;
        }

        // Foot frame sensors
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "left_foot_bottom_linvel");
        if (sensor_id >= 0) {
            left_foot_bottom_linvel_adr_ = mj_model_->sensor_adr[sensor_id];
            std::cout << "Found left_foot_bottom_linvel sensor: sensor_id=" << sensor_id 
                      << ", adr=" << left_foot_bottom_linvel_adr_ << std::endl;
        } else {
            std::cout << "WARNING: left_foot_bottom_linvel sensor not found!" << std::endl;
        }

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "right_foot_bottom_linvel");
        if (sensor_id >= 0) {
            right_foot_bottom_linvel_adr_ = mj_model_->sensor_adr[sensor_id];
            std::cout << "Found right_foot_bottom_linvel sensor: sensor_id=" << sensor_id 
                      << ", adr=" << right_foot_bottom_linvel_adr_ << std::endl;
        } else {
            std::cout << "WARNING: right_foot_bottom_linvel sensor not found!" << std::endl;
        }
    }


};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }

        // Lock physics mutex to ensure mj_data_ is not mid-step
        {
            std::lock_guard<std::mutex> phys_lock(param::physics_mutex);

            // lowcmd
            {
                std::lock_guard<std::mutex> lock(lowcmd->mutex_);
                for(int i(0); i<num_motor_; i++) {
                    auto & m = lowcmd->msg_.motor_cmd()[i];
                    mj_data_->ctrl[i] = m.tau() +
                                        m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                        m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
                }
            }

            // lowstate
            if(lowstate->trylock()) {
                for(int i(0); i<num_motor_; i++) {
                    lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                    lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                    lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
                }
                
                if(imu_quat_adr_ >= 0) {
                    lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                    lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                    lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                    lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                    double w = lowstate->msg_.imu_state().quaternion()[0];
                    double x = lowstate->msg_.imu_state().quaternion()[1];
                    double y = lowstate->msg_.imu_state().quaternion()[2];
                    double z = lowstate->msg_.imu_state().quaternion()[3];

                    lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                    lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                    lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
                }
                
                if(imu_gyro_adr_ >= 0) {
                    lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                    lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                    lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
                }

                if(imu_acc_adr_ >= 0) {
                    lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                    lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                    lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
                }
                
                lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
                lowstate->unlockAndPublish();
            }
            // highstate
            if(highstate->trylock()) {
                if(frame_pos_adr_ >= 0) {
                    highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                    highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                    highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
                }
                if(frame_vel_adr_ >= 0) {
                    highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                    highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                    highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
                }
                
                // Contact detection with debounce to prevent transient false negatives.
                // When contact is detected, hold it for CONTACT_HOLD_TICKS to smooth out
                // any remaining single-frame glitches.
                // NOTE: Publishing binary contact state (0/1) in foot_force field
                // foot_force()[0] = left foot contact state (0=no contact, 1=contact)
                // foot_force()[1] = right foot contact state (0=no contact, 1=contact)
                if(left_foot_contact_adr_ >= 0) {
                    if(mj_data_->sensordata[left_foot_contact_adr_] > 0) {
                        left_contact_hold_ = CONTACT_HOLD_TICKS;
                    } else if(left_contact_hold_ > 0) {
                        left_contact_hold_--;
                    }
                    highstate->msg_.foot_force()[0] = static_cast<int16_t>(left_contact_hold_ > 0 ? 1 : 0);
                }
                
                if(right_foot_contact_adr_ >= 0) {
                    if(mj_data_->sensordata[right_foot_contact_adr_] > 0) {
                        right_contact_hold_ = CONTACT_HOLD_TICKS;
                    } else if(right_contact_hold_ > 0) {
                        right_contact_hold_--;
                    }
                    highstate->msg_.foot_force()[1] = static_cast<int16_t>(right_contact_hold_ > 0 ? 1 : 0);
                }

                // Foot frame velocities
                // NOTE: foot_speed_body is semantically body-frame, but publishing world-frame here
                // foot_speed_body[0-2] = left foot linear velocity in world frame (vx, vy, vz)
                if(left_foot_bottom_linvel_adr_ >= 0) {
                    highstate->msg_.foot_speed_body()[0] = mj_data_->sensordata[left_foot_bottom_linvel_adr_ + 0];
                    highstate->msg_.foot_speed_body()[1] = mj_data_->sensordata[left_foot_bottom_linvel_adr_ + 1];
                    highstate->msg_.foot_speed_body()[2] = mj_data_->sensordata[left_foot_bottom_linvel_adr_ + 2];
                }

                if (right_foot_bottom_linvel_adr_ >= 0) {
                    highstate->msg_.foot_speed_body()[3] = mj_data_->sensordata[right_foot_bottom_linvel_adr_ + 0];
                    highstate->msg_.foot_speed_body()[4] = mj_data_->sensordata[right_foot_bottom_linvel_adr_ + 1];
                    highstate->msg_.foot_speed_body()[5] = mj_data_->sensordata[right_foot_bottom_linvel_adr_ + 2];
                }
                
                highstate->unlockAndPublish();
            }
        } // physics mutex released

        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
