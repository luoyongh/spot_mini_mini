#ifndef I2CController_HPP
#define I2CController_HPP

#define USE_TEENSY_HW_SERIAL

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3.h"
#include <mini_ros/ContactData.h>
#include <mini_ros/IMUdata.h>
#include <mini_ros/JointAngles.h>
#include <mini_ros/JointPulse.h>
#include "SpotServo/SpotServo.hpp"
//#include "IMU/IMU.hpp"
#include "SpotServo/i2cpwm_pca9685.h"

struct LegJoints
{
    LegType legtype;
    double shoulder = 0.0;
    double elbow = 0.0;
    double wrist = 0.0;

    LegJoints(const LegType & legtype_)
    {
        legtype = legtype_;
    }

    void FillLegJoint(const double & s, const double & e, const double & w)
    {
        shoulder = s;
        elbow = e;
        wrist = w;
    }
};


class I2CController
{
    // Joint Angle Subscriber
    ros::Subscriber ja_sub_;
    ros::Subscriber jp_sub_;
    // joint msg timer
    double prev_joints_time_;
    double prev_resetter_time_;
    // joint cmd msg flag
    bool joints_cmd_active_ = false;
    // joint pulse msg flag
    bool joints_pulse_active_ = false;
    int joint_num = -1;
    int pulse = 1250;
    // move type
    // False is step, True is view
    bool step_or_view = false;
    // joint msgs
    LegType fl_leg = FL;
    LegType fr_leg = FR;
    LegType bl_leg = BL;
    LegType br_leg = BR;
    LegJoints FL_ = LegJoints(fl_leg);
    LegJoints FR_ = LegJoints(fr_leg);
    LegJoints BL_ = LegJoints(bl_leg);
    LegJoints BR_ = LegJoints(br_leg);

    // IMU msg and publisher
    mini_ros::IMUdata imu_msg_;
    ros::Publisher imu_pub_;

    // Contact msg and publisher
    mini_ros::ContactData contact_msg_;
    ros::Publisher contact_pub_;

    void JointCommandCallback(const mini_ros::JointAngles& ja_msg)
    {
        prev_joints_time_ = micros();

        FL_.FillLegJoint(ja_msg.fls, ja_msg.fle, ja_msg.flw);
        FR_.FillLegJoint(ja_msg.frs, ja_msg.fre, ja_msg.frw);
        BL_.FillLegJoint(ja_msg.bls, ja_msg.ble, ja_msg.blw);
        BR_.FillLegJoint(ja_msg.brs, ja_msg.bre, ja_msg.brw);
        step_or_view = ja_msg.step_or_view;
        // flag
        joints_cmd_active_ = true;
    }

    void JointPulseCallback(const mini_ros::JointPulse& jp_msg)
    {
        joint_num = jp_msg.servo_num;
        pulse = jp_msg.servo_pulse;
        // flag
        joints_pulse_active_ = true;

        // disable joint cmd flag
        joints_cmd_active_ = false;
    }

    public:
        I2CController(){;}

        int initialize(ros::NodeHandle nh_)
        {
            ja_sub_ = nh_.subscribe("spot/joints", 1, &I2CController::JointCommandCallback, this);
            jp_sub_ = nh_.subscribe("spot/pulse", 1, &I2CController::JointPulseCallback, this);

            imu_pub_ = nh_.advertise<mini_ros::IMUdata>("spot/imu", 1);
            contact_pub_ = nh_.advertise<mini_ros::ContactData>("spot/contact", 1);

            int res = pca9685_init ("/dev/i2c-0");
            if (res != 0) {
                return res;
            }
            set_active_board (1);
            set_pwm_frequency (50);

            ROS_INFO("SPOT MINI MINI ROS CLIENT CONNECTED");
            return 0;
        }

        void returnJoints(LegJoints & FL_ref,  LegJoints & FR_ref, LegJoints & BL_ref, LegJoints & BR_ref, bool & step_or_view_)
        {
            FL_ref = FL_;
            FR_ref = FR_;
            BL_ref = BL_;
            BR_ref = BR_;
            step_or_view_ = step_or_view;
        }

        bool jointsInputIsActive()
        {
            return joints_cmd_active_;
        }

        bool jointsPulseIsActive()
        {
            return joints_pulse_active_;
        }

        int returnPulse()
        {
            return pulse;
        }

        int returnServoNum()
        {
            return joint_num;
        }

        void resetPulseTopic()
        {
            joints_pulse_active_ = false;
            pulse = 1250;
            joint_num = -1;
        }

        void run()
        {
            // timeout
            double now = micros();
            if((now - prev_joints_time_) > 1000000)
            {
                joints_cmd_active_ = false;
            }
            ros::spinOnce();
        }

        void publishContacts(const bool & FLC, const bool & FRC, const bool & BLC, const bool & BRC)
        {
            contact_msg_.FL = FLC;
            contact_msg_.FR = FRC;
            contact_msg_.BL = BLC;
            contact_msg_.BR = BRC;

            contact_pub_.publish(contact_msg_);
        }

        void publishIMU(geometry_msgs::Vector3 eul, geometry_msgs::Vector3 acc, geometry_msgs::Vector3 gyro)
        {
            // NOTE: switching eul.x and eul.x because Bosch is weird..
            imu_msg_.roll = eul.z;
            imu_msg_.pitch = eul.y;

            imu_msg_.acc_x = acc.x;
            imu_msg_.acc_y = acc.y;
            imu_msg_.acc_z = acc.z;

            imu_msg_.gyro_x = gyro.x;
            imu_msg_.gyro_y = gyro.y;
            imu_msg_.gyro_z = gyro.z;

            imu_pub_.publish(imu_msg_);
        }
};

#endif