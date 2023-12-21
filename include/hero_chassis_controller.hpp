#pragma once  //避免头文件被包含多次

#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace hero_chassis_controller
{//这意味着 HeroChassisController 类会继承并使用 controller_interface::Controller 类中定义的成员，并且将 hardware_interface::EffortJointInterface 作为其操作的接口类型。
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
            {
            public:
                HeroChassisController();//构造函数和析构函数的声明
                ~HeroChassisController();
                bool init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle &root_nh, ros::NodeHandle &n) override;

                void HeroChassisController::starting(const ros::Time& time);
                void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

                void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)override;
            private:
                control_toolbox::Pid pid_front_left_;
                control_toolbox::Pid pid_front_right_;
                control_toolbox::Pid pid_back_left_;
                control_toolbox::Pid pid_back_right_;

                hardware_interface::JointHandle front_left_joint_;
                hardware_interface::JointHandle front_right_joint_;
                hardware_interface::JointHandle back_left_joint_;
                hardware_interface::JointHandle back_right_joint_;

                ros::Subscriber cmd_vel_sub_;
                ros::Publisher odom_pub_;
                //ros::Subscriber controller_state_publisher_;

                tf::TransformBroadcaster odom_broadcaster;

                std::unique_ptr<
                       realtime_tools::RealtimePublisher<
                                control_msgs::JointControllerState> > controller_state_publisher_ ;

                //void get_vel_cmd(const geometry_msgs::TwistConstPtr &msg);

                double x{0.0},y{0.0},th{0.0};
                double lx{0.0},ly{0.0},az{0.0};
                double vx{0.0},vy{0.0},vth{0.0};
                int loop_count_;
                double wheel_base_ {0.0};
                double wheel_track_ {0.0};
                double wheel_radius_ {0.0};
                double vel_now[4] {0.0,0.0,0.0,0.0};
                double vel_cmd[4] {0.0,0.0,0.0,0.0};
                ros::Time now;
                int error[4]{0,0,0,0} ;

                ros::Time current_time, last_time;

                /*void setCommandCB(const geometry_msgs::Twist::ConstPtr& msg);
                void compute_the_cmd();
                void compute_the_cur_vel();
                void transform_then_pub();
                void transform_vel();*/
                tf::TransformBroadcaster frame_broadcaster;
                tf::TransformListener frame_listener;
                geometry_msgs::Vector3Stamped vel_odom;
                geometry_msgs::Vector3Stamped vel_base;
                bool mode;
            };
}
#endif



