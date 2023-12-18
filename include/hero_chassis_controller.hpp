#pragma once  //避免头文件被包含多次

#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller
{//这意味着 HeroChassisController 类会继承并使用 controller_interface::Controller 类中定义的成员，并且将 hardware_interface::EffortJointInterface 作为其操作的接口类型。
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
            {
            public:
                HeroChassisController();//构造函数和析构函数的声明
                ~HeroChassisController();
                bool init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle &root_nh, ros::NodeHandle &n) override;

                void JointVelocityController::setCommand(double cmd)
                void JointVelocityController::getCommand(double& cmd)

                void HerChassisController::starting(const ros::Time& time)
                void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
            private:
                control_toolbox::Pid pid_front_left_;
                control_toolbox::pid pid_front_right_;
                control_toolbox::pid pid_back_left_;
                control_toolbox::pid pid_back_right_;

                hardware_interface::EffortJointInterface front_left_joint_;
                hardware_interface::EffortJointInterface front_right_joint_;
                hardware_interface::EffortJointInterface back_left_joint_;
                hardware_interface::EffortJointInterface back_right_joint_;

                ros::Subscriber cmd_vel_sub_;

                tf::TransformBroadcaster odom_broadcaster;

                void get_vel_cmd(const geometry_msgs::TwistConstPtr &msg);

                double x{0.0},y{0.0},th{0.0};

                ros::Time current_time, last_time;

                void setCommandCB(const geometry_msgs::Twist::ConstPtr& msg);
                void compute_the_cmd();
                void compute_the_cur_vel();
                void transform_then_pub();
                void transform_vel();
                tf::TransformBroadcaster frame_broadcaster;
                tf::TransformListener frame_listener;
                geometry_msgs::Vector3Stamped vel_odom;
                geometry_msgs::Vector3Stamped vel_base;
                bool mode;
            };
}
#endif


