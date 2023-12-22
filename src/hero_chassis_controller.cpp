#include <ros/ros.h>
#include <hero_chassis_controller.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
    HeroChassisController::HeroChassisController()
    : loop_count_(0)
    {}
    HeroChassisController::~HeroChassisController()
    {
        cmd_vel_sub_.shutdown();
        odom_pub_.shutdown();
    }
//通过effort_joint_interface->getHandle("...")这样的用法，对传入的effort_joint_interface对象指针进行操作，调用了getHandle函数并传入了关节名称作为参数。
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh , ros::NodeHandle &controller_nh)
    {//这是用来从EffortJointInterface对象中获取特定名称关节的控制句柄，然后将这些控制句柄赋值给了类成员变量front_left_joint_、front_right_joint_、back_left_joint_、back_right_joint_。
        std::string front_left_wheel_joint,front_right_wheel_joint,back_left_wheel_joint,back_right_wheel_joint;
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        //初始化4个pid控制器
        if (!pid_front_left_.init(ros::NodeHandle(controller_nh, "left_front_wheel/pid")))
        {
            ROS_ERROR("NO wheel_PID_param was given");
            return false;
        }
        if (!pid_front_right_.init(ros::NodeHandle(controller_nh, "right_front_wheel/pid")))
        {
            ROS_ERROR("NO wheel_PID_param was given");
            return false;
        }
        if (!pid_back_left_.init(ros::NodeHandle(controller_nh, "left_back_wheel/pid")))
        {
            ROS_ERROR("NO wheel_PID_param was given");
            return false;
        }
        if (!pid_back_right_.init(ros::NodeHandle(controller_nh, "right_back_wheel/pid")))
        {
            ROS_ERROR("NO wheel_PID_param was given");
            return false;
        }

        //读取参数文件中的轮距参数
        if (!controller_nh.getParam("wheel_param/wheel_base",wheel_base_))
        {
            ROS_ERROR("NO wheel_base was given");
            return false;
        }
        if (!controller_nh.getParam("wheel_param/wheel_track",wheel_track_))
        {
            ROS_ERROR("NO wheel_track was given");
            return false;
        }
        if (!controller_nh.getParam("wheel_param/radius",wheel_radius_))
        {
            ROS_ERROR("NO wheel_radius was given");
            return false;
        }


        // Start realtime state publisher
        controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(controller_nh, "state", 1));
        //创建一个/cmd_vel的订阅器
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &HeroChassisController::setCommandCB, this);
        /*// Create a publisher for PID parameters
        pid_param_pub = nh.advertise<std_msgs::Float64>("/pid", 10);*/
        //创建odom的订阅器，发布nav_msgs::Odometry类型消息到/odom话题
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 1);
        return true;


//        void HeroChassisController::starting(const ros::Time& time) {
            //初始化速度变量
//            cmd_vel_.linear.x = 0.0;
//            cmd_vel_.linear.y = 0.0;
//            cmd_vel_.linear.z = 0.0;
//            cmd_vel_.angular.x = 0.0;
//            cmd_vel_.angular.y = 0.0;
//            cmd_vel_.angular.z = 0.0;

            //初始化pid控制器的参数值
            pid_front_left_.reset();
            pid_front_right_.reset();
            pid_back_left_.reset();
            pid_back_right_.reset();
//        }
            //使用setCommand函数让四个关节的控制命令初始化
            //front_left_joint_.setCommand(0);
            //front_right_joint_.setCommand(0);
            //back_left_joint_.setCommand(0);
            //back_right_joint_.setCommand(0);

    }
        void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
            now = time;
            vel_now[0] = front_left_joint_.getVelocity();//实际值
            vel_now[1] = front_right_joint_.getVelocity();
            vel_now[2] = back_left_joint_.getVelocity();
            vel_now[3] = back_right_joint_.getVelocity();

            /*if(mode)
            {
                transform_vel()
            }*/

            //transform_then_pub();
            //compute_the_cmd();
            for (int i = 1; i < 5; i++) {
                error[i] = vel_cmd[i] - vel_now[i];//差值=期望速度值-实际值
            }

            //pid控制器计算差值并进行调整
            double commanded_effort_front_left = pid_front_left_.computeCommand(error[0], period);
            double commanded_effort_front_right = pid_front_right_.computeCommand(error[1], period);
            double commanded_effort_back_left = pid_back_left_.computeCommand(error[2], period);
            double commanded_effort_back_right = pid_back_right_.computeCommand(error[3], period);

            //将经过 PID 控制器计算得出的控制力或速度值 commanded_effort... 设置为关节的命令值。
            front_left_joint_.setCommand(commanded_effort_front_left);
            front_right_joint_.setCommand(commanded_effort_front_right);
            back_left_joint_.setCommand(commanded_effort_back_left);
            back_right_joint_.setCommand(commanded_effort_back_right);

            if (loop_count_ % 10 == 0) {
                if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
                    controller_state_publisher_->msg_.header.stamp = now;
                    controller_state_publisher_->msg_.set_point = commanded_effort_front_left;
                    controller_state_publisher_->msg_.process_value = front_left_joint_.getVelocity();
                    controller_state_publisher_->msg_.error = error[3];
                    controller_state_publisher_->msg_.command = commanded_effort_front_left;

                    double dummy;
                    bool antiwindup;
                    getGains(controller_state_publisher_->msg_.p,
                             controller_state_publisher_->msg_.i,
                             controller_state_publisher_->msg_.d,
                             controller_state_publisher_->msg_.i_clamp,
                             dummy,
                             antiwindup);
                    controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                    controller_state_publisher_->unlockAndPublish();

                }
            }
            loop_count_++;



            //以下是使用正运动学实现里程计（选做第6项）
            //假设机器人从最初的"odom"坐标系原点开始
            double ox = 0.0;
            double oy = 0.0;
            double oth = 0.0;
            //设置一些速度，使“base_link”坐标系在“odom”坐标系中以 x 方向 0.1m/s、y 方向 -0.1m/s 和 0.1rad/s 的速率朝着th方向移动
            //void HeroChassisController::compute_the_cur_vel()
            //{//根据论文中的正运动学计算公式
            vx = (vel_now[0] + vel_now[1] + vel_now[2] + vel_now[3]) * wheel_radius_ / 4;
            vy = (-vel_now[0] + vel_now[1] + vel_now[2] - vel_now[3]) * wheel_radius_ / 4;
            vth = (-vel_now[0] + vel_now[1] - vel_now[2] + vel_now[3]) * wheel_radius_ / 4l;
            //}
            //根据机器人的速度计算里程计
            current_time = ros::Time::now();
            last_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();//计算当前时间 current_time 与上一个时间 last_time 之间的时间差 dt
            //根据机器人底盘的线速度 vx、垂直速度 vy 和角速度 vth 以及时间差 dt，计算机器人在 x、y 和角度上的变化量：
            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;//机器人在x方向上的位移变化
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;//机器人在y方向上的位移变化
            double delta_th = vth * dt;//机器人绕z轴的角度变化

            //这些位姿变化叠加到机器人的当前位置和姿态参数上
            ox += delta_x;
            oy += delta_y;
            oth += delta_th;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(oth);

            //通过tf发布转换
            geometry_msgs::TransformStamped odom_trans;//创建一条TransformStamped消息
            odom_trans.header.stamp = current_time;//通过 tf.我们想要发布“odom”的变换帧到“base_link”帧位于 current_time。
            odom_trans.header.frame_id = "odom";//这里指要被变换的父坐标系为odom
            odom_trans.child_frame_id = "base_link";//这里指的变换为base_link
            //根据里程计数据填充转换消息，然后使用我们的 TransformBroadcaster 发送转换。
            odom_trans.transform.translation.x = ox;//这里设置了变换消息的平移部分在x轴上的值为x，表示相对于参考坐标系的x轴方向的平移距离。
            odom_trans.transform.translation.y = oy;//这里设置了变换消息的平移部分在y轴上的值为y，表示相对于参考坐标系的y轴方向的平移距离。
            odom_trans.transform.translation.z = oth;//这里设置了变换消息的平移部分在z轴上的值为0.0，表示相对于参考坐标系的z轴方向上的平移距离。在这里将其设置为0.0，表示没有z轴方向上的平移。
            odom_trans.transform.rotation = odom_quat;//设置了变换消息的旋转部分，使用了之前定义的odom_quat，表示相对于参考坐标系的旋转姿态。
            //发送转换，设置好的变换消息通过odom_broadcaster对象进行发布。这个发布者将根据消息中指定的时间戳和坐标系信息发布坐标系变换消息。
            odom_broadcaster.sendTransform(odom_trans);

            //我们还需要发布nav_msgs/Odometry消息，以便导航堆栈可以从中获取速度信息。我们将消息的标头设置为 current_time 并将“odom”设置为坐标系
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            //这会用里程计数据填充消息并通过线路发送出去。我们将消息的 child_frame_id 设置为“base_link”帧，因为那是我们发送速度信息的坐标系。
            //set the position
            odom.pose.pose.position.x = ox;
            odom.pose.pose.position.y = oy;
            odom.pose.pose.position.z = oth;
            odom.pose.pose.orientation = odom_quat;
            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;
            //publish the message
            odom_pub_.publish(odom);
            last_time = current_time;
            //r.sleep();
        }

    ros::Rate r(1.0);//让程序在这里暂停一段时间，这样可以控制数据的发布频率减轻系统负载

    //7. 使用tf计算实现世界坐标下的速度控制
    void HeroChassisController::setCommandAB()
    {
        vel_odom.header.stamp = ros::Time(now);
        vel_odom.header.frame_id = "odom";
        vel_odom.vector.x = lx;
        vel_odom.vector.y = ly;
        vel_odom.vector.z = az;

        frame_listener.waitForTransform("base_link","odom",ros::Time(0.0),ros::Duration(1.0));
        frame_listener.transformVector("base_link",vel_odom,vel_base);

        lx = vel_base.vector.x;
        ly = vel_base.vector.y;
        az = vel_base.vector.z;
    }
    void HeroChassisController::setCommandCB(
            const geometry_msgs::Twist::ConstPtr &msg ,const ros::Duration& period){  //逆运动学是根据机器人的位姿来计算对应部分的运行状态的，这个机器人运动总结来说就是三种（x方向的位移，y方向的位移和绕z轴的旋转），所以就用到这三个数据
            lx = msg->linear.x;//获取 Twist 消息中的 x 方向的线速度信息
            ly = msg->linear.y;//获取 Twist 消息中的 y 方向的线速度信息
            az = msg->angular.z;//获取 Twist 消息中以 z 轴为转轴的角速度信息


            double l = (wheel_base_ + wheel_track_) / 2;
            //void HeroChassisController::compute_the_cmd()
            //{
            //描述差分驱动式机器人轮子的角速度与机器人底盘线速度之间的关系
            vel_cmd[0] = (lx - ly - l * az) * wheel_radius_;
            vel_cmd[1] = (lx + ly + l * az) * wheel_radius_;
            vel_cmd[2] = (lx + ly - l * az) * wheel_radius_;
            vel_cmd[3] = (lx - ly + l * az) * wheel_radius_;


            //计算每个车轮的PID控制(控制器根据PID传入的参数进行调节)
            double front_left_effort = pid_front_left_.computeCommand(vel_cmd[0], period);
            double front_right_effort = pid_front_right_.computeCommand(vel_cmd[1], period);
            double back_left_effort = pid_back_left_.computeCommand(vel_cmd[2], period);
            double back_right_effort = pid_back_right_.computeCommand(vel_cmd[3], period);

            //应用到每个轮子中
            front_left_joint_.setCommand(front_left_effort);
            front_right_joint_.setCommand(front_right_effort);
            back_left_joint_.setCommand(back_left_effort);
            back_right_joint_.setCommand(back_right_effort);
    }
 }//namespace
    PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)