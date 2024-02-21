#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 定义全局变量
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// 回调函数，更新当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 回调函数，更新当前位置
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

// // 回调函数，更新当前速度
// void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
// {
//     current_velocity = *msg;
// }

// 主函数
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    // 订阅无人机状态、位置和速度话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    // ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, velocity_cb);

    // 发布期望位置话题
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 服务客户端，用于解锁和上锁电机，以及设置飞行模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置循环频率
    ros::Rate rate(20.0);

    // 等待连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 定义目标位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 发送目标位置，直到切换到OFFBOARD模式
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 尝试切换到OFFBOARD模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    bool response = set_mode_client.call(offb_set_mode);
    if (response && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("OFFBOARD_Success");
    }
    else
    {
        ROS_ERROR("OFFBOARD_fail");
    }

    // 尝试解锁电机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    response = arming_client.call(arm_cmd);
    if (response && arm_cmd.response.success)
    {
        ROS_INFO("MOTO_Success");
    }
    else
    {
        ROS_ERROR("MOTO_fail");
    }

    // 定义矩形的四个顶点
    std::vector<std::tuple<double, double, double>> points = {{0, 0, 2}, {5, 0, 2}, {5, 5, 2}, {0, 5, 2}};

    // 定义矩形的索引
    int index = 0;

    // 主循环
    while (ros::ok())
    {
        // 更新目标位置
        pose.pose.position.x = std::get<0>(points[index]);
        pose.pose.position.y = std::get<1>(points[index]);
        pose.pose.position.z = std::get<2>(points[index]);

        // 发布目标位置
        local_pos_pub.publish(pose);

        // 计算当前位置和目标位置的距离
        double distance = std::sqrt(std::pow(pose.pose.position.x - current_pose.pose.position.x, 2) + std::pow(pose.pose.position.y - current_pose.pose.position.y, 2) + std::pow(pose.pose.position.z - current_pose.pose.position.z, 2));

        // 如果距离小于一定阈值，切换到下一个顶点
        if (distance < 0.1)
        {
            index = (index + 1) % 4;
        }

        // 休眠一段时间
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}