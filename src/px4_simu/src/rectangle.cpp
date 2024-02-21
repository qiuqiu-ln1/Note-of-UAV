#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//a simple callback which will save the current state of the autopilot
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg_state){
    current_state = *msg_state;
}
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_pos){
    current_pos = *msg_pos;
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"rectangle");
    ros::NodeHandle n;
    
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
        ("mavros/state",10,state_cb);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose",10,pos_cb);
    ros::Publisher  local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
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
    std::vector<std::tuple<double, double, double>> points = {{0, 0, 2}, {3, 0, 2}, {3, 4, 2}, {0, 4, 2}};

    // 定义矩形的索引
    int index = 0;

    while(ros::ok())
    {
                
        // 更新目标位置
        pose.pose.position.x = std::get<0>(points[index]);
        pose.pose.position.y = std::get<1>(points[index]);
        pose.pose.position.z = std::get<2>(points[index]);

        // 发布目标位置
        local_pos_pub.publish(pose);

        // 计算当前位置和目标位置的距离
        double distance =  std::sqrt(   std::pow(pose.pose.position.x - current_pos.pose.position.x, 2)
                                        + std::pow(pose.pose.position.y - current_pos.pose.position.y, 2)
                                        + std::pow(pose.pose.position.z - current_pos.pose.position.z, 2));

        if (distance < 0.1)
        {
            index = (index + 1) % 4;
        }


        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

