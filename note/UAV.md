# 基础1：ROS

## 1 创建工作空间

```

mkdir <workspace_name>
cd <workspace_name>
mkdir src
cd src
catkin_init_workspace
cd ..
catkin_make
catkin_make install

source devel/setup.bash  #该语句定位到工程文件夹下devel文件夹
#如果是zsh下，则文件后缀为.zsh
echo $ROS_PACKAGE_PATH

#或者
#将上述setup.bash文件添加到.bashrc中,注意如果有多个工作空间建议使用前一个方法，此处只能将最后一行的工作空间添加入环境
#再执行
source ~/.bashrc
#若是zsh,则将setup.zsh添加到.zshrc中并执行source ~/.zshrc,zsh中好像暂时没有bash中的问题
```

## 2 创建功能包

```
cd ~/<workspace_name>/src
catkin_create_pkg <pkg_name> <#依赖：roscpp rospy std_msgs geometry_msgs mavros ...>

#编写相应cpp代码后
cd ~/<workspace_name>
catkin_make
或者
catkin build(有时需要先运行catkin clean)
```

## 3 基本概念

1. 节点（Node）：执行单元
   * 执行具体任务的进程
   * 不同节点可以使用不同编程语言，可以在不同主机运行
   * 节点在系统中名称必须唯一
2. 节点管理器(ROS Master)：控制中心
   * 为节点提供命名和注册服务
   * 跟踪记录通讯，帮助各个节点建立连接
   * 提供参数服务器，每个节点都使用此服务器的参数(类似于全局的词典)
3. 节点之间建立联系的通讯机制：Topic & Service
   * **Topic话题：** 发布(Publish)/订阅(Subscribe)模型，由发布者(Publisher)发布话题数据(消息——Message)再由订阅者（Subscireber）订阅，话题的发布者和订阅者可以不唯一。消息(Message)有标准的ROS类型，也可以通过 `.msg`文件定义
     ![Topic](/note/image/UAV/Topic.png)
   * **Service服务：** 使用客户端(Client)/服务器(Server)模型，客户端发送请求(Request)数据，服务器完成处理后返回应答(Reply)数据。请求和应答数据由 `.srv`文件定义
     ![Service](/note/image/UAV/Service.png)
   * **Topic和Service的应用场景：**
     * **topic适用于连续、高频的数据发布**：里程计、雷达、GPS等数据的发布和接收(类似于仪表盘和方向盘)；**Service适用于简单、低频、具体的任务/功能命令**：模式切换、相机拍照等具体功能(类似于开关、按键)
     * Topic发布数据后会直接进行之后的程序，而Service会等待服务器响应后再执行接下来的数据
     * Topic是**多对多**；Service是**单(Server)对多**
4. 参数(Parameter)：全局共享字典
   * 可通过网络访问
   * 节点使用此服务器来存储和检索参数
   * 只适用于静态的配置参数，而不是动态的参数和数据:当setParameter改变参数后，如果getParameter不重新get则其读到的参数仍为旧值。
     ![Parameter](/note/image/UAV/Parameter.png)
5. 元功能包&功能包：功能包是ros软件中的基本单元，包含节点源码、配置文件、数据类型定义等，如px4_simu中的offb_rectangle；元功能包是是一系列功能包的集合，即px4_simu

## 4 Topic

### Publisher发布者

#### CPP版本

步骤：

1. 初始化节点
2. 向ROS Master注册节点信息
3. 创建消息数据
4. 按照一定频率发布

```

int main(int argc, char **argv)
{
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle nh;

    ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);

    ros::Rate rate(10);

    int count = 0;

    geometry_msgs::Twist vel;
    vel.linear.x = 0.5;
    vel.angular.z = 0.2;

    while(ros::ok())
    {
        turtle_vel_pub.publish(vel);
        ROS_INFO("Publish turtle velocity [ %0.2f m/s, %0.2f rad/s]",vel.linear.x,vel.angular.z);
        rate.sleep();
    }
    return 0;

}
```

1. 该段代码中，前几句均为初始化ros节点

```
int main(int argc, char **argv)
{
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle nh;
    ...
}
```

2. 注册节点信息

 `ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>`

`其中：`

`("turtle1/cmd_vel",10);`为创建发布者Pubulisher节点，这里 `ros::Publihser`是固定的，`trutle_vel_pub`是自己定义的节点名称，`nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10)`中 `advertise`表示发布，`<..>`里的内容是消息类型的名称，`“turtle1/cmd_vel”`是话题Topic名称（这个不是随便给的，而是由turtlesim自带的，因为发布和订阅的话题名称需要一致，而turtlesim是从该名称里订阅海龟的移动速度消息的），`10`代表队列长度可自己设置

3. 创建消息数据
   ```
   geometry_msgs::Twist vel;
       vel.linear.x = 0.5;
       vel.angular.z = 0.2;
   ```
4. 按照一定频率发布

```
  while(ros::ok())
      {
          turtle_vel_pub.publish(vel);
          ROS_INFO("Publish turtle velocity [ %0.2f m/s, %0.2f rad/s]",vel. linear.x,vel.angular.z);
          rate.sleep();
      }
```

可以通过 `rosrun rqt_graph rqt_graph`查看节点关系图
![node_publish](/note/image/UAV/node_publisher.png)

---

#### Pyhotn版本

---

# 基础2：MAVROS

## 1 Simulation的基本框架

Simulator提供了一个由电脑生成的无人机及环境，模拟的无人机与真实的无人机一样可以通过PX4软件来进行控制，控制主要通过三种：

* QGC：设定plan；
* offboardAPI：板载计算机；
* radio controller：遥控。

仿真分为SITL和HITL：

1. SITL：是基于电脑创建虚拟的环境，由模拟器提供虚拟的环境数据和PX4
2. HITL：基于PX4固件的HITL模式，需要固件

SITL的通讯如下：
offboard和qgc通过mavlink命令控制SITL虚拟的PX4，个人推测gazebo（Simulator）也可以自己直接控制虚拟的PX4（可能是用于模拟遥控？）故有Simulator与PX4的通讯端口

![SITL](/note/image/UAV/SITL.png)
QGC、Offboard主要通过MavLink与PX4飞控通信(mavlink_main.cpp)，PX4也通过mavlink与模拟器通信(simulator_mavlink.cpp)

## 2 MAVROS的作用

板载计算机(运行ROS)、PX4、QGC、Gazebo(Simulator)的通信是通过mavlink，mavros是ros的一个功能包，其作用即运行在ros下的mavlink消息收发工具。可以理解为mavros即一个ros话题，消息类型为mavlink  
下图中的mavros对应上图的mavlink，QGC通过WIFI发布/订阅mavros可以视作上图的端口14550，minipc(下图中的记载计算机，对应上图的offboard)通过mavros
![基于mavros的minipc控制框架](/note/image/UAV/mavros_contorl.png)

## 3 MAVROS的运行

```
#先启动gazebo,提供虚拟无人机及环境
make px4_sitl gazebo
#再启动mavros,提供通讯，roslaunch会自动检测roscore是否启动并自动开启roscore如果未启动roscore的话
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
#最后启动基于mavros功能包编写的功能包节点
rosrun pkg_name node_name 
```
