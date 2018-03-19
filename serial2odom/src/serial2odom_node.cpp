#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <iostream>

using namespace std;
serial::Serial ser;
struct Odom_data{
    double x;
    double y;
    double th;
    double vx;
    double vy;
    double vth;

};

void pub_odom(Odom_data odom_data,tf::TransformBroadcaster odom_broadcaster,ros::Publisher odom_pub);
struct Odom_data get_odom(std_msgs::String result);

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

template <class Type>
Type stringToNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial2odom_node");
    ros::NodeHandle n;

    ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = n.advertise<std_msgs::String>("read", 1000);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    struct Odom_data odom_data;
    try
    {
        ser.setPort("/dev/ttyUSB0");//ttyACM0 pts/23
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            odom_data=get_odom(result);
            cout<<odom_data.x<<endl;
            pub_odom(odom_data,odom_broadcaster,odom_pub);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

struct Odom_data get_odom(std_msgs::String result)//这部分根据通信协议修改，这里只是测试
{
    struct Odom_data odom_data;
    unsigned char buf[13];
    int length;
    length = result.data.copy(buf, 12);
    buf[length] = '\0';
    if(buf[0]==0x1D)
    {
        odom_data.x=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
        odom_data.y=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
        odom_data.th=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
        odom_data.vx=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
        odom_data.vy=buf[5]+buf[6]*256+buf[7]*256*256+buf[8]*256*256*256;
        odom_data.vth=buf[9]+buf[10]*256+buf[11]*256*256+buf[12]*256*256*256;
    }
    else
    {
        ROS_INFO_STREAM("Wrong data format!");
    }
    return odom_data;
}
/*如果只给dx dy dth,需要订阅slam定位的结果，作为x y th的初始值，然后结合dx dy dth去计算新的x y th
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
*/

void pub_odom(Odom_data odom_data,tf::TransformBroadcaster odom_broadcaster,ros::Publisher odom_pub)
{
    ros::Time current_time, last_time;
    last_time = ros::Time::now();
    current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_data.th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_data.x;
    odom_trans.transform.translation.y = odom_data.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odom_data.x;
    odom.pose.pose.position.y = odom_data.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odom_data.vx;
    odom.twist.twist.linear.y = odom_data.vy;
    odom.twist.twist.angular.z = odom_data.vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}


