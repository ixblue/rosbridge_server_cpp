#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

sensor_msgs::CompressedImage::ConstPtr generateMsg()
{
    sensor_msgs::CompressedImage::Ptr msg;
    msg.reset(new sensor_msgs::CompressedImage());
    msg->header.stamp.sec = 123;
    msg->header.stamp.nsec = 456;
    msg->header.seq = 100;
    msg->header.frame_id = "camera";
    msg->data.resize(100'000);
    //msg->data.resize(100);
    for(size_t i=0;i<msg->data.size();++i)
    {
        msg->data[i] = static_cast<uint8_t>(i % 256);
    }
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::CompressedImage>("chatter", 1000);

    ros::Rate loop_rate(100); // 100 hz

    auto msg = generateMsg();

    while(ros::ok())
    {
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
