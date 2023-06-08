#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"

sensor_msgs::NavSatFix::ConstPtr generateMsg()
{
    sensor_msgs::NavSatFix::Ptr msg;
    msg.reset(new sensor_msgs::NavSatFix());
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "gps_antenna";
    msg->latitude = 45.123456543;
    msg->longitude = 4.98754321;
    msg->altitude = 100.3243256;
    msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    msg->position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg->position_covariance.at(0) = 0.123;
    msg->position_covariance.at(4) = 0.123;
    msg->position_covariance.at(8) = 0.456;
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::NavSatFix>("chatter", 1000);

    ros::Rate loop_rate(100); // 100 hz
    const int msgs_per_loop = 20;

    while(ros::ok())
    {
        for(int i = 0; i < msgs_per_loop; ++i)
        {
            chatter_pub.publish(generateMsg());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
