#include <QCoreApplication>

#include <librosqt/QRosCallBackQueue.h>
#include <ros/ros.h>

#include "ROSNode.h"

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(argc, argv, "p_rosbridge_server_cpp", ros::init_options::NoSigintHandler);
    ROSNode node;
    node.start();
    return QCoreApplication::exec();
}
