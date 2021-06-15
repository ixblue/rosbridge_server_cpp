#pragma once

#include <memory>
#include <thread>

#include <QObject>
#include <QString>
#include <QTimer>

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>

class ServiceCallerWithTimeout : public QObject
{
    Q_OBJECT
public:
    ServiceCallerWithTimeout(const std::shared_ptr<ros_babel_fish::BabelFish>& fish,
                             const std::string& serviceName,
                             const ros_babel_fish::Message::Ptr& req,
                             double serviceTimeout, QObject* parent);
    ~ServiceCallerWithTimeout();

    void call();
    ros_babel_fish::TranslatedMessage::Ptr getResponse() const;

signals:
    // Service call succeeded, call getResponse() to get the response
    void success();
    // Error while calling the service
    void error(const QString& errorMsg);
    // Timeout while calling the service
    void timeout();

    // Internal use, to translate response on main thread
    void serviceCallFinished();
    void callError(const QString& errorMsg);

private:
    // BabelFish object must only be used from the main thread!
    std::shared_ptr<ros_babel_fish::BabelFish> m_fish;
    std::thread m_serviceThread;
    std::thread m_timeoutThread;
    ros::NodeHandle m_nh;
    ros::ServiceClient m_serviceClient;
    ros_babel_fish::BabelFishMessage::Ptr m_request;
    ros_babel_fish::BabelFishMessage::Ptr m_response;
    ros_babel_fish::TranslatedMessage::Ptr m_translatedResponse;
    QTimer m_timeoutTimer;
    const std::string m_serviceName;
    const double m_serviceTimeout;
    bool m_hasTimedOut = false;
};
