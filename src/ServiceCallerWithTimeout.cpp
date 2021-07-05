#include "ServiceCallerWithTimeout.h"

ServiceCallerWithTimeout::ServiceCallerWithTimeout(
    const std::shared_ptr<ros_babel_fish::BabelFish>& fish,
    const std::string& serviceName, const ros_babel_fish::Message::Ptr& req,
    double serviceTimeout, QObject* parent)
    : QObject{parent}, m_fish{fish}, m_serviceName{serviceName}, m_serviceTimeout{
                                                                     serviceTimeout}
{
    // Timeout Timer
    connect(&m_timeoutTimer, &QTimer::timeout, this, [this]() {
        ROS_WARN_STREAM_NAMED("service", "Timeout on service call to "
                                             << m_serviceName
                                             << ", shutdown service client");
        m_hasTimedOut = true;
        // Emit timeout signal
        emit timeout();

        // Then spawn a new thread waiting for the service thread to join
        m_timeoutThread = std::thread([this]() {
            m_serviceClient.shutdown();
            if(m_serviceThread.joinable())
            {
                m_serviceThread.join();
            }
            ROS_DEBUG_STREAM_NAMED("service",
                                   "Service thread to " << m_serviceName << " joined");
            // Once both threads have finished, delete itself later in Qt event loop
            deleteLater();
        });
    });
    m_timeoutTimer.setSingleShot(true);

    connect(this, &ServiceCallerWithTimeout::serviceCallFinished, this, [this]() {
        if(!m_hasTimedOut)
        {
            ROS_DEBUG_STREAM_NAMED("service",
                                   "Service call to " << m_serviceName << " succeeded");
            m_timeoutTimer.stop();
            m_translatedResponse = m_fish->translateMessage(m_response);
            deleteLater();
            emit success();
        }
        else
        {
            ROS_WARN_STREAM_NAMED(
                "service",
                "Service call to "
                    << m_serviceName
                    << " has just finished but already timed out, ignore response");
            // No deleteLater, already called in timeoutThread
        }
    });

    connect(this, &ServiceCallerWithTimeout::callError, this,
            [this](const QString& errorMsg) {
                m_timeoutTimer.stop();
                deleteLater();
                emit error(errorMsg);
            });

    // Content from BabelFish::callService, split here to avoid the use of m_fish in
    // the other thread
    const std::string& datatype = req->as<ros_babel_fish::CompoundMessage>().datatype();
    if(std::strcmp(datatype.c_str() + datatype.length() - 7, "Request") != 0)
    {
        throw ros_babel_fish::BabelFishException(
            "BabelFish can't call a service with a message that is not a request!");
    }
    const std::string& service_type = datatype.substr(0, datatype.length() - 7);
    const ros_babel_fish::ServiceDescription::ConstPtr& description =
        m_fish->descriptionProvider()->getServiceDescription(service_type);
    if(description == nullptr)
    {
        throw ros_babel_fish::BabelFishException(
            "BabelFish doesn't know a service of type: " + service_type);
    }

    m_request = m_fish->translateMessage(req);
    m_response = boost::make_shared<ros_babel_fish::BabelFishMessage>();
    m_response->morph(description->response);

    // Create service client
    ros::ServiceClientOptions ops(ros::names::resolve(serviceName),
                                  ros::service_traits::md5sum(*m_request), false,
                                  ros::M_string());
    m_serviceClient = m_nh.serviceClient(ops);
}

ServiceCallerWithTimeout::~ServiceCallerWithTimeout()
{
    if(m_serviceThread.joinable())
    {
        m_serviceThread.join();
    }

    if(m_timeoutThread.joinable())
    {
        m_timeoutThread.join();
    }
    ROS_DEBUG_STREAM_NAMED("service", "~ServiceCallerWithTimeout " << m_serviceName);
}

void ServiceCallerWithTimeout::call()
{
    m_timeoutTimer.start(m_serviceTimeout * 1000.0);

    // Service thread
    m_serviceThread = std::thread([this]() {
        ROS_DEBUG_STREAM_NAMED("service", "Calling service " << m_serviceName
                                                             << " from a new thread");
        if(m_serviceClient.call(*m_request, *m_response))
        {
            emit serviceCallFinished();
        }
        else
        {
            emit callError(
                QStringLiteral("Failed to call service %1")
                    .arg(QString::fromStdString(m_serviceClient.getService())));
        }
    });
}

ros_babel_fish::TranslatedMessage::Ptr ServiceCallerWithTimeout::getResponse() const
{
    return m_translatedResponse;
}
