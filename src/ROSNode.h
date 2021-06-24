#pragma once

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QWebSocketServer>

#include "nlohmann/json.hpp"

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>

#include "rosbridge_protocol.h"

class WSClient;
class ServiceCallerWithTimeout;

struct ROSBridgePublisher
{
    std::string type;
    ros::Publisher pub;
    std::set<WSClient*> clients;
};

struct SubscriberClient
{
    WSClient* client;
    unsigned int queueSize = 1;
    int throttleRate_ms = 0; // in ms
    int fragmentSize = -1;
    std::string compression;
    rosbridge_protocol::Encoding encoding = rosbridge_protocol::Encoding::JSON;
    ros::Time lastTimeMsgSent;
    // TODO circular buffer
};

struct ROSBridgeSubscriber
{
    ros::Subscriber sub;
    std::string type;
    std::vector<std::shared_ptr<SubscriberClient>> clients;
};

class ROSNode : public QObject
{
    Q_OBJECT
public:
    explicit ROSNode(QObject* parent = nullptr);
    ~ROSNode();
    void start();

public slots:
    void onWSMessage(const QString& message);
    void onWSBinaryMessage(const QByteArray& message);
    void onWSClientDisconnected();

private slots:
    void onNewWSConnection();
    void onWSServerError(QWebSocketProtocol::CloseCode error);

    // Op handlers
    void advertiseHandler(WSClient* client, const nlohmann::json& json,
                          const std::string& id);
    void unadvertiseHandler(WSClient* client, const nlohmann::json& json,
                            const std::string& id);
    void publishHandler(WSClient* client, const nlohmann::json& json,
                        const std::string& id);
    void subscribeHandler(WSClient* client, const nlohmann::json& json,
                          const std::string& id);
    void unsubscribeHandler(WSClient* client, const nlohmann::json& json,
                            const std::string& id);
    void callServiceHandler(WSClient* client, const nlohmann::json& json,
                            const std::string& id);

    // rosbridge protocol
    void advertise(WSClient* client, const rosbridge_protocol::AdvertiseArgs& args);
    void unadvertise(WSClient* client, const rosbridge_protocol::UnadvertiseArgs& args);
    void publish(WSClient* client, const rosbridge_protocol::PublishArgs& args);
    void subscribe(WSClient* client, const rosbridge_protocol::SubscribeArgs& args);
    void unsubscribe(WSClient* client, const rosbridge_protocol::UnsubscribeArgs& args);
    void callService(WSClient* client, const rosbridge_protocol::CallServiceArgs& args);

signals:
    void deleteServiceClient(const QString& serviceName);

private:
    void handleROSMessage(const std::string& topic,
                          const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
    void sendStatus(WSClient* client, rosbridge_protocol::StatusLevel level,
                    const std::string& msg, const std::string& id = std::string{});
    void sendMsg(WSClient* client, const std::string& msg);
    void sendBinaryMsg(WSClient* client, const std::vector<uint8_t>& binaryMsg);

    void publishStats();

    ros::NodeHandle m_nh;
    ros::Publisher m_clientsCountPub;
    ros::Publisher m_connectedClientsPub;
    std::map<std::string, ROSBridgePublisher> m_pubs;
    std::map<std::string, ROSBridgeSubscriber> m_subs;
    std::shared_ptr<ros_babel_fish::BabelFish> m_fish;
    std::set<rosbridge_protocol::Encoding> m_encodings;
    QWebSocketServer m_wsServer;
    std::vector<std::shared_ptr<WSClient>> m_clients;

    std::map<std::string,
             std::function<void(WSClient*, const nlohmann::json&, const std::string&)>>
        m_opHandlers;

    // Parameters
    int m_wsPort = 9090;
    double m_serviceTimeout = 5.0;
};
