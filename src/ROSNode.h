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

#include "rapidjson/document.h"

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>

#include "rosbridge_protocol.h"

class WSClient;

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
    ros::Time lastTimeMsgSent;
    // TODO circular buffer
};

struct ROSBridgeSubscriber
{
    ros::Subscriber sub;
    std::vector<std::shared_ptr<SubscriberClient>> clients;
    // std::function<void(const ros_babel_fish::BabelFishMessage&)> cb;
};

class ROSNode : public QObject
{
    Q_OBJECT
public:
    explicit ROSNode(QObject* parent = nullptr);

public slots:

    //    void unadvertise(const QString& id, const QString& topic);
    //    void publish(const QString& id, const QString& topic, const rapidjson::Value&
    //    msg); void subscribe(const QString& id, const QString& topic, const QString&
    //    type,
    //                   unsigned int queueSize, int throttleRate, int fragmentSize,
    //                   const QString& compression);
    //    void unsubscribe(const QString& id, const QString& topic);
    //    void callService(const QString& id, const QString& serviceName,
    //                     const QString& serviceType, const rapidjson::Value& args,
    //                     int fragmentSize, const QString& compression);

    void onWSMessage(const QString& message);
    void onWSClientDisconnected();

private slots:
    void onNewWSConnection();
    void onWSServerError(QWebSocketProtocol::CloseCode error);

    // Op handlers
    void advertiseHandler(WSClient* client, const rapidjson::Value& json,
                          const std::string& id);
    void unadvertiseHandler(WSClient* client, const rapidjson::Value& json,
                            const std::string& id);
    void publishHandler(WSClient* client, const rapidjson::Value& json,
                        const std::string& id);
    void subscribeHandler(WSClient* client, const rapidjson::Value& json,
                          const std::string& id);
    void unsubscribeHandler(WSClient* client, const rapidjson::Value& json,
                            const std::string& id);
    void callServiceHandler(WSClient* client, const rapidjson::Value& json,
                            const std::string& id);

    // rosbridge protocol
    void advertise(WSClient* client, const rosbridge_protocol::AdvertiseArgs& args);
    void unadvertise(WSClient* client, const rosbridge_protocol::UnadvertiseArgs& args);
    void publish(WSClient* client, const rosbridge_protocol::PublishArgs& args,
                 const rapidjson::Value& msg);
    void subscribe(WSClient* client, const rosbridge_protocol::SubscribeArgs& args);
    void unsubscribe(WSClient* client, const rosbridge_protocol::UnsubscribeArgs& args);
    void callService(WSClient* client, const rosbridge_protocol::CallServiceArgs& args,
                     const rapidjson::Value& msg);

private:
    void handleROSMessage(const std::string& topic,
                          const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
    void sendStatus(WSClient* client, rosbridge_protocol::StatusLevel level,
                    const std::string& msg, const std::string& id = std::string{});
    void sendMsg(WSClient* client, const std::string& msg);
    void sendJson(WSClient* client, const rapidjson::Document& doc);

    ros::NodeHandle m_nh;
    std::map<std::string, ROSBridgePublisher> m_pubs;
    std::map<std::string, ROSBridgeSubscriber> m_subs;
    ros_babel_fish::BabelFish m_fish;

    QWebSocketServer m_wsServer;
    std::vector<std::shared_ptr<WSClient>> m_clients;

    std::map<std::string,
             std::function<void(WSClient*, const rapidjson::Value&, const std::string&)>>
        m_opHandlers;
};
