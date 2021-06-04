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

class WSClient;

struct ROSBridgePublisher
{
    QString type;
    ros::Publisher pub;
    std::set<WSClient*> clients;
};

struct SubscriberClient
{
    WSClient* client;
    unsigned int queueSize = 1;
    int throttleRate_ms = 0; // in ms
    int fragmentSize = -1;
    QString compression;
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
    // rosbridge protocol
    void advertise(const QString& id, const QString& topic, const QString& type,
                   unsigned int queueSize, bool latched);
    void unadvertise(const QString& id, const QString& topic);
    void publish(const QString& id, const QString& topic, const rapidjson::Value& msg);
    void subscribe(const QString& id, const QString& topic, const QString& type,
                   unsigned int queueSize, int throttleRate, int fragmentSize,
                   const QString& compression);
    void unsubscribe(const QString& id, const QString& topic);
    void callService(const QString& id, const QString& serviceName,
                     const QString& serviceType, const rapidjson::Value& args,
                     int fragmentSize, const QString& compression);

private slots:
    void onNewWSConnection();
    // void onWsConnected();
    // void onWsClosed();
    // void onWsError(QAbstractSocket::SocketError error);

private:
    void handleMessage(const QString& topic,
                       const ros_babel_fish::BabelFishMessage::ConstPtr& msg);

    ros::NodeHandle m_nh;
    std::map<QString, ROSBridgePublisher> m_pubs;
    std::map<QString, ROSBridgeSubscriber> m_subs;
    ros_babel_fish::BabelFish m_fish;

    QWebSocketServer m_wsServer;
    std::vector<std::shared_ptr<WSClient>> m_clients;
};
