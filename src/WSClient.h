#pragma once

#include <functional>
#include <map>
#include <string>

#include <QObject>
#include <QString>

#include "rapidjson/document.h"

#include "rosbridge_protocol.h"

class QWebSocket;
class ROSNode;

class WSClient : public QObject
{
    Q_OBJECT
public:
    explicit WSClient(QWebSocket* ws, ROSNode* rosNode);

    void msgCallback(const QString& topic, const rapidjson::Value& msg);
    void serviceResponseCallback(const QString& serviceName, bool success,
                                 const rapidjson::Value& response);

signals:
    void advertise(const QString& id, const QString& topic, const QString& type,
                   unsigned int queueSize, bool latched);
    void unadvertise(const QString& id, const QString& topic);
    void publish(const QString& id, const QString topicName, const rapidjson::Value& msg);
    void subscribe(const QString& id, const QString& topic, const QString& type,
                   unsigned int queueSize, int throttleRate, int fragmentSize,
                   const QString& compression);
    void unsubscribe(const QString& id, const QString& topic);
    void callService(const QString& id, const QString& serviceName,
                     const QString& serviceType, const rapidjson::Value& args,
                     int fragmentSize, const QString& compression);
    void disconected();

private slots:
    void onWSMessageReceived(const QString& message);
    void onWsDisconnected();
    void sendStatus(rosbridge_protocol::StatusLevel severity, const QString& id,
                    const QString& msg);
    void sendMsg(const QString& msg);

private:
    // Op handlers
    void advertiseHandler(const rapidjson::Value& json, const QString& id);
    void unadvertiseHandler(const rapidjson::Value& json, const QString& id);
    void publishHandler(const rapidjson::Value& json, const QString& id);
    void subscribeHandler(const rapidjson::Value& json, const QString& id);
    void unsubscribeHandler(const rapidjson::Value& json, const QString& id);
    void callServiceHandler(const rapidjson::Value& json, const QString& id);

    // Helpers
    void sendJson(const rapidjson::Document& doc);

    QWebSocket* m_ws = nullptr;
    ROSNode* m_node;
    std::map<QString, std::function<void(const rapidjson::Value&, const QString&)>>
        m_opHandlers;
};

