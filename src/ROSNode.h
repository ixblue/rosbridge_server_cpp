#pragma once

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <QObject>
#include <QString>
#include <QWebSocketServer>

#include "nlohmann/json.hpp"

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>

#include "rosbridge_protocol.h"

class WSClient;
class ServiceCallerWithTimeout;

struct ROSBridgePublisher
{
    std::string type;
    ros::Publisher pub;
    std::set<const WSClient*> clients;
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

    SubscriberClient(WSClient* wsClient, const rosbridge_protocol::SubscribeArgs& args);
};

struct ROSBridgeSubscriber
{
    ros::Subscriber sub;
    std::string type;
    std::vector<std::shared_ptr<SubscriberClient>> clients;
    // Store last message if latched for new subscribers
    bool isLatched = false;
    ros_babel_fish::BabelFishMessage::ConstPtr lastMessage;
    ros::Time lastMessageReceivedTime;
};

class ROSNode : public QObject
{
    Q_OBJECT
public:
    explicit ROSNode(QObject* parent = nullptr);
    void start();

    static std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
    encodeMsgToWireFormat(ros_babel_fish::BabelFish& fish, const ros::Time& receivedTime,
                          const std::string& topic,
                          const ros_babel_fish::BabelFishMessage::ConstPtr& msg,
                          bool toJson, bool toCbor, bool toCborRaw);
    static std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
    encodeMsgToWireFormat(ros_babel_fish::BabelFish& fish, const ros::Time& receivedTime,
                          const std::string& topic,
                          const ros_babel_fish::BabelFishMessage::ConstPtr& msg,
                          rosbridge_protocol::Encoding encoding);

    static std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
    encodeServiceResponseToWireFormat(const std::string& service, const std::string& id,
                                      const nlohmann::json& values, bool result,
                                      rosbridge_protocol::Encoding encoding);

    static std::string getMandatoryNotEmptyStringFromJson(const nlohmann::json& json,
                                                          const std::string& key);

public slots:
    void onWSMessage(const QString& message);
    void onWSBinaryMessage(const QByteArray& message) const;
    void onWSClientDisconnected();

private slots:
    void onNewWSConnection();
    void onWSServerError(QWebSocketProtocol::CloseCode error) const;

    // JSON Op handlers
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
    void setLevelHandler(WSClient* client, const nlohmann::json& json,
                         const std::string& id);

    // rosbridge protocol
    void advertise(WSClient* client, const rosbridge_protocol::AdvertiseArgs& args);
    void unadvertise(WSClient* client, const rosbridge_protocol::UnadvertiseArgs& args);
    void publish(WSClient* client, const rosbridge_protocol::PublishArgs& args);
    void subscribe(WSClient* client, const rosbridge_protocol::SubscribeArgs& args);
    void unsubscribe(WSClient* client, const rosbridge_protocol::UnsubscribeArgs& args);
    void callService(WSClient* client, const rosbridge_protocol::CallServiceArgs& args);
    void setLevel(const WSClient* client, const rosbridge_protocol::SetLevelArgs& args);

signals:
    void deleteServiceClient(const QString& serviceName);

private:
    void handleROSMessage(const std::string& topic,
                          const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
    void sendStatus(WSClient* client, rosbridge_protocol::StatusLevel level,
                    const std::string& msg, const std::string& id = std::string{});
    void sendMsg(WSClient* client, const std::string& msg) const;
    void sendBinaryMsg(WSClient* client, const std::vector<uint8_t>& binaryMsg) const;
    void sendMsgToClient(WSClient* client, const std::string& jsonStr,
                         const std::vector<uint8_t>& cborVect,
                         const std::vector<uint8_t>& cborRawVect,
                         rosbridge_protocol::Encoding encoding);
    void sendTopicToClient(SubscriberClient* client, const std::string& jsonStr,
                           const std::vector<uint8_t>& cborVect,
                           const std::vector<uint8_t>& cborRawVect);
    void addNewSubscriberClient(WSClient* client,
                                const rosbridge_protocol::SubscribeArgs& args);
    void udapteSubscriberClient(SubscriberClient& c,
                                const rosbridge_protocol::SubscribeArgs& args);

    void publishStats() const;
    void produceNetworkDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

    ros::NodeHandle m_nhPrivate{"~"};
    ros::NodeHandle m_nhNs;
    ros::Publisher m_clientsCountPub;
    ros::Publisher m_connectedClientsPub;
    ros::Timer m_pubStatsTimer;
    std::map<std::string, ROSBridgePublisher> m_pubs;
    std::map<std::string, ROSBridgeSubscriber> m_subs;
    std::shared_ptr<ros_babel_fish::BabelFish> m_fish;
    std::set<rosbridge_protocol::Encoding> m_encodings;
    QWebSocketServer m_wsServer{QStringLiteral("rosbridge server"),
                                QWebSocketServer::NonSecureMode};
    std::vector<std::shared_ptr<WSClient>> m_clients;
    rosbridge_protocol::StatusLevel m_currentStatusLevel =
        rosbridge_protocol::StatusLevel::Error;

    std::map<std::string,
             std::function<void(WSClient*, const nlohmann::json&, const std::string&)>>
        m_opHandlers;

    // Parameters
    int m_wsPort = 9090;
    double m_serviceTimeout = 5.0;
    int m_maxWebSocketBufferSize_MB = 10;

    // Diags
    diagnostic_updater::Updater m_diagnostics;
    ros::Timer m_diagTimer;
    // Empty means no error
    std::string m_clientErrorMsg = "";
};
