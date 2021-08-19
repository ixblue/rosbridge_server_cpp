#include <sstream>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QMetaObject>
#include <QWebSocket>

#include <rosbridge_msgs/ConnectedClients.h>
#include <std_msgs/Int32.h>

#include "nlohmann/json.hpp"

#include "ROSNode.h"
#include "ServiceCallerWithTimeout.h"
#include "WSClient.h"
#include "nlohmann_to_ros.h"
#include "ros_to_nlohmann.h"

namespace rbp = rosbridge_protocol;

ROSNode::ROSNode(QObject* parent)
    : QObject(parent), m_nhPrivate{"~"},
      m_opHandlers{
          {"advertise", [this](WSClient* c, const auto& j,
                               const auto& id) { advertiseHandler(c, j, id); }},
          {"unadvertise", [this](WSClient* c, const auto& j,
                                 const auto& id) { unadvertiseHandler(c, j, id); }},
          {"publish", [this](WSClient* c, const auto& j,
                             const auto& id) { publishHandler(c, j, id); }},
          {"subscribe", [this](WSClient* c, const auto& j,
                               const auto& id) { subscribeHandler(c, j, id); }},
          {"unsubscribe", [this](WSClient* c, const auto& j,
                                 const auto& id) { unsubscribeHandler(c, j, id); }},
          {"call_service", [this](WSClient* c, const auto& j,
                                  const auto& id) { callServiceHandler(c, j, id); }},
          {"set_level", [this](WSClient* c, const auto& j, const auto& id) {
               setLevelHandler(c, j, id);
           }}}
{
    // Parameters
    m_nhPrivate.getParam("port", m_wsPort);
    m_nhPrivate.getParam("service_timeout", m_serviceTimeout);

    m_clientsCountPub = m_nhNs.advertise<std_msgs::Int32>("client_count", 10, true);
    m_connectedClientsPub =
        m_nhNs.advertise<rosbridge_msgs::ConnectedClients>("connected_clients", 10, true);

    connect(&m_wsServer, &QWebSocketServer::newConnection, this,
            &ROSNode::onNewWSConnection);
    connect(&m_wsServer, &QWebSocketServer::serverError, this, &ROSNode::onWSServerError);

    m_fish = std::make_shared<ros_babel_fish::BabelFish>();
}

void ROSNode::start()
{
    if(!m_wsServer.listen(QHostAddress::Any, static_cast<uint16_t>(m_wsPort)))
    {
        ROS_FATAL_STREAM("Failed to start WS server on port "
                         << m_wsPort << ": " << m_wsServer.errorString().toStdString());
        QCoreApplication::exit(1);
    }

    ROS_INFO_STREAM("Start WS on port: " << m_wsServer.serverPort());
    m_nhPrivate.setParam("actual_port", m_wsServer.serverPort());

    publishStats();
}

std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
ROSNode::encodeToWireFormat(ros_babel_fish::BabelFish& fish,
                            const ros::Time& receivedTime, const std::string& topic,
                            const ros_babel_fish::BabelFishMessage::ConstPtr& msg,
                            bool toJson, bool toCbor, bool toCborRaw)
{
    std::string jsonStr;
    std::vector<uint8_t> cborVect;
    std::vector<uint8_t> cborRawVect;

    nlohmann::json json{{"op", "publish"}, {"topic", topic}};
    auto msgJson = ros_nlohmann_converter::toJson(fish, *msg);

    if(toJson || toCbor)
    {
        auto j = json;
        j["msg"] = msgJson;
        if(toCbor)
        {
            cborVect = nlohmann::json::to_cbor(j);
        }
        if(toJson)
        {
            jsonStr = j.dump();
        }
    }

    if(toCborRaw)
    {
        auto j = json;
        std::vector<uint8_t> data(msg->buffer(), msg->buffer() + msg->size());
        nlohmann::json::binary_t jsonBin{data};
        j["msg"] = {
            {"secs", receivedTime.sec}, {"nsecs", receivedTime.nsec}, {"bytes", jsonBin}};
        cborRawVect = nlohmann::json::to_cbor(j);
    }

    return {jsonStr, cborVect, cborRawVect};
}

void ROSNode::advertise(WSClient* client, const rbp::AdvertiseArgs& args)
{
    if(const auto it = m_pubs.find(args.topic); it != m_pubs.end())
    {
        if(args.type != it->second.type)
        {
            ROS_ERROR_STREAM("Trying to advertise topic '"
                             << args.topic << "' with type '" << args.type
                             << "' but this topic is already advertised with type '"
                             << it->second.type << "'");
            return;
        }

        ROS_DEBUG_STREAM(
            "Add a new publisher on already advertised topic: " << args.topic);
        it->second.clients.insert(client);
    }
    else
    {
        try
        {
            const auto dummyMsg = m_fish->createMessage(args.type);
        }
        catch(const ros_babel_fish::BabelFishException&)
        {
            std::ostringstream ss;
            ss << "Unknown message type '" << args.type << "'";
            sendStatus(client, rbp::StatusLevel::Error, ss.str());
            return;
        }

        ROS_DEBUG_STREAM("Create a new publisher on topic: " << args.topic);
        m_pubs.emplace(
            args.topic,
            ROSBridgePublisher{args.type,
                               m_fish->advertise(m_nhPrivate, args.type, args.topic,
                                                 args.queueSize, args.latched),
                               {client}});
    }
}

void ROSNode::unadvertise(WSClient* client, const rbp::UnadvertiseArgs& args)
{
    if(const auto it = m_pubs.find(args.topic); it != m_pubs.end())
    {
        ROS_DEBUG_STREAM("Remove a publisher on topic: " << args.topic);
        it->second.clients.erase(client);
        if(it->second.clients.empty())
        {
            ROS_DEBUG_STREAM("No more publisher on topic: "
                             << args.topic << " unadvertise and delete it");
            m_pubs.erase(it);
        }
    }
    else
    {
        std::ostringstream ss;
        ss << "unadvertise cmd on a topic not advertised: '" << args.topic << "'";
        sendStatus(client, rbp::StatusLevel::Error, ss.str());
    }
}

void ROSNode::publish(WSClient* client, const rbp::PublishArgs& args)
{
    if(const auto it = m_pubs.find(args.topic); it != m_pubs.end())
    {
        try
        {
            ros_babel_fish::BabelFishMessage::Ptr rosMsg =
                ros_nlohmann_converter::createMsg(*m_fish, it->second.type,
                                                  ros::Time::now(), args.msg);

            ROS_DEBUG_STREAM("Publish a msg on topic " << args.topic);
            it->second.pub.publish(rosMsg);
        }
        catch(const ros_babel_fish::BabelFishException& e)
        {
            std::ostringstream ss;
            ss << "publish cmd on topic with unknown type: '" << it->second.type
               << "': " << e.what();
            sendStatus(client, rbp::StatusLevel::Error, ss.str());
            return;
        }
    }
    else
    {
        std::ostringstream ss;
        ss << "publish cmd on topic not advertised: '" << args.topic << "'";
        sendStatus(client, rbp::StatusLevel::Error, ss.str());
        return;
    }
}

void ROSNode::udapteSubscriberClient(SubscriberClient& c, const rbp::SubscribeArgs& args)
{
    if(c.compression != args.compression)
    {
        std::ostringstream ss;
        ss << "Client is subscribing once more on topic " << args.topic
           << " but with compression " << args.compression
           << " while already subscribed to this topic but with compression "
           << c.compression << " switching to compression " << args.compression;
        sendStatus(c.client, rbp::StatusLevel::Warning, ss.str());
    }
    else
    {
        ROS_DEBUG_STREAM("Client is subscribing once more on the " << args.topic);
    }

    c.throttleRate_ms = std::min(c.throttleRate_ms, args.throttleRate);
    c.fragmentSize = std::min(c.fragmentSize, args.fragmentSize);
    c.queueSize = std::min(c.queueSize, args.queueSize);
    c.compression = args.compression;
    c.setEncoding(args.compression);
    m_encodings.insert(c.encoding);
}

void ROSNode::subscribe(WSClient* client, const rbp::SubscribeArgs& args)
{
    if(const auto it = m_subs.find(args.topic); it != m_subs.end())
    {
        std::shared_ptr<SubscriberClient> subClient;

        // topic already subscribed, check if this client already registered
        if(const auto clientIt = std::find_if(
               it->second.clients.begin(), it->second.clients.end(),
               [&client](const auto& clientSub) { return clientSub->client == client; });
           clientIt != it->second.clients.end())
        {
            udapteSubscriberClient(*(*clientIt), args);
            subClient = *clientIt;
        }
        else
        {
            ROS_DEBUG_STREAM("Add a new client subscribed on topic " << args.topic);
            subClient = std::make_shared<SubscriberClient>(client, args);
            it->second.clients.push_back(subClient);
            m_encodings.insert(subClient->encoding);
        }

        // If the topic is latched, send the last message directly to the new client
        if(it->second.isLatched && subClient)
        {
            const bool toJson = subClient->encoding == rbp::Encoding::JSON;
            const bool toCbor = subClient->encoding == rbp::Encoding::CBOR;
            const bool toCborRaw = subClient->encoding == rbp::Encoding::CBOR_RAW;
            const auto [jsonStr, cborVect, cborRawVect] =
                encodeToWireFormat(*m_fish, it->second.lastMessageReceivedTime, it->first,
                                   it->second.lastMessage, toJson, toCbor, toCborRaw);
            sendToClient(subClient.get(), jsonStr, cborVect, cborRawVect);
        }
    }
    else
    {
        addNewSubscriberClient(client, args);
    }
}

void ROSNode::unsubscribe(WSClient* client, const rbp::UnsubscribeArgs& args)
{
    if(const auto it = m_subs.find(args.topic); it != m_subs.end())
    {
        ROS_DEBUG_STREAM("A client unubscribe to topic " << args.topic);

        it->second.clients.erase(
            std::remove_if(it->second.clients.begin(), it->second.clients.end(),
                           [client](const auto& elem) { return client == elem->client; }),
            it->second.clients.end());

        if(it->second.clients.empty())
        {
            ROS_DEBUG_STREAM("No more subscriber on topic "
                             << args.topic << ", unsubscribe to this topic");
            m_subs.erase(it);
        }
    }
    else
    {
        std::ostringstream ss;
        ss << "unsubscribe cmd on a topic not subscribed: '" << args.topic << "'";
        sendStatus(client, rbp::StatusLevel::Error, ss.str());
    }
}

void ROSNode::callService(WSClient* client, const rbp::CallServiceArgs& args)
{
    ROS_INFO_STREAM_NAMED("service", "Call service " << args.serviceName);
    try
    {
        ros_babel_fish::Message::Ptr req = m_fish->createServiceRequest(args.serviceType);
        auto& compound = req->as<ros_babel_fish::CompoundMessage>();
        ros_nlohmann_converter::fillMessageFromJson(args.args, compound);

        // Allocated on heap, will be deleted by calling deleteLater itself later
        // Used to properly delete in the timeoutThread
        auto serviceClient = new ServiceCallerWithTimeout(m_fish, args.serviceName, req,
                                                          m_serviceTimeout, this);

        connect(serviceClient, &ServiceCallerWithTimeout::success, this,
                [this, id = args.id, serviceName = args.serviceName,
                 compression = args.compression, client]() {
                    auto* serviceClient =
                        qobject_cast<ServiceCallerWithTimeout*>(sender());
                    if(client != nullptr)
                    {
                        auto res = serviceClient->getResponse();

                        nlohmann::json json;
                        json["op"] = "service_response";
                        json["service"] = serviceName;
                        if(!id.empty())
                        {
                            json["id"] = id;
                        }
                        json["result"] = true;

                        nlohmann::json msgJson =
                            ros_nlohmann_converter::translatedMsgtoJson(
                                *res->translated_message);

                        // Encode to json only once
                        if(compression == "cbor-raw")
                        {
                            std::vector<uint8_t> data(res->input_message->buffer(),
                                                      res->input_message->buffer() +
                                                          res->input_message->size());
                            nlohmann::json::binary_t jsonBin{data};
                            json["values"] = {{"bytes", jsonBin}};
                            sendBinaryMsg(client, nlohmann::json::to_cbor(json));
                        }
                        else
                        {
                            json["values"] = msgJson;
                            if(compression == "cbor")
                            {
                                sendBinaryMsg(client, nlohmann::json::to_cbor(json));
                            }
                            sendMsg(client, json.dump());
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM_NAMED("service",
                                               "Received response for service "
                                                   << serviceName
                                                   << " but client ptr is now null");
                    }

                    // Disconnect allows to drop all copies of serviceClient shared_ptr
                    serviceClient->disconnect();
                });

        connect(serviceClient, &ServiceCallerWithTimeout::error, this,
                [this, id = args.id,compression = args.compression, client, serviceName = args.serviceName](const QString& errorMsg) {

                    if(client != nullptr)
                    {
                        nlohmann::json json;
                        json["op"] = "service_response";
                        json["service"] = serviceName;
                        json["values"] = {errorMsg.toStdString()};
                        if(!id.empty())
                        {
                            json["id"] = id;
                        }
                        json["result"] = false;

                        // Encode to json only once
                        if(compression == "cbor-raw")
                        {
                            sendBinaryMsg(client, nlohmann::json::to_cbor(json));
                        }
                        else
                        {
                            if(compression == "cbor")
                            {
                                sendBinaryMsg(client, nlohmann::json::to_cbor(json));
                            }
                            sendMsg(client, json.dump());
                        }
                        sendStatus(client, rbp::StatusLevel::Error,
                                   errorMsg.toStdString());
                    }

                    // Disconnect allows to drop all copies of serviceClient shared_ptr
                    sender()->disconnect();
                });
        connect(serviceClient, &ServiceCallerWithTimeout::timeout, this,
                [this, client, serviceName = args.serviceName]() {
                    if(client != nullptr)
                    {
                        std::ostringstream ss;
                        ss << "Service " << serviceName << " call timeout";
                        sendStatus(client, rbp::StatusLevel::Error, ss.str());
                    }

                    // Disconnect allows to drop all copies of serviceClient shared_ptr
                    sender()->disconnect();
                });

        serviceClient->call();
    }
    catch(const ros_babel_fish::BabelFishException& e)
    {
        std::ostringstream ss;
        ss << "Service call on unknown service type: '" << args.serviceType
           << "': " << e.what();
        sendStatus(client, rbp::StatusLevel::Error, ss.str());

        nlohmann::json json;
        json["op"] = "service_response";
        json["service"] = args.serviceName;
        json["values"] = {"Unknown service type : " + args.serviceType};
        if(!args.id.empty())
        {
            json["id"] = args.id;
        }
        json["result"] = false;

        // Encode to json only once
        if(args.compression == "cbor-raw")
        {
            sendBinaryMsg(client, nlohmann::json::to_cbor(json));
        }
        else
        {
            if(args.compression == "cbor")
            {
                sendBinaryMsg(client, nlohmann::json::to_cbor(json));
            }
            sendMsg(client, json.dump());
        }

        return;
    }
}

void ROSNode::setLevel(WSClient* client, const rosbridge_protocol::SetLevelArgs& args)
{
    Q_UNUSED(client)
    m_currentStatusLevel = args.level;
    ROS_INFO_STREAM("Change status level to "
                    << rbp::statusLevelStringMap.at(args.level));
}

void ROSNode::onWSMessage(const QString& message)
{
    ROS_DEBUG_STREAM_NAMED("json",
                           "<- Received on ws: '" << message.toStdString() << "'");

    auto* client = qobject_cast<WSClient*>(sender());

    try
    {
        const auto json = nlohmann::json::parse(message.toStdString());

        if(!json.is_object())
        {
            ROS_WARN_STREAM(
                "Received JSON is not a rosbridge message (not a JSON object): "
                << message.toStdString());
            return;
        }

        if(const auto opIt = json.find("op"); opIt != json.end())
        {
            const auto op = opIt->get<std::string>();

            std::string id;
            if(const auto it = json.find("id"); it != json.end())
            {
                id = it->get<std::string>();
            }

            if(const auto it = m_opHandlers.find(op); it != m_opHandlers.end())
            {
                it->second(client, json, id);
            }
            else
            {
                ROS_ERROR_STREAM("Received unkown OP '" << op << "' ignoring");
            }
        }
        else
        {
            ROS_WARN_STREAM("Received JSON is not a rosbridge message (on 'op' element): "
                            << message.toStdString());
            return;
        }
    }
    catch(const nlohmann::json::exception& e)
    {
        ROS_ERROR_STREAM("Failed to parse the JSON message: "
                         << e.what() << " message: '" << message.toStdString() << "'");
    }
}

void ROSNode::onWSBinaryMessage(const QByteArray& message)
{
    Q_UNUSED(message)
    ROS_WARN_STREAM("Unhandled binary message received on WS");
}

void ROSNode::onWSClientDisconnected()
{
    const auto* client = qobject_cast<WSClient*>(sender());

    const auto clientName = client->name();

    for(auto pubIt = m_pubs.begin(); pubIt != m_pubs.end();)
    {
        bool hasDeleted = false;
        if(const auto clientIt = pubIt->second.clients.find(client);
           clientIt != pubIt->second.clients.end())
        {
            pubIt->second.clients.erase(clientIt);
            if(pubIt->second.clients.empty())
            {
                pubIt = m_pubs.erase(pubIt);
                hasDeleted = true;
            }
        }
        if(!hasDeleted)
        {
            pubIt++;
        }
    }

    for(auto subIt = m_subs.begin(); subIt != m_subs.end();)
    {
        bool hasDeleted = false;
        if(const auto clientIt = std::find_if(
               subIt->second.clients.begin(), subIt->second.clients.end(),
               [client](const auto& elem) { return client == elem->client; });
           clientIt != subIt->second.clients.end())
        {
            subIt->second.clients.erase(clientIt);
            if(subIt->second.clients.empty())
            {
                subIt = m_subs.erase(subIt);
                hasDeleted = true;
            }
        }
        if(!hasDeleted)
        {
            subIt++;
        }
    }

    m_clients.erase(
        std::remove_if(m_clients.begin(), m_clients.end(),
                       [client](const auto& elem) { return client == elem.get(); }),
        m_clients.end());

    ROS_INFO_STREAM("Client " << clientName << " disconnected (" << m_clients.size()
                              << " clients)");
    publishStats();
}

void ROSNode::onNewWSConnection()
{
    QWebSocket* socket = m_wsServer.nextPendingConnection();
    ROS_INFO_STREAM("New client connected! "
                    << socket->peerAddress().toString().toStdString() << ":"
                    << socket->peerPort());

    auto client = std::make_shared<WSClient>(socket);
    client->connectSignals();
    connect(client.get(), &WSClient::onWSMessage, this, &ROSNode::onWSMessage);
    connect(client.get(), &WSClient::onWSBinaryMessage, this, &ROSNode::onWSMessage);
    connect(client.get(), &WSClient::disconected, this, &ROSNode::onWSClientDisconnected);

    m_clients.push_back(client);

    publishStats();
}

void ROSNode::onWSServerError(QWebSocketProtocol::CloseCode error)
{
    ROS_ERROR_STREAM("Websocket server error ("
                     << static_cast<int>(error)
                     << "): " << m_wsServer.errorString().toStdString());
}

void ROSNode::handleROSMessage(const std::string& topic,
                               const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
    try
    {
        auto receivedTime = ros::Time::now();
        ROS_DEBUG_STREAM_ONCE("Received ROS msg on topic " << topic);
        ROS_DEBUG_STREAM_NAMED("topic", "Handle ROS msg on topic "
                                            << topic << " latched: " << msg->isLatched());
        const bool toJson = m_encodings.count(rbp::Encoding::JSON) > 0;
        const bool toCbor = m_encodings.count(rbp::Encoding::CBOR) > 0;
        const bool toCborRaw = m_encodings.count(rbp::Encoding::CBOR_RAW) > 0;
        QElapsedTimer t;
        auto [jsonStr, cborVect, cborRawVect] = encodeToWireFormat(
            *m_fish, receivedTime, topic, msg, toJson, toCbor, toCborRaw);

        ROS_DEBUG_STREAM_NAMED("topic", "Converted ROS msg on topic "
                                            << topic << " to wire format(s) in "
                                            << t.nsecsElapsed() / 1000 << " us");

        if(const auto it = m_subs.find(topic); it != m_subs.end())
        {
            // Store latched msg for a new client
            if(msg->isLatched())
            {
                it->second.isLatched = true;
                it->second.lastMessage = msg;
                it->second.lastMessageReceivedTime = receivedTime;
            }

            for(auto& client : it->second.clients)
            {
                if(client->client->isReady())
                {
                    if(client->throttleRate_ms == 0 ||
                       (ros::Time::now() - client->lastTimeMsgSent).toSec() >
                           client->throttleRate_ms / 1000.)
                    {
                        sendToClient(client.get(), jsonStr, cborVect, cborRawVect);
                    }
                }
                else
                {
                    ROS_WARN_STREAM("Client " << client->client->name()
                                              << " not ready, skip this msg");
                }
            }
        }
    }
    catch(const ros_babel_fish::BabelFishException& e)
    {
        std::ostringstream ss;
        ss << "Error on ROS message received on topic " << topic;
        const auto it = m_subs.find(topic);
        if(it != m_subs.end())
        {
            ss << " of type " << it->second.type;
        }
        ss << ": " << e.what();

        if(it != m_subs.end())
        {
            for(auto& client : it->second.clients)
            {

                sendStatus(client->client, rbp::StatusLevel::Error, ss.str(),
                           "subscribe_" + topic);
            }
        }
        else
        {
            ROS_ERROR_STREAM(ss.str());
        }
    }
}

void ROSNode::sendStatus(WSClient* client, rbp::StatusLevel level, const std::string& msg,
                         const std::string& id)
{
    if(level == rbp::StatusLevel::None)
    {
        ROS_ERROR_STREAM(
            "Tried to send status with level None, not possible. Message was: " << msg);
        return;
    }

    switch(level)
    {
    case rbp::StatusLevel::Info: ROS_INFO_STREAM(msg); break;
    case rbp::StatusLevel::Warning: ROS_WARN_STREAM(msg); break;
    case rbp::StatusLevel::Error: ROS_ERROR_STREAM(msg); break;
    case rbp::StatusLevel::None: ROS_DEBUG_STREAM(msg); break;
    }

    if(level >= m_currentStatusLevel)
    {
        nlohmann::json json{{"op", "status"},
                            {"level", rbp::statusLevelStringMap.at(level)},
                            {"msg", msg}};
        if(!id.empty())
        {
            json["id"] = id;
        }

        sendMsg(client, json.dump());
    }
}

void ROSNode::sendMsg(WSClient* client, const std::string& msg)
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send on ws: '" << msg << "'");
    QMetaObject::invokeMethod(client, "sendMsg",
                              Q_ARG(QString, QString::fromStdString(msg)));
}

void ROSNode::sendBinaryMsg(WSClient* client, const std::vector<uint8_t>& binaryMsg)
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send binary msg on ws");
    auto data =
        QByteArray(reinterpret_cast<const char*>(binaryMsg.data()), binaryMsg.size());
    QMetaObject::invokeMethod(client, "sendBinaryMsg", Q_ARG(QByteArray, data));
}

void ROSNode::sendToClient(SubscriberClient* client, const std::string& jsonStr,
                           const std::vector<uint8_t>& cborVect,
                           const std::vector<uint8_t>& cborRawVect)
{
    if(client->encoding == rbp::Encoding::CBOR)
    {
        sendBinaryMsg(client->client, cborVect);
    }
    else if(client->encoding == rbp::Encoding::CBOR_RAW)
    {
        sendBinaryMsg(client->client, cborRawVect);
    }
    else
    {
        sendMsg(client->client, jsonStr);
    }
    client->lastTimeMsgSent = ros::Time::now();
}

void ROSNode::addNewSubscriberClient(WSClient* client,
                                     const rosbridge_protocol::SubscribeArgs& args)
{
    // Try to create a msg to check that we can find it
    try
    {
        const auto dummyMsg = m_fish->createMessage(args.type);
    }
    catch(const ros_babel_fish::BabelFishException&)
    {
        std::ostringstream ss;
        ss << "Unknown message type '" << args.type << "'";
        sendStatus(client, rbp::StatusLevel::Error, ss.str());
        return;
    }

    ROSBridgeSubscriber sub;
    sub.type = args.type;
    sub.sub = m_nhPrivate.subscribe<ros_babel_fish::BabelFishMessage>(
        args.topic, 100,
        [this,
         topic = args.topic](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) {
            handleROSMessage(topic, msg);
        });

    sub.clients.push_back(std::make_shared<SubscriberClient>(client, args));
    m_encodings.insert(sub.clients.back()->encoding);
    m_subs.emplace(args.topic, std::move(sub));
    ROS_DEBUG_STREAM("Subscribe to a new topic " << args.topic);
}

void ROSNode::publishStats()
{
    ROS_DEBUG("publishStats");
    {
        std_msgs::Int32 msg;
        msg.data = static_cast<int32_t>(m_clients.size());
        m_clientsCountPub.publish(msg);
    }
    {
        rosbridge_msgs::ConnectedClients msg;
        msg.clients.reserve(m_clients.size());
        for(const auto& client : m_clients)
        {
            rosbridge_msgs::ConnectedClient c;
            c.ip_address = client->ipAddress();
            c.connection_time = client->connectionTime();
            msg.clients.push_back(c);
        }
        m_connectedClientsPub.publish(msg);
    }
}

void ROSNode::advertiseHandler(WSClient* client, const nlohmann::json& json,
                               const std::string& id)
{
    rbp::AdvertiseArgs args;
    args.id = id;

    if(const auto it = json.find("topic"); it != json.end())
    {
        args.topic = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'advertise' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.find("type"); it != json.end())
    {
        args.type = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'advertise' msg without required 'type' key");
        return;
    }

    if(const auto it = json.find("latch"); it != json.end())
    {
        args.latched = it->get<bool>();
    }

    if(const auto it = json.find("queue_size"); it != json.end())
    {
        args.queueSize = it->get<unsigned int>();
    }

    advertise(client, args);
}

void ROSNode::unadvertiseHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    rbp::UnadvertiseArgs args;
    args.id = id;

    if(const auto it = json.find("topic"); it != json.end())
    {
        args.topic = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'unadvertise' msg without required 'topic' key");
        return;
    }

    unadvertise(client, args);
}

void ROSNode::publishHandler(WSClient* client, const nlohmann::json& json,
                             const std::string& id)
{
    rbp::PublishArgs args;
    args.id = id;

    if(const auto it = json.find("topic"); it != json.end())
    {
        args.topic = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'publish' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.find("msg"); it != json.end())
    {
        args.msg = *it;
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'publish' msg without required 'msg' key");
        return;
    }

    publish(client, args);
}

void ROSNode::subscribeHandler(WSClient* client, const nlohmann::json& json,
                               const std::string& id)
{
    rbp::SubscribeArgs args;
    args.id = id;

    if(const auto it = json.find("topic"); it != json.end())
    {
        args.topic = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'subscribe' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.find("type"); it != json.end())
    {
        args.type = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'subscribe' msg without required 'type' key");
        return;
    }

    if(const auto it = json.find("throttle_rate"); it != json.end())
    {
        args.throttleRate = it->get<int>();
    }

    if(const auto it = json.find("queue_size"); it != json.end())
    {
        args.queueSize = it->get<unsigned int>();
    }

    if(const auto it = json.find("fragment_size"); it != json.end())
    {
        args.fragmentSize = it->get<int>();
    }

    if(const auto it = json.find("compression"); it != json.end())
    {
        args.compression = it->get<std::string>();
    }

    subscribe(client, args);
}

void ROSNode::unsubscribeHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    rbp::UnsubscribeArgs args;
    args.id = id;

    if(const auto it = json.find("topic"); it != json.end())
    {
        args.topic = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'unsubscribe' msg without required 'topic' key");
        return;
    }

    unsubscribe(client, args);
}

void ROSNode::callServiceHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    rbp::CallServiceArgs args;
    args.id = id;

    if(const auto it = json.find("service"); it != json.end())
    {
        args.serviceName = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'call_service' msg without required 'service' key");
        return;
    }

    if(const auto it = json.find("type"); it != json.end())
    {
        args.serviceType = it->get<std::string>();
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'call_service' msg without required 'type' key");
        return;
    }

    if(const auto it = json.find("fragment_size"); it != json.end())
    {
        args.fragmentSize = it->get<int>();
    }

    if(const auto it = json.find("compression"); it != json.end())
    {
        args.compression = it->get<std::string>();
    }

    if(const auto it = json.find("args"); it != json.end())
    {
        args.args = *it;
    }
    callService(client, args);
}

void ROSNode::setLevelHandler(WSClient* client, const nlohmann::json& json,
                              const std::string& id)
{
    rbp::SetLevelArgs args;
    args.id = id;

    if(const auto it = json.find("level"); it != json.end())
    {
        const QString desiredLevel =
            QString::fromStdString(it->get<std::string>()).toLower();
        if(desiredLevel == "info")
        {
            args.level = rbp::StatusLevel::Info;
        }
        else if(desiredLevel == "warn")
        {
            args.level = rbp::StatusLevel::Warning;
        }
        else if(desiredLevel == "error")
        {
            args.level = rbp::StatusLevel::Error;
        }
        else if(desiredLevel == "none")
        {
            args.level = rbp::StatusLevel::None;
        }
        else
        {
            std::ostringstream ss;
            ss << "Received 'set_level' request with unknown status level: '"
               << desiredLevel.toStdString() << "', accepted values are: [";
            for(const auto& [level, str] : rbp::statusLevelStringMap)
            {
                (void)level;
                ss << str << " ";
            }
            ss << "]";
            sendStatus(client, rbp::StatusLevel::Error, ss.str());
            return;
        }
    }
    else
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'set_level' msg without required 'level' key");
        return;
    }

    setLevel(client, args);
}

SubscriberClient::SubscriberClient(WSClient* wsClient, const rbp::SubscribeArgs& args)
    : client{wsClient}, queueSize{args.queueSize}, throttleRate_ms{args.throttleRate},
      fragmentSize{args.fragmentSize}, compression{args.compression}
{
    setEncoding(args.compression);
}

void SubscriberClient::setEncoding(const std::string& compression)
{
    // Default = JSON, PNG not implemented
    if(compression == "cbor")
    {
        encoding = rbp::Encoding::CBOR;
    }
    else if(compression == "cbor-raw")
    {
        encoding = rbp::Encoding::CBOR_RAW;
    }
    else
    {
        encoding = rbp::Encoding::JSON;
    }
}
