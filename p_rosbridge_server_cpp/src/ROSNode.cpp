#include <sstream>

#include <boost/algorithm/string.hpp>

#include <QCoreApplication>
#include <stdexcept>

#include <QElapsedTimer>
#include <QMetaObject>
#include <QWebSocket>

#include <rosbridge_cpp_msgs/WebSocketConnectedClients.h>
#include <std_msgs/Int32.h>
#include <string>

#include "nlohmann/json.hpp"

#include "ROSNode.h"
#include "ServiceCallerWithTimeout.h"
#include "WSClient.h"
#include "nlohmann_to_ros.h"
#include "ros_to_nlohmann.h"

using namespace std::string_literals;
namespace rbp = rosbridge_protocol;

ROSNode::ROSNode(QObject* parent)
    : QObject(parent), m_nhPrivate{"~"}, m_watchdog(5.0),
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
    m_nhPrivate.getParam("max_wsocket_buffer_size_mbytes", m_maxWebSocketBufferSize_MB);
    m_nhPrivate.getParam("pong_timeout_s", m_pongTimeout_s);

    m_clientsCountPub = m_nhNs.advertise<std_msgs::Int32>("client_count", 10, true);
    m_connectedClientsPub =
        m_nhNs.advertise<rosbridge_cpp_msgs::WebSocketConnectedClients>(
            "connected_clients", 10, true);

    connect(&m_wsServer, &QWebSocketServer::newConnection, this,
            &ROSNode::onNewWSConnection);
    connect(&m_wsServer, &QWebSocketServer::serverError, this, &ROSNode::onWSServerError);

    m_fish = std::make_shared<ros_babel_fish::BabelFish>();

    m_diagnostics.add("Network", this, &ROSNode::produceNetworkDiagnostics);
    m_diagnostics.setHardwareID("rosbridge");
    m_diagTimer = m_nhPrivate.createTimer(
        ros::Duration(1.0), [this](const ros::TimerEvent&) { m_diagnostics.update(); });

    m_pubStatsTimer = m_nhPrivate.createTimer(
        ros::Duration(10.0), [this](const ros::TimerEvent&) { publishStats(); });
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
ROSNode::encodeMsgToWireFormat(ros_babel_fish::BabelFish& fish,
                               const ros::Time& receivedTime, const std::string& topic,
                               const ros_babel_fish::BabelFishMessage::ConstPtr& msg,
                               bool toJson, bool toCbor, bool toCborRaw)
{
    std::string jsonStr;
    std::vector<uint8_t> cborVect;
    std::vector<uint8_t> cborRawVect;

    if(toJson || toCbor)
    {
        if(toCbor)
        {
            const auto msgJson = ros_nlohmann_converter::toBinaryJson(fish, *msg);
            const nlohmann::json j{{"op", "publish"}, {"topic", topic}, {"msg", msgJson}};
            nlohmann::json::to_cbor(j, cborVect);
        }
        if(toJson)
        {
            const auto msgJson = ros_nlohmann_converter::toJson(fish, *msg);
            const nlohmann::json j{{"op", "publish"}, {"topic", topic}, {"msg", msgJson}};
            jsonStr = ros_nlohmann_converter::dumpJson(j);
        }
    }

    if(toCborRaw)
    {
        std::vector<uint8_t> data(msg->buffer(), msg->buffer() + msg->size());
        nlohmann::json::binary_t jsonBin{data};
        const nlohmann::json j{{"op", "publish"},
                               {"topic", topic},
                               {"msg",
                                {{"secs", receivedTime.sec},
                                 {"nsecs", receivedTime.nsec},
                                 {"bytes", jsonBin}}}};
        nlohmann::json::to_cbor(j, cborRawVect);
    }

    return {jsonStr, cborVect, cborRawVect};
}

std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
ROSNode::encodeMsgToWireFormat(ros_babel_fish::BabelFish& fish,
                               const ros::Time& receivedTime, const std::string& topic,
                               const ros_babel_fish::BabelFishMessage::ConstPtr& msg,
                               rosbridge_protocol::Encoding encoding)
{
    const bool toJson = encoding == rbp::Encoding::JSON;
    const bool toCbor = encoding == rbp::Encoding::CBOR;
    const bool toCborRaw = encoding == rbp::Encoding::CBOR_RAW;
    return encodeMsgToWireFormat(fish, receivedTime, topic, msg, toJson, toCbor,
                                 toCborRaw);
}

std::tuple<std::string, std::vector<uint8_t>, std::vector<uint8_t>>
ROSNode::encodeServiceResponseToWireFormat(const std::string& service,
                                           const std::string& id,
                                           const nlohmann::json& values, bool result,
                                           rosbridge_protocol::Encoding encoding)
{
    nlohmann::json json;
    json["op"] = "service_response";
    json["service"] = service;
    if(!id.empty())
    {
        json["id"] = id;
    }
    json["result"] = result;
    json["values"] = values;

    if((encoding != rbp::Encoding::JSON) && (encoding != rbp::Encoding::CBOR))
    {
        ROS_ERROR_STREAM("Only JSON and CBOR encoding supported for service response, "
                         "defaulting to JSON");
    }

    if(encoding == rbp::Encoding::CBOR)
    {
        return {"", nlohmann::json::to_cbor(json), {}};
    }
    return {ros_nlohmann_converter::dumpJson(json), {}, {}};
}

std::string ROSNode::getMandatoryNotEmptyStringFromJson(const nlohmann::json& json,
                                                        const std::string& key)
{
    if(const auto it = json.find(key); it != json.end())
    {
        const auto str = it->get<std::string>();
        if(!str.empty())
        {
            return str;
        }
    }
    std::ostringstream ss;
    ss << "without required '" << key << "' key";
    throw std::runtime_error(ss.str());
}

void ROSNode::SetServiceCallTimeout(double timeout)
{
    m_serviceTimeout = timeout;
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
    c.encoding = rbp::compressionToEncoding(args.compression);
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
        }

        // If the topic is latched, send the last message directly to the new client
        if(it->second.isLatched && subClient)
        {
            const auto [jsonStr, cborVect, cborRawVect] = encodeMsgToWireFormat(
                *m_fish, it->second.lastMessageReceivedTime, it->first,
                it->second.lastMessage, subClient->encoding);
            sendTopicToClient(subClient.get(), jsonStr, cborVect, cborRawVect);
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
    if(client == nullptr)
    {
        ROS_ERROR_STREAM_NAMED("service", "called service with nullptr client");
        return;
    }

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

        // establish the qt connection on the WSClient context: if the connection is
        // deleted it will be disconnected automatically

        connect(serviceClient, &ServiceCallerWithTimeout::success, client,
                [this, id = args.id, compression = args.compression,
                 serviceName = args.serviceName, client, serviceClient]() {
                    auto res = serviceClient->getResponse();

                    const auto encoding = rbp::compressionToEncoding(compression);

                    nlohmann::json responseJson =
                        ros_nlohmann_converter::translatedMsgtoJson(
                            *res->translated_message,
                            encoding == rosbridge_protocol::Encoding::CBOR);

                    const auto [json, cbor, cborRaw] = encodeServiceResponseToWireFormat(
                        serviceName, id, responseJson, true, encoding);
                    sendMsgToClient(client, json, cbor, cborRaw, encoding);
                });

        connect(serviceClient, &ServiceCallerWithTimeout::error, client,
                [this, id = args.id, compression = args.compression, client,
                 serviceName = args.serviceName](const QString& errorMsg) {
                    const auto encoding = rbp::compressionToEncoding(compression);
                    const auto [json, cbor, cborRaw] = encodeServiceResponseToWireFormat(
                        serviceName, id, {errorMsg.toStdString()}, false, encoding);
                    sendMsgToClient(client, json, cbor, cborRaw, encoding);

                    sendStatus(client, rbp::StatusLevel::Error, errorMsg.toStdString());
                });

        connect(serviceClient, &ServiceCallerWithTimeout::timeout, client,
                [this, id = args.id, compression = args.compression, client,
                 serviceName = args.serviceName]() {
                    std::ostringstream ss;
                    ss << "Service " << serviceName << " call timeout";

                    const auto encoding = rbp::compressionToEncoding(compression);
                    const auto [json, cbor, cborRaw] = encodeServiceResponseToWireFormat(
                        serviceName, id, {ss.str()}, false, encoding);
                    sendMsgToClient(client, json, cbor, cborRaw, encoding);

                    sendStatus(client, rbp::StatusLevel::Error, ss.str());
                });

        serviceClient->call();
    }
    catch(const ros_babel_fish::BabelFishException& e)
    {
        std::ostringstream ss;
        ss << "Service call on unknown service type: '" << args.serviceType
           << "': " << e.what();

        const auto encoding = rbp::compressionToEncoding(args.compression);
        const auto [json, cbor, cborRaw] = encodeServiceResponseToWireFormat(
            args.serviceName, args.id, {ss.str()}, false, encoding);
        sendMsgToClient(client, json, cbor, cborRaw, encoding);

        sendStatus(client, rbp::StatusLevel::Error, ss.str());
        return;
    }
    catch(const std::runtime_error& e)
    {
        std::ostringstream ss;
        ss << "Bad service args: '" << ros_nlohmann_converter::dumpJson(args.args)
           << "': " << e.what();
        sendStatus(client, rbp::StatusLevel::Error, ss.str());

        const auto encoding = rbp::compressionToEncoding(args.compression);
        const auto [json, cbor, cborRaw] = encodeServiceResponseToWireFormat(
            args.serviceName, args.id, {ss.str()}, false, encoding);
        sendMsgToClient(client, json, cbor, cborRaw, encoding);

        return;
    }
}

void ROSNode::setLevel(const WSClient* client,
                       const rosbridge_protocol::SetLevelArgs& args)
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
            std::ostringstream ss;
            ss << "Received JSON is not a rosbridge message (not a JSON object): '"
               << message.toStdString() << '"';
            sendStatus(client, rbp::StatusLevel::Error, ss.str());
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
                std::ostringstream ss;
                ss << "Received unkown OP '" << op << "' ignoring: '"
                   << message.toStdString() << "'";
                sendStatus(client, rbp::StatusLevel::Error, ss.str());
                return;
            }
        }
        else
        {
            std::ostringstream ss;
            ss << "Received JSON is not a rosbridge message (no 'op' element): '"
               << message.toStdString() << "'";
            sendStatus(client, rbp::StatusLevel::Error, ss.str());
            return;
        }
    }
    catch(const nlohmann::json::exception& e)
    {
        std::ostringstream ss;
        ss << "Failed to parse the JSON message: " << e.what() << " message: '"
           << message.toStdString() << "'";
        sendStatus(client, rbp::StatusLevel::Error, ss.str());
    }
}

void ROSNode::onWSBinaryMessage(const QByteArray& message) const
{
    Q_UNUSED(message)
    ROS_WARN_STREAM("Unhandled binary message received on WS");
}

void ROSNode::onWSClientDisconnected()
{
    const auto* client = qobject_cast<WSClient*>(sender());

    const auto clientName = client->name();
    m_clientErrorMsg = client->errorMsg();

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

    const int64_t max_buffer_size_bytes = m_maxWebSocketBufferSize_MB * 1000 * 1000;
    auto client =
        std::make_shared<WSClient>(socket, max_buffer_size_bytes, 1000, m_pongTimeout_s);
    client->connectSignals();
    connect(client.get(), &WSClient::onWSMessage, this, &ROSNode::onWSMessage);
    connect(client.get(), &WSClient::onWSBinaryMessage, this, &ROSNode::onWSMessage);

    // Disconnection is handled with a QueuedConnection to be delayed later.
    // In case of an abort on the WebSocket (buffer full), the WSClient will be removed
    // will we might be iterating on the list of clients in handleROSMessage() and
    // erasing an element of a vector invalidates the iterators
    connect(client.get(), &WSClient::disconected, this, &ROSNode::onWSClientDisconnected,
            Qt::QueuedConnection);

    m_clients.push_back(client);

    publishStats();
}

void ROSNode::onWSServerError(QWebSocketProtocol::CloseCode error) const
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

        if(const auto it = m_subs.find(topic); it != m_subs.end())
        {
            // Store latched msg for a new client
            if(msg->isLatched())
            {
                it->second.isLatched = true;
                it->second.lastMessage = msg;
                it->second.lastMessageReceivedTime = receivedTime;
            }

            // find used encodings
            bool toJson = false;
            bool toCbor = false;
            bool toCborRaw = false;
            for(const auto& client : it->second.clients)
            {
                switch(client->encoding)
                {
                case rosbridge_protocol::Encoding::JSON: toJson = true; break;
                case rosbridge_protocol::Encoding::CBOR: toCbor = true; break;
                case rosbridge_protocol::Encoding::CBOR_RAW: toCborRaw = true; break;
                default: throw std::runtime_error("unhandled encoding"); break;
                }
            }

            // convert message to used encodings
            QElapsedTimer convertTimer;
            convertTimer.start();
            auto [jsonStr, cborVect, cborRawVect] = encodeMsgToWireFormat(
                *m_fish, receivedTime, topic, msg, toJson, toCbor, toCborRaw);

            ROS_DEBUG_STREAM_NAMED("topic", "Converted ROS msg on topic "
                                                << topic << " to wire format(s) in "
                                                << convertTimer.nsecsElapsed() / 1000
                                                << " us");

            // Warning, the sendTopicToClient() call can trigger an abort operation on the
            // Websocket, and then remove a client while we are looping on the clients
            // Thus, the onWSClientDisconnected() method is connected to a queued
            // connection to delete it later
            for(const auto& client : it->second.clients)
            {
                if(client->client->isReady())
                {
                    if(client->throttleRate_ms == 0 ||
                       ((ros::Time::now() - client->lastTimeMsgSent).toSec() >
                        (client->throttleRate_ms / 1000.)))
                    {
                        sendTopicToClient(client.get(), jsonStr, cborVect, cborRawVect);
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
            for(const auto& client : it->second.clients)
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

        sendMsg(client, ros_nlohmann_converter::dumpJson(json));
    }
}

void ROSNode::sendMsg(WSClient* client, const std::string& msg) const
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send on ws: '" << msg << "'");
    QMetaObject::invokeMethod(client, "sendMsg",
                              Q_ARG(QString, QString::fromStdString(msg)));
}

void ROSNode::sendBinaryMsg(WSClient* client, const std::vector<uint8_t>& binaryMsg) const
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send binary msg on ws");
    const auto data = QByteArray(reinterpret_cast<const char*>(binaryMsg.data()),
                                 static_cast<int>(binaryMsg.size()));
    QMetaObject::invokeMethod(client, "sendBinaryMsg", Q_ARG(QByteArray, data));
}

void ROSNode::sendMsgToClient(WSClient* client, const std::string& jsonStr,
                              const std::vector<uint8_t>& cborVect,
                              const std::vector<uint8_t>& cborRawVect,
                              rosbridge_protocol::Encoding encoding)
{
    if(encoding == rbp::Encoding::CBOR)
    {
        sendBinaryMsg(client, cborVect);
    }
    else if(encoding == rbp::Encoding::CBOR_RAW)
    {
        sendBinaryMsg(client, cborRawVect);
    }
    else
    {
        sendMsg(client, jsonStr);
    }
}

void ROSNode::sendTopicToClient(SubscriberClient* client, const std::string& jsonStr,
                                const std::vector<uint8_t>& cborVect,
                                const std::vector<uint8_t>& cborRawVect)
{
    sendMsgToClient(client->client, jsonStr, cborVect, cborRawVect, client->encoding);
    client->lastTimeMsgSent = ros::Time::now();
}

void ROSNode::addNewSubscriberClient(WSClient* client,
                                     const rosbridge_protocol::SubscribeArgs& args)
{
    ROSBridgeSubscriber sub;
    sub.type = args.type;
    sub.sub = m_nhPrivate.subscribe<ros_babel_fish::BabelFishMessage>(
        args.topic, 100,
        [this,
         topic = args.topic](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) {
            handleROSMessage(topic, msg);
        });

    sub.clients.push_back(std::make_shared<SubscriberClient>(client, args));
    m_subs.emplace(args.topic, std::move(sub));
    ROS_DEBUG_STREAM("Subscribe to a new topic " << args.topic);
}

void ROSNode::publishStats() const
{
    ROS_DEBUG("publishStats");
    {
        std_msgs::Int32 msg;
        msg.data = static_cast<int32_t>(m_clients.size());
        m_clientsCountPub.publish(msg);
    }
    {
        rosbridge_cpp_msgs::WebSocketConnectedClients msg;
        msg.clients.reserve(m_clients.size());
        for(const auto& client : m_clients)
        {
            rosbridge_cpp_msgs::WebSocketConnectedClient c;
            c.ip_address = client->ipAddress();
            c.connection_time = client->connectionTime();
            c.websocket_input_rate_kbytes_sec = client->webSocketInputKBytesSec();
            c.network_output_rate_kbytes_sec = client->networkOutputKBytesSec();
            c.ping_ms = client->pingTime_ms();
            msg.clients.push_back(c);
        }
        m_connectedClientsPub.publish(msg);
    }
}

void ROSNode::produceNetworkDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
    status.add("Connected clients", m_clients.size());
    auto network_output_kbytessec = 0.;
    for(const auto& client : m_clients)
    {
        network_output_kbytessec += client->networkOutputKBytesSec();
    }
    status.add("Network output (KBytes/sec)", network_output_kbytessec);
    if(m_clientErrorMsg.empty())
    {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, m_clientErrorMsg);
    }
    else
    {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, m_clientErrorMsg);
    }
}

void ROSNode::advertiseHandler(WSClient* client, const nlohmann::json& json,
                               const std::string& id)
{
    try
    {
        rbp::AdvertiseArgs args;
        args.id = id;
        args.topic = getMandatoryNotEmptyStringFromJson(json, "topic");
        args.type = getMandatoryNotEmptyStringFromJson(json, "type");

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
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'advertise' msg "s + e.what());
    }
}

void ROSNode::unadvertiseHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    try
    {
        rbp::UnadvertiseArgs args;
        args.id = id;
        args.topic = getMandatoryNotEmptyStringFromJson(json, "topic");
        unadvertise(client, args);
    }
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'unadvertise' msg "s + e.what());
    }
}

void ROSNode::publishHandler(WSClient* client, const nlohmann::json& json,
                             const std::string& id)
{
    try
    {
        rbp::PublishArgs args;
        args.id = id;
        args.topic = getMandatoryNotEmptyStringFromJson(json, "topic");

        if(const auto it = json.find("msg"); it != json.end())
        {
            args.msg = *it;
        }
        else
        {
            throw std::runtime_error("without required 'msg' key");
        }

        publish(client, args);
    }
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'publish' msg "s + e.what());
    }
}

void ROSNode::subscribeHandler(WSClient* client, const nlohmann::json& json,
                               const std::string& id)
{
    try
    {
        rbp::SubscribeArgs args;
        args.id = id;
        args.topic = getMandatoryNotEmptyStringFromJson(json, "topic");

        if(const auto it = json.find("type"); it != json.end())
        {
            args.type = it->get<std::string>();
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
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'subscribe' msg "s + e.what());
    }
}

void ROSNode::unsubscribeHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    try
    {
        rbp::UnsubscribeArgs args;
        args.id = id;
        args.topic = getMandatoryNotEmptyStringFromJson(json, "topic");
        unsubscribe(client, args);
    }
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'unsubscribe' msg "s + e.what());
    }
}

void ROSNode::callServiceHandler(WSClient* client, const nlohmann::json& json,
                                 const std::string& id)
{
    try
    {
        rbp::CallServiceArgs args;
        args.id = id;
        args.serviceName = getMandatoryNotEmptyStringFromJson(json, "service");
        args.serviceType = getMandatoryNotEmptyStringFromJson(json, "type");

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
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'call_service' msg "s + e.what());
    }
}

void ROSNode::setLevelHandler(WSClient* client, const nlohmann::json& json,
                              const std::string& id)
{
    try
    {
        rbp::SetLevelArgs args;
        args.id = id;

        std::string desiredLevel = getMandatoryNotEmptyStringFromJson(json, "level");
        boost::algorithm::to_lower(desiredLevel);
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
            ss << "with unknown status level: '" << desiredLevel
               << "', accepted values are: [";
            for(const auto& [level, str] : rbp::statusLevelStringMap)
            {
                (void)level;
                ss << str << " ";
            }
            ss << "]";
            throw std::runtime_error(ss.str());
        }
        setLevel(client, args);
    }
    catch(const std::runtime_error& e)
    {
        sendStatus(client, rbp::StatusLevel::Error,
                   "Received 'set_level' msg "s + e.what());
    }
}

SubscriberClient::SubscriberClient(WSClient* wsClient, const rbp::SubscribeArgs& args)
    : client{wsClient}, queueSize{args.queueSize}, throttleRate_ms{args.throttleRate},
      fragmentSize{args.fragmentSize}, compression{args.compression},
      encoding{rosbridge_protocol::compressionToEncoding(args.compression)}
{}
