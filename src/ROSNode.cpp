#include <sstream>

#include <QElapsedTimer>
#include <QMetaObject>
#include <QWebSocket>

#include "nlohmann/json.hpp"

#include "rapidjson/writer.h"

#include "ROSNode.h"
#include "ServiceCallerWithTimeout.h"
#include "WSClient.h"
#include "rapidjson_to_ros.h"
#include "ros_to_nlohmann.h"
#include "ros_to_rapidjson.h"

namespace rbp = rosbridge_protocol;

ROSNode::ROSNode(QObject* parent)
    : QObject(parent), m_nh{"~"}, m_wsServer{"rosbridge server",
                                             QWebSocketServer::NonSecureMode},
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
          {"call_service", [this](WSClient* c, const auto& j, const auto& id) {
               callServiceHandler(c, j, id);
           }}}
{
    // Parameters
    m_nh.getParam("port", m_wsPort);
    m_nh.getParam("service_timeout", m_serviceTimeout);

    connect(&m_wsServer, &QWebSocketServer::newConnection, this,
            &ROSNode::onNewWSConnection);
    connect(&m_wsServer, &QWebSocketServer::serverError, this, &ROSNode::onWSServerError);

    m_fish = std::make_shared<ros_babel_fish::BabelFish>();
}

ROSNode::~ROSNode()
{
    ROS_DEBUG_STREAM("~ROSNode");
}

void ROSNode::start()
{
    if(!m_wsServer.listen(QHostAddress::Any, m_wsPort))
    {
        ROS_FATAL_STREAM("Failed to start WS server on port "
                         << m_wsPort << ": " << m_wsServer.errorString().toStdString());
        exit(1);
    }

    ROS_INFO_STREAM("Start WS on port: " << m_wsServer.serverPort());
    m_nh.setParam("actual_port", m_wsServer.serverPort());
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
        m_pubs.emplace(args.topic,
                       ROSBridgePublisher{args.type,
                                          m_fish->advertise(m_nh, args.type, args.topic,
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

void ROSNode::publish(WSClient* client, const rbp::PublishArgs& args,
                      const rapidjson::Value& msg)
{
    if(const auto it = m_pubs.find(args.topic); it != m_pubs.end())
    {
        try
        {
            ros_babel_fish::BabelFishMessage::Ptr rosMsg =
                ros_rapidjson_converter::createMsg(*m_fish, it->second.type,
                                                   ros::Time::now(), msg);

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

void ROSNode::subscribe(WSClient* client, const rbp::SubscribeArgs& args)
{
    auto getNewClient = [=]() {
        auto newSubClient = std::make_shared<SubscriberClient>();
        newSubClient->client = client;
        newSubClient->queueSize = args.queueSize;
        newSubClient->throttleRate_ms = args.throttleRate;
        newSubClient->fragmentSize = args.fragmentSize;
        newSubClient->compression = args.compression;
        // Default = JSON, PNG not implemented
        if(args.compression == "cbor")
        {
            newSubClient->encoding = rbp::Encoding::CBOR;
        }
        else if(args.compression == "cbor-raw")
        {
            newSubClient->encoding = rbp::Encoding::CBOR_RAW;
        }
        else
        {
            newSubClient->encoding = rbp::Encoding::JSON;
        }
        m_encodings.insert(newSubClient->encoding);
        return newSubClient;
    };

    if(const auto it = m_subs.find(args.topic); it != m_subs.end())
    {
        // topic already subscribed, check if this client already registered
        if(const auto clientIt = std::find_if(
               it->second.clients.begin(), it->second.clients.end(),
               [&client](const auto& clientSub) { return clientSub->client == client; });
           clientIt != it->second.clients.end())
        {
            ROS_DEBUG_STREAM("Client is subscribing once more on the " << args.topic);

            SubscriberClient& c = *(*clientIt);
            c.throttleRate_ms = std::min(c.throttleRate_ms, args.throttleRate);
            c.fragmentSize = std::min(c.fragmentSize, args.fragmentSize);
            c.queueSize = std::min(c.queueSize, args.queueSize);
        }
        else
        {
            ROS_DEBUG_STREAM("Add a new client subscribed on topic " << args.topic);
            it->second.clients.push_back(getNewClient());
        }
    }
    else
    {
        if(!args.type.empty())
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
        }

        ROSBridgeSubscriber sub;
        sub.type = args.type;
        sub.sub = m_nh.subscribe<ros_babel_fish::BabelFishMessage>(
            args.topic, 100,
            [this,
             topic = args.topic](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) {
                handleROSMessage(topic, msg);
            });

        sub.clients.push_back(getNewClient());

        m_subs.emplace(args.topic, std::move(sub));
        ROS_DEBUG_STREAM("Subscribe to a new topic " << args.topic);
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

void ROSNode::callService(WSClient* client, const rbp::CallServiceArgs& args,
                          const rapidjson::Value& msg)
{
    ROS_INFO_STREAM_NAMED("service", "Call service " << args.serviceName);
    try
    {
        ros_babel_fish::Message::Ptr req = m_fish->createServiceRequest(args.serviceType);
        auto& compound = req->as<ros_babel_fish::CompoundMessage>();
        ros_rapidjson_converter::fillMessageFromJson(msg, compound);

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

                        /*
                        rapidjson::Document jsonDoc;
                        jsonDoc.SetObject();
                        jsonDoc.AddMember("op", "service_response",
                                          jsonDoc.GetAllocator());
                        if(!id.empty())
                        {
                            jsonDoc.AddMember("id", id, jsonDoc.GetAllocator());
                        }
                        jsonDoc.AddMember("service", serviceName, jsonDoc.GetAllocator());
                        rapidjson::Value jsonMsg;
                        ros_rapidjson_converter::translatedMsgtoJson(
                            *res->translated_message, jsonMsg, jsonDoc.GetAllocator());
                        jsonDoc.AddMember("values", jsonMsg, jsonDoc.GetAllocator());
                        jsonDoc.AddMember("result", true, jsonDoc.GetAllocator());
                        sendJson(client, jsonDoc);
                        */

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
                [this, client, serviceName = args.serviceName](const QString& errorMsg) {
                    if(client != nullptr)
                    {
                        // TODO send status error or service_response?
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
        return;
    }
}

void ROSNode::onWSMessage(const QString& message)
{
    ROS_DEBUG_STREAM_NAMED("json",
                           "<- Received on ws: '" << message.toStdString() << "'");

    auto* client = qobject_cast<WSClient*>(sender());

    rapidjson::Document doc;
    doc.Parse(message.toUtf8());
    if(doc.HasParseError())
    {
        ROS_WARN_STREAM("Error "
                        << doc.GetParseError()
                        << " while parsing received json: " << message.toStdString());
        return;
    }

    if(!doc.IsObject() || !doc.HasMember("op"))
    {
        ROS_WARN_STREAM(
            "Received JSON is not a rosbridge message (not object or no 'op' field): "
            << message.toStdString());
        return;
    }

    std::string id;
    if(const auto it = doc.FindMember("id"); it != doc.MemberEnd())
    {
        id = it->value.GetString();
    }

    const std::string op{doc["op"].GetString()};
    if(const auto it = m_opHandlers.find(op); it != m_opHandlers.end())
    {
        it->second(client, doc, id);
    }
    else
    {
        ROS_ERROR_STREAM("Received unkown OP '" << op << "' ignoring");
    }
}

void ROSNode::onWSBinaryMessage(const QByteArray& message)
{
    Q_UNUSED(message)
    ROS_WARN_STREAM("Unhandled binary message received on WS");
}

void ROSNode::onWSClientDisconnected()
{
    auto* client = qobject_cast<WSClient*>(sender());

    ROS_INFO_STREAM("Client " << client->name() << " disconnected (" << m_clients.size()
                              << " client)");

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

    ROS_DEBUG_STREAM("Removed, now (" << m_clients.size() << " client)");
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
        ROS_DEBUG_STREAM_NAMED("topic", "Handle ROS msg on topic " << topic);
        QElapsedTimer t;
        t.start();
        /*
            rapidjson::Document jsonDoc;
            jsonDoc.SetObject();
            jsonDoc.AddMember("op", "publish", jsonDoc.GetAllocator());
            jsonDoc.AddMember("topic", topic, jsonDoc.GetAllocator());
            rapidjson::Value jsonMsg;
            ros_rapidjson_converter::toJson(*m_fish, *msg, jsonMsg,
           jsonDoc.GetAllocator()); jsonDoc.AddMember("msg", jsonMsg,
           jsonDoc.GetAllocator());

            ROS_DEBUG_STREAM_NAMED("topic", "Converted message on topic "
                                                << topic << " to rapidjson::Document in "
                                                << t.nsecsElapsed() / 1000 << " us");
            t.start();
            // Encode to json only once
            const std::string jsonStr = ros_rapidjson_converter::jsonToString(jsonDoc);
        */

        nlohmann::json json{{"op", "publish"}, {"topic", topic}};
        auto msgJson = ros_nlohmann_converter::toJson(*m_fish, *msg);
        ROS_DEBUG_STREAM_NAMED("topic", "Converted ROS message on topic "
                                            << topic << " to JSON struct in "
                                            << t.nsecsElapsed() / 1000 << " us");
        t.start();

        // Encode to json only once
        std::string jsonStr;
        std::vector<uint8_t> cborVect;
        const bool hasJson = m_encodings.count(rbp::Encoding::JSON) > 0;
        const bool hasCbor = m_encodings.count(rbp::Encoding::CBOR) > 0;
        if(hasJson || hasCbor)
        {
            auto j = json;
            j["msg"] = msgJson;
            if(hasCbor)
            {
                cborVect = nlohmann::json::to_cbor(j);
            }
            if(hasJson)
            {
                jsonStr = j.dump();
            }
        }

        std::vector<uint8_t> cborRawVect;
        if(m_encodings.count(rbp::Encoding::CBOR_RAW) > 0)
        {
            auto j = json;
            std::vector<uint8_t> data(msg->buffer(), msg->buffer() + msg->size());
            nlohmann::json::binary_t jsonBin{data};
            j["msg"] = {{"secs", receivedTime.sec},
                        {"nsecs", receivedTime.nsec},
                        {"bytes", jsonBin}};
            cborRawVect = nlohmann::json::to_cbor(j);
        }

        ROS_DEBUG_STREAM_NAMED("topic", "Converted JSON struct on topic "
                                            << topic << " to wire format(s) in "
                                            << t.nsecsElapsed() / 1000 << " us");

        if(const auto it = m_subs.find(topic); it != m_subs.end())
        {
            for(auto& client : it->second.clients)
            {
                if(client->client->isReady())
                {
                    if(client->throttleRate_ms == 0 ||
                       (ros::Time::now() - client->lastTimeMsgSent).toSec() >
                           client->throttleRate_ms / 1000.)
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
    switch(level)
    {
    case rbp::StatusLevel::Info: ROS_INFO_STREAM(msg); break;
    case rbp::StatusLevel::Warning: ROS_WARN_STREAM(msg); break;
    case rbp::StatusLevel::Error: ROS_ERROR_STREAM(msg); break;
    case rbp::StatusLevel::None: ROS_DEBUG_STREAM(msg); break;
    }

    rapidjson::Document jsonDoc;
    jsonDoc.SetObject();
    jsonDoc.AddMember("op", "status", jsonDoc.GetAllocator());
    if(!id.empty())
    {
        jsonDoc.AddMember("id", id, jsonDoc.GetAllocator());
    }
    jsonDoc.AddMember("level", rbp::statusLevelStringMap.at(level),
                      jsonDoc.GetAllocator());
    jsonDoc.AddMember("msg", msg, jsonDoc.GetAllocator());

    sendJson(client, jsonDoc);
}

void ROSNode::sendMsg(WSClient* client, const std::string& msg)
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send on ws: '" << msg << "'");
    QMetaObject::invokeMethod(client, "sendMsg",
                              Q_ARG(QString, QString::fromStdString(msg)));
}

void ROSNode::sendJson(WSClient* client, const rapidjson::Document& doc)
{
    sendMsg(client, ros_rapidjson_converter::jsonToString(doc));
}

void ROSNode::sendBinaryMsg(WSClient* client, const std::vector<uint8_t>& binaryMsg)
{
    ROS_DEBUG_STREAM_NAMED("json", "-> Send binary msg on ws");
    auto data =
        QByteArray(reinterpret_cast<const char*>(binaryMsg.data()), binaryMsg.size());
    QMetaObject::invokeMethod(client, "sendBinaryMsg", Q_ARG(QByteArray, data));
}

void ROSNode::advertiseHandler(WSClient* client, const rapidjson::Value& json,
                               const std::string& id)
{
    rbp::AdvertiseArgs args;
    args.id = id;

    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        args.topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'advertise' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        args.type = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'advertise' msg without required 'type' key");
        return;
    }

    if(const auto it = json.FindMember("latch"); it != json.MemberEnd())
    {
        args.latched = it->value.GetBool();
    }

    if(const auto it = json.FindMember("queue_size"); it != json.MemberEnd())
    {
        args.queueSize = it->value.GetInt();
    }

    advertise(client, args);
}

void ROSNode::unadvertiseHandler(WSClient* client, const rapidjson::Value& json,
                                 const std::string& id)
{
    rbp::UnadvertiseArgs args;
    args.id = id;

    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        args.topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }

    unadvertise(client, args);
}

void ROSNode::publishHandler(WSClient* client, const rapidjson::Value& json,
                             const std::string& id)
{
    rbp::PublishArgs args;
    args.id = id;

    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        args.topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.FindMember("msg"); it != json.MemberEnd())
    {
        // To do deep-copy, use CopyFrom but require allocator, so a Document
        publish(client, args, it->value);
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }
}

void ROSNode::subscribeHandler(WSClient* client, const rapidjson::Value& json,
                               const std::string& id)
{
    rbp::SubscribeArgs args;
    args.id = id;

    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        args.topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'subscribe' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        args.type = it->value.GetString();
    }

    if(const auto it = json.FindMember("throttle_rate"); it != json.MemberEnd())
    {
        args.throttleRate = it->value.GetInt();
    }

    if(const auto it = json.FindMember("queue_size"); it != json.MemberEnd())
    {
        args.queueSize = it->value.GetInt();
    }

    if(const auto it = json.FindMember("fragment_size"); it != json.MemberEnd())
    {
        args.fragmentSize = it->value.GetInt();
    }

    if(const auto it = json.FindMember("compression"); it != json.MemberEnd())
    {
        args.compression = it->value.GetString();
    }

    subscribe(client, args);
}

void ROSNode::unsubscribeHandler(WSClient* client, const rapidjson::Value& json,
                                 const std::string& id)
{
    rbp::UnsubscribeArgs args;
    args.id = id;

    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        args.topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'unsubscribe' msg without required 'topic' key");
        return;
    }

    unsubscribe(client, args);
}

void ROSNode::callServiceHandler(WSClient* client, const rapidjson::Value& json,
                                 const std::string& id)
{
    rbp::CallServiceArgs args;
    args.id = id;

    if(const auto it = json.FindMember("service"); it != json.MemberEnd())
    {
        args.serviceName = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'call_service' msg without required 'service' key");
        return;
    }

    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        args.serviceType = it->value.GetString();
    }

    if(const auto it = json.FindMember("fragment_size"); it != json.MemberEnd())
    {
        args.fragmentSize = it->value.GetInt();
    }

    if(const auto it = json.FindMember("compression"); it != json.MemberEnd())
    {
        args.compression = it->value.GetString();
    }

    if(const auto it = json.FindMember("args"); it != json.MemberEnd())
    {
        callService(client, args, it->value);
    }
    else
    {
        rapidjson::Value msg;
        callService(client, args, msg);
    }
}
