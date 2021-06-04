#include <sstream>

#include <QMetaObject>

#include "rapidjson/writer.h"

#include "ROSNode.h"
#include "WSClient.h"
#include "rapidjson_to_ros.h"
#include "ros_to_rapidjson.h"

ROSNode::ROSNode(QObject* parent)
    : QObject(parent), m_wsServer{"rosbridge server", QWebSocketServer::NonSecureMode}
{
    int port = 9090;
    m_nh.getParam("port", port);

    if(!m_wsServer.listen(QHostAddress::Any, port))
    {
        ROS_FATAL_STREAM("Failed to start WS server on port "
                         << port << ": " << m_wsServer.errorString().toStdString());
        exit(1);
    }

    connect(&m_wsServer, &QWebSocketServer::newConnection, this,
            &ROSNode::onNewWSConnection);
}

void ROSNode::advertise(const QString& id, const QString& topic, const QString& type,
                        unsigned int queueSize, bool latched)
{
    WSClient* client = qobject_cast<WSClient*>(this->sender());

    if(const auto it = m_pubs.find(topic); it == m_pubs.end())
    {
        try
        {
            const auto dummyMsg = m_fish.createMessage(type.toStdString());
        }
        catch(const ros_babel_fish::BabelFishException&)
        {
            std::ostringstream ss;
            ss << "Unknown message type '" << type.toStdString() << "'";
            ROS_ERROR_STREAM(ss.str());
            if(client)
            {
                QMetaObject::invokeMethod(
                    client, "sendStatus", Q_ARG(QString, "error"), Q_ARG(QString, id),
                    Q_ARG(QString, QString::fromStdString(ss.str())));
            }
        }

        m_pubs.emplace(
            topic,
            ROSBridgePublisher{type,
                               m_fish.advertise(m_nh, type.toStdString(),
                                                topic.toStdString(), queueSize, latched),
                               {client}});
    }
    else
    {
        if(topic != it->second.type)
        {
            ROS_ERROR_STREAM("Trying to advertise topic '"
                             << topic.toStdString() << "' with type '"
                             << type.toStdString()
                             << "' but this topic is already advertised with type '"
                             << it->second.type.toStdString() << "'");
            return;
        }

        it->second.clients.insert(client);
    }
}

void ROSNode::unadvertise(const QString& id, const QString& topic)
{
    WSClient* client = qobject_cast<WSClient*>(sender());
    if(const auto it = m_pubs.find(topic); it != m_pubs.end())
    {
        it->second.clients.erase(client);
        if(it->second.clients.empty())
        {
            m_pubs.erase(it);
        }
    }
    else
    {
        std::ostringstream ss;
        ss << "unadvertise cmd on a topic not advertised: '" << topic.toStdString()
           << "'";
        ROS_ERROR_STREAM(ss.str());
        if(client)
        {
            QMetaObject::invokeMethod(client, "sendStatus", Q_ARG(QString, "warning"),
                                      Q_ARG(QString, id),
                                      Q_ARG(QString, QString::fromStdString(ss.str())));
        }
    }
}

void ROSNode::publish(const QString& id, const QString& topic,
                      const rapidjson::Value& msg)
{
    WSClient* client = qobject_cast<WSClient*>(sender());
    if(const auto it = m_pubs.find(topic); it != m_pubs.end())
    {
        try
        {
            ros_babel_fish::BabelFishMessage::Ptr rosMsg =
                ros_rapidjson_converter::createMsg(m_fish, it->second.type.toStdString(),
                                                   ros::Time::now(), msg);
            it->second.pub.publish(rosMsg);
        }
        catch(const ros_babel_fish::BabelFishException& e)
        {
            std::ostringstream ss;
            ss << "publish cmd on topic with bad type: '" << topic.toStdString()
               << "': " << e.what();
            ROS_ERROR_STREAM(ss.str());
            if(client)
            {
                QMetaObject::invokeMethod(
                    client, "sendStatus", Q_ARG(QString, "error"), Q_ARG(QString, id),
                    Q_ARG(QString, QString::fromStdString(ss.str())));
            }
        }
    }
    else
    {
        std::ostringstream ss;
        ss << "publish cmd on topic not advertised: '" << topic.toStdString() << "'";
        ROS_ERROR_STREAM(ss.str());
        if(client)
        {
            QMetaObject::invokeMethod(client, "sendStatus", Q_ARG(QString, "error"),
                                      Q_ARG(QString, id),
                                      Q_ARG(QString, QString::fromStdString(ss.str())));
        }
    }
}

void ROSNode::subscribe(const QString& id, const QString& topic, const QString& type,
                        unsigned int queueSize, int throttleRate, int fragmentSize,
                        const QString& compression)
{
    WSClient* client = qobject_cast<WSClient*>(sender());

    auto getNewClient = [=]() {
        auto newSubClient = std::make_shared<SubscriberClient>();
        newSubClient->client = client;
        newSubClient->queueSize = queueSize;
        newSubClient->throttleRate_ms = throttleRate;
        newSubClient->fragmentSize = fragmentSize;
        newSubClient->compression = compression;
        return newSubClient;
    };

    if(const auto it = m_subs.find(topic); it != m_subs.end())
    {
        // topic already subscribed, check if this client already registered
        if(const auto clientIt = std::find_if(
               it->second.clients.begin(), it->second.clients.end(),
               [&client](const auto& clientSub) { return clientSub->client == client; });
           clientIt != it->second.clients.end())
        {
            SubscriberClient& c = *(*clientIt);
            c.throttleRate_ms = std::min(c.throttleRate_ms, throttleRate);
            c.fragmentSize = std::min(c.fragmentSize, fragmentSize);
            c.queueSize = std::min(c.queueSize, queueSize);
        }
        else
        {
            it->second.clients.push_back(getNewClient());
        }
    }
    else
    {
        if(!topic.isEmpty())
        {
            try
            {
                const auto dummyMsg = m_fish.createMessage(type.toStdString());
            }
            catch(const ros_babel_fish::BabelFishException&)
            {
                std::ostringstream ss;
                ss << "Unknown message type '" << type.toStdString() << "'";
                ROS_ERROR_STREAM(ss.str());
                if(client)
                {
                    QMetaObject::invokeMethod(
                        client, "sendStatus", Q_ARG(QString, "error"), Q_ARG(QString, id),
                        Q_ARG(QString, QString::fromStdString(ss.str())));
                }
            }
        }

        ROSBridgeSubscriber sub;
        sub.sub = m_nh.subscribe<ros_babel_fish::BabelFishMessage>(
            topic.toStdString(), 100,
            [this, topic](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) {
                handleMessage(topic, msg);
            });

        sub.clients.push_back(getNewClient());

        m_subs.emplace(topic, std::move(sub));
    }
}

void ROSNode::unsubscribe(const QString& id, const QString& topic) {}

void ROSNode::callService(const QString& id, const QString& serviceName,
                          const QString& serviceType, const rapidjson::Value& args,
                          int fragmentSize, const QString& compression)
{
    WSClient* client = qobject_cast<WSClient*>(sender());

    ros_babel_fish::Message::Ptr req =
        m_fish.createServiceRequest(serviceType.toStdString());
    auto& compound = req->as<ros_babel_fish::CompoundMessage>();
    ros_rapidjson_converter::fillMessageFromJson(args, compound);
    ros_babel_fish::TranslatedMessage::Ptr res;
    if(m_fish.callService(serviceName.toStdString(), req, res))
    {
        res->translated_message->as<ros_babel_fish::CompoundMessage>();

        rapidjson::Document jsonDoc;
        jsonDoc.SetObject();
        jsonDoc.AddMember("op", "service_response", jsonDoc.GetAllocator());
        if(!id.isEmpty())
        {
            const auto idBA = id.toUtf8();
            jsonDoc.AddMember(
                "id", rapidjson::Value::StringRefType(idBA.constData(), idBA.size()),
                jsonDoc.GetAllocator());
        }
        const QByteArray serviceBA = serviceName.toUtf8();
        jsonDoc.AddMember(
            "service",
            rapidjson::Value::StringRefType(serviceBA.constData(), serviceBA.size()),
            jsonDoc.GetAllocator());
        rapidjson::Value jsonMsg;
        ros_rapidjson_converter::translatedMsgtoJson(*res->translated_message, jsonMsg,
                                                     jsonDoc.GetAllocator());
        jsonDoc.AddMember("values", jsonMsg, jsonDoc.GetAllocator());
        jsonDoc.AddMember("result", true, jsonDoc.GetAllocator());

        rapidjson::StringBuffer jsonBuffer;
        rapidjson::Writer<rapidjson::StringBuffer, rapidjson::UTF8<>, rapidjson::UTF8<>,
                          rapidjson::CrtAllocator,
                          rapidjson::kWriteDefaultFlags | rapidjson::kWriteNanAndInfFlag>
            jsonWriter(jsonBuffer);
        jsonDoc.Accept(jsonWriter);

        QMetaObject::invokeMethod(client, "sendMsg",
                                  Q_ARG(QString, jsonBuffer.GetString()));
    }
}

void ROSNode::onNewWSConnection()
{
    auto socket = m_wsServer.nextPendingConnection();
    ROS_INFO_STREAM("New client connected!");

    auto client = std::make_shared<WSClient>(socket, this);
    connect(client.get(), &WSClient::advertise, this, &ROSNode::advertise);
    connect(client.get(), &WSClient::unadvertise, this, &ROSNode::unadvertise);
    connect(client.get(), &WSClient::publish, this, &ROSNode::publish);
    connect(client.get(), &WSClient::subscribe, this, &ROSNode::subscribe);
    connect(client.get(), &WSClient::unsubscribe, this, &ROSNode::unsubscribe);
    connect(client.get(), &WSClient::callService, this, &ROSNode::callService);
    m_clients.push_back(client);
}

void ROSNode::handleMessage(const QString& topic,
                            const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
    if(const auto it = m_subs.find(topic); it != m_subs.end())
    {
        for(auto& client : it->second.clients)
        {
            if(client->throttleRate_ms == 0 ||
               (ros::Time::now() - client->lastTimeMsgSent).toSec() >
                   client->throttleRate_ms / 1000.)
            {
                rapidjson::Document jsonDoc;
                jsonDoc.SetObject();
                jsonDoc.AddMember("op", "publish", jsonDoc.GetAllocator());
                const QByteArray topicBA = topic.toUtf8();
                jsonDoc.AddMember(
                    "topic",
                    rapidjson::Value::StringRefType(topicBA.constData(), topicBA.size()),
                    jsonDoc.GetAllocator());
                rapidjson::Value jsonMsg;
                ros_rapidjson_converter::toJson(m_fish, *msg, jsonMsg,
                                                jsonDoc.GetAllocator());
                jsonDoc.AddMember("msg", jsonMsg, jsonDoc.GetAllocator());
                rapidjson::StringBuffer jsonBuffer;
                rapidjson::Writer<rapidjson::StringBuffer, rapidjson::UTF8<>,
                                  rapidjson::UTF8<>, rapidjson::CrtAllocator,
                                  rapidjson::kWriteDefaultFlags |
                                      rapidjson::kWriteNanAndInfFlag>
                    jsonWriter(jsonBuffer);
                jsonDoc.Accept(jsonWriter);

                QMetaObject::invokeMethod(client->client, "sendMsg",
                                          Q_ARG(QString, jsonBuffer.GetString()));
                client->lastTimeMsgSent = ros::Time::now();
            }
        }
    }
}
