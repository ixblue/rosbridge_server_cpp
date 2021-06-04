#include <QWebSocket>

#include "rapidjson/writer.h"

#include "ROSNode.h"
#include "WSClient.h"

WSClient::WSClient(QWebSocket* ws, ROSNode* rosNode)
    : QObject(nullptr), m_ws{ws}, m_node{rosNode},
      m_opHandlers{
          {"advertise",
           [this](const auto& j, const auto& id) { advertiseHandler(j, id); }},
          {"unadvertise",
           [this](const auto& j, const auto& id) { unadvertiseHandler(j, id); }},
          {"publish", [this](const auto& j, const auto& id) { publishHandler(j, id); }},
          {"subscribe",
           [this](const auto& j, const auto& id) { subscribeHandler(j, id); }},
          {"unsubscribe",
           [this](const auto& j, const auto& id) { unsubscribeHandler(j, id); }},
          {"call_service",
           [this](const auto& j, const auto& id) { callServiceHandler(j, id); }}}
{
    connect(m_ws, &QWebSocket::textMessageReceived, this, &WSClient::onWSMessageReceived);
    connect(m_ws, &QWebSocket::disconnected, this, &WSClient::onWsDisconnected);

    // connect(this, &WSClient::publish, rosNode, &ROSNode::publish);
}

void WSClient::onWSMessageReceived(const QString& message)
{
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

    QString id;
    if(const auto it = doc.FindMember("id"); it != doc.MemberEnd())
    {
        id = it->value.GetString();
    }

    const QString op{doc["op"].GetString()};
    if(const auto it = m_opHandlers.find(op); it != m_opHandlers.end())
    {
        it->second(doc, id);
    }
    else
    {
        ROS_ERROR_STREAM("Received unkown OP '" << op.toStdString() << "' ignoring");
    }
}

void WSClient::onWsDisconnected()
{
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    if(client && client == m_ws)
    {
        // m_clients.removeAll(pClient);
        client->deleteLater();
        emit disconected();
    }
}

void WSClient::sendStatus(rosbridge_protocol::StatusLevel severity, const QString& id,
                          const QString& msg)
{
    rapidjson::Document jsonDoc;
    jsonDoc.SetObject();
    jsonDoc.AddMember("op", "status", jsonDoc.GetAllocator());
    if(!id.isEmpty())
    {
        const auto idBA = id.toUtf8();
        jsonDoc.AddMember("id",
                          rapidjson::Value::StringRefType(idBA.constData(), idBA.size()),
                          jsonDoc.GetAllocator());
    }
    jsonDoc.AddMember("level", rosbridge_protocol::statusLevelStringMap.at(severity),
                      jsonDoc.GetAllocator());
    const auto msgBA = msg.toUtf8();
    jsonDoc.AddMember("msg",
                      rapidjson::Value::StringRefType(msgBA.constData(), msgBA.size()),
                      jsonDoc.GetAllocator());
    sendJson(jsonDoc);
}

void WSClient::sendMsg(const QString& msg)
{
    m_ws->sendTextMessage(msg);
}

void WSClient::advertiseHandler(const rapidjson::Value& json, const QString& id)
{
    QString topic;
    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'advertise' msg without required 'topic' key");
        return;
    }

    QString type;
    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        type = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'advertise' msg without required 'type' key");
        return;
    }

    bool latched = false;
    if(const auto it = json.FindMember("latch"); it != json.MemberEnd())
    {
        latched = it->value.GetBool();
    }

    int queueSize = 10;
    if(const auto it = json.FindMember("queue_size"); it != json.MemberEnd())
    {
        queueSize = it->value.GetInt();
    }

    emit advertise(id, topic, type, queueSize, latched);
}

void WSClient::unadvertiseHandler(const rapidjson::Value& json, const QString& id)
{
    QString topic;
    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }

    emit unadvertise(id, topic);
}

void WSClient::publishHandler(const rapidjson::Value& json, const QString& id)
{
    QString topic;
    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }

    if(const auto it = json.FindMember("msg"); it != json.MemberEnd())
    {
        emit publish(id, topic, it->value);
    }
    else
    {
        ROS_WARN_STREAM("Received 'unadvertise' msg without required 'topic' key");
        return;
    }
}

void WSClient::subscribeHandler(const rapidjson::Value& json, const QString& id)
{
    QString topic;
    if(const auto it = json.FindMember("topic"); it != json.MemberEnd())
    {
        topic = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'subscribe' msg without required 'topic' key");
        return;
    }

    QString type;
    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        type = it->value.GetString();
    }

    int throttleRate = false;
    if(const auto it = json.FindMember("throttle_rate"); it != json.MemberEnd())
    {
        throttleRate = it->value.GetInt();
    }

    int queueSize = 1;
    if(const auto it = json.FindMember("queue_size"); it != json.MemberEnd())
    {
        queueSize = it->value.GetInt();
    }

    int fragmentSize = -1;
    if(const auto it = json.FindMember("fragment_size"); it != json.MemberEnd())
    {
        fragmentSize = it->value.GetInt();
    }

    QString compression;
    if(const auto it = json.FindMember("compression"); it != json.MemberEnd())
    {
        compression = it->value.GetString();
    }

    emit subscribe(id, topic, type, queueSize, throttleRate, fragmentSize, compression);
}

void WSClient::unsubscribeHandler(const rapidjson::Value& json, const QString& id) {}

void WSClient::callServiceHandler(const rapidjson::Value& json, const QString& id)
{
    QString serviceName;
    if(const auto it = json.FindMember("service"); it != json.MemberEnd())
    {
        serviceName = it->value.GetString();
    }
    else
    {
        ROS_WARN_STREAM("Received 'call_service' msg without required 'service' key");
        return;
    }

    QString serviceType;
    if(const auto it = json.FindMember("type"); it != json.MemberEnd())
    {
        serviceType = it->value.GetString();
    }

    int fragmentSize = -1;
    if(const auto it = json.FindMember("fragment_size"); it != json.MemberEnd())
    {
        fragmentSize = it->value.GetInt();
    }

    QString compression;
    if(const auto it = json.FindMember("compression"); it != json.MemberEnd())
    {
        compression = it->value.GetString();
    }

    if(const auto it = json.FindMember("args"); it != json.MemberEnd())
    {
        emit callService(id, serviceName, serviceType, it->value, fragmentSize,
                         compression);
    }
    else
    {
        rapidjson::Value args;
        emit callService(id, serviceName, serviceType, args, fragmentSize, compression);
    }
}

void WSClient::sendJson(const rapidjson::Document& doc)
{
    rapidjson::StringBuffer jsonBuffer;
    rapidjson::Writer<rapidjson::StringBuffer, rapidjson::UTF8<>, rapidjson::UTF8<>,
                      rapidjson::CrtAllocator,
                      rapidjson::kWriteDefaultFlags | rapidjson::kWriteNanAndInfFlag>
        jsonWriter(jsonBuffer);
    doc.Accept(jsonWriter);
    m_ws->sendTextMessage(jsonBuffer.GetString());
}
