#include <QWebSocket>

#include <ros/console.h>

#include "WSClient.h"

WSClient::WSClient(QWebSocket* ws)
    : QObject(nullptr), m_ws{ws}, m_connectionTime{ros::Time::now()}
{}

WSClient::~WSClient()
{
    ROS_DEBUG_STREAM("~WSClient " << name());
    m_ws->deleteLater();
}

void WSClient::connectSignals()
{
    connect(m_ws, &QWebSocket::textMessageReceived, this, &WSClient::onWSMessage);
    connect(m_ws, &QWebSocket::binaryMessageReceived, this, &WSClient::onWSBinaryMessage);
    connect(m_ws, &QWebSocket::disconnected, this, &WSClient::onWSDisconnected);

    connect(m_ws, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error),
            [this](QAbstractSocket::SocketError error) {
                ROS_ERROR_STREAM("WS error (" << static_cast<int>(error) << "): "
                                              << m_ws->errorString().toStdString());
            });

    connect(m_ws, &QWebSocket::pong, this,
            [](quint64 elapsedTime, const QByteArray& payload) {
                Q_UNUSED(payload)
                ROS_DEBUG_STREAM_NAMED("websocket",
                                       "Pong received with round-trip time of "
                                           << elapsedTime << " ms");
            });

    connect(m_ws, &QWebSocket::stateChanged, this,
            [](QAbstractSocket::SocketState state) {
                ROS_DEBUG_STREAM_NAMED("websocket",
                                       "State changed to: " << static_cast<int>(state));
            });

    connect(&m_pingTimer, &QTimer::timeout, this, [this]() { m_ws->ping(); });
    m_pingTimer.start(2000);
}

std::string WSClient::ipAddress() const
{
    return m_ws->peerAddress().toString().toStdString();
}

ros::Time WSClient::connectionTime() const
{
    return m_connectionTime;
}

std::string WSClient::name() const
{
    return m_ws->peerAddress().toString().toStdString() + ":" +
           std::to_string(m_ws->peerPort());
}

bool WSClient::isReady() const
{
    return m_ws->isValid();
}

void WSClient::onWSDisconnected()
{
    ROS_DEBUG_STREAM_NAMED("websocket", "Client Disconnected");
    auto* client = qobject_cast<QWebSocket*>(sender());
    if((client != nullptr) && client == m_ws)
    {
        client->deleteLater();
        emit disconected();
    }
}

void WSClient::sendMsg(const QString& msg)
{
    m_ws->sendTextMessage(msg);
}

void WSClient::sendBinaryMsg(const QByteArray &binaryMsg)
{
    m_ws->sendBinaryMessage(binaryMsg);
}
