#include <cassert>

#include <QWebSocket>

#include <ros/console.h>

#include "WSClient.h"

WSClient::WSClient(QWebSocket* ws, const int64_t max_socket_buffer_size_bytes,
                   const int transferRateUpdatePeriod_ms, const double pong_timeout_seconds,
                   const bool requireAuth)
    : QObject(nullptr), m_ws{ws}, m_connectionTime{ros::Time::now()},
      m_maxSocketBufferSize{max_socket_buffer_size_bytes},
      m_transferRateUpdatePeriod_ms{transferRateUpdatePeriod_ms},
      m_pongReceiveTimeout{pong_timeout_seconds},
      m_requireAuth{requireAuth}
{
    m_transferRateTimer.setInterval(m_transferRateUpdatePeriod_ms);
    m_transferRateTimer.setSingleShot(false);
    m_lastPongReceivedStamp = ros::SteadyTime::now();
}

WSClient::~WSClient()
{
    if(m_ws != nullptr)
    {
        m_ws->deleteLater();
    }
}

void WSClient::connectSignals()
{
    m_name = m_ws->peerAddress().toString().toStdString() + ":" +
             std::to_string(m_ws->peerPort());

    connect(m_ws, &QWebSocket::textMessageReceived, this, &WSClient::onWSMessage);
    connect(m_ws, &QWebSocket::binaryMessageReceived, this, &WSClient::onWSBinaryMessage);
    connect(m_ws, &QWebSocket::disconnected, this, &WSClient::onWSDisconnected);
    connect(m_ws, &QWebSocket::bytesWritten, this, &WSClient::onWSBytesWritten);

    connect(m_ws, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error),
            [this](QAbstractSocket::SocketError error) {
                ROS_ERROR_STREAM("WS error (" << static_cast<int>(error) << "): "
                                              << m_ws->errorString().toStdString());
            });

    connect(m_ws, &QWebSocket::pong, this,
            [this](quint64 elapsedTime, const QByteArray& payload) {
                Q_UNUSED(payload)
                m_lastPingTime_ms = elapsedTime;
                m_lastPongReceivedStamp = ros::SteadyTime::now();
                ROS_DEBUG_STREAM_NAMED("websocket",
                                       "Pong received with round-trip time of "
                                           << elapsedTime << " ms");
            });

    connect(m_ws, &QWebSocket::stateChanged, this,
            [](QAbstractSocket::SocketState state) {
                ROS_DEBUG_STREAM_NAMED("websocket",
                                       "State changed to: " << static_cast<int>(state));
            });

    connect(&m_transferRateTimer, &QTimer::timeout, this,
            &WSClient::computeTransferRates);
    m_transferRateTimer.start();

    connect(&m_pingTimer, &QTimer::timeout, this, [this]() { onPingTimer(); });
    m_pingTimer.start(2000);

    QTimer::singleShot(15000, this, &WSClient::closeIfNotAuthenticated);
}

void WSClient::onPingTimer()
{
    assert(m_ws != nullptr);
    m_ws->ping();

    ros::WallDuration durationSincePong =
        ros::SteadyTime::now() - m_lastPongReceivedStamp;

    if(durationSincePong > m_pongReceiveTimeout)
    {
        ROS_ERROR_STREAM("Websocket client " << ipAddress()
                                             << " timed out. Connection aborted");
        // Cannot call disconnect because it requires the current buffer data to be fully
        // sent before trying to gently terminate the TCP connection. It needs to be
        // stopped immediatly to stop filling the buffer. This will trigger a disconnected
        // signal.
        m_errorMsg = "Connection to client " + ipAddress() + " timed out.";
        m_ws->abort();
    }
}

void WSClient::computeTransferRates()
{
    static constexpr auto MS_TO_SECS = 1 / 1000.;
    const auto input_kbytes = m_webSocketInputBytes / 1000.;
    m_webSocketInputRateKBytesSec =
        input_kbytes / (m_transferRateUpdatePeriod_ms * MS_TO_SECS);
    // Reset counter for next update
    m_webSocketInputBytes = 0;
    const auto output_kbytes = m_networkOutputBytes / 1000.;
    m_networkOutputRateKBytesSec =
        output_kbytes / (m_transferRateUpdatePeriod_ms * MS_TO_SECS);
    // Reset counter for next update
    m_networkOutputBytes = 0;
}

std::string WSClient::ipAddress() const
{
    assert(m_ws != nullptr);
    return m_ws->peerAddress().toString().toStdString();
}

ros::Time WSClient::connectionTime() const
{
    return m_connectionTime;
}

float WSClient::webSocketInputKBytesSec() const
{
    return m_webSocketInputRateKBytesSec;
}

float WSClient::networkOutputKBytesSec() const
{
    return m_networkOutputRateKBytesSec;
}

std::string WSClient::name() const
{
    return m_name;
}

bool WSClient::isReady() const
{
    if(m_ws != nullptr)
    {
        return m_ws->isValid();
    }
    return false;
}

std::string WSClient::errorMsg() const
{
    return m_errorMsg;
}

bool WSClient::isAuthenticated() const
{
    return m_authenticated;
}

void WSClient::setAuthenticated(const bool authenticated)
{
    m_authenticated = authenticated;
}

void WSClient::closeIfNotAuthenticated() const
{
    if(!isAuthenticated())
    {
        ROS_WARN_STREAM_NAMED("security", "Client " << ipAddress() << " failed to authenticate, closing socket");
        m_ws->close(QWebSocketProtocol::CloseCodeAbnormalDisconnection, "Failed to authenticate");
    }
}

void WSClient::onWSDisconnected()
{
    ROS_DEBUG_STREAM_NAMED("websocket", "Client Disconnected");
    auto* client = qobject_cast<QWebSocket*>(sender());
    if((client != nullptr) && client == m_ws)
    {
        emit disconnected();
    }
}

void WSClient::onWSBytesWritten(qint64 bytes)
{
    // Called everytime bytes were removed from the internal QWebSocket buffer
    // to be written on the system socket.
    auto correctBytes = bytes;
#if QT_VERSION < QT_VERSION_CHECK(5, 12, 0)
    // For some unknown Qt reasons (bug), the bytesWritten signal is emitted twice so
    // we need to divide by 2 to have the correct number.
    correctBytes = bytes / 2;
    m_socketBytesToWrite = std::max(static_cast<qint64>(0), m_socketBytesToWrite - correctBytes);
#endif
    m_networkOutputBytes += correctBytes;
}

void WSClient::abortConnection()
{
    assert(m_ws != nullptr);
    ROS_ERROR_STREAM("Internal websocket buffer is full for client "
                     << ipAddress() << ". Connection aborted");
    // Cannot call disconnect because it requires the current buffer data to be fully sent
    // before trying to gently terminate the TCP connection. It needs to be stopped
    // immediately to stop filling the buffer. This will trigger a disconnected signal.
    m_errorMsg = "Connection to client " + ipAddress() +
                 " has been aborted due to network performance issues.";
    m_ws->abort();
}

qint64 WSClient::pingTime_ms() const
{
    return m_lastPingTime_ms;
}

void WSClient::sendMsg(const QString& msg)
{
    assert(m_ws != nullptr);
    auto bytesInBuffer = m_ws->sendTextMessage(msg);
    m_webSocketInputBytes += bytesInBuffer;
#if QT_VERSION < QT_VERSION_CHECK(5, 12, 0)
    m_socketBytesToWrite += bytesInBuffer;
    if(m_socketBytesToWrite > m_maxSocketBufferSize)
#else
    if(m_ws->bytesToWrite() > m_maxSocketBufferSize)
#endif
    {
        abortConnection();
    }
}

void WSClient::sendBinaryMsg(const QByteArray& binaryMsg)
{
    assert(m_ws != nullptr);
    auto bytesInBuffer = m_ws->sendBinaryMessage(binaryMsg);
    m_webSocketInputBytes += bytesInBuffer;

#if QT_VERSION < QT_VERSION_CHECK(5, 12, 0)
    m_socketBytesToWrite += bytesInBuffer;
    if(m_socketBytesToWrite > m_maxSocketBufferSize)
#else
    if(m_ws->bytesToWrite() > m_maxSocketBufferSize)
#endif
    {
        abortConnection();
    }
}
