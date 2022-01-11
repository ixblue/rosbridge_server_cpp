#include <QWebSocket>

#include <ros/console.h>

#include "WSClient.h"

WSClient::WSClient(QWebSocket* ws, int64_t max_socket_buffer_size_bytes,
                   int transferRateUpdatePeriod_ms)
    : QObject(nullptr), m_ws{ws}, m_connectionTime{ros::Time::now()},
      m_maxSocketBufferSize{max_socket_buffer_size_bytes},
      m_transferRateUpdatePeriod_ms{transferRateUpdatePeriod_ms}
{
    m_transferRateTimer.setInterval(m_transferRateUpdatePeriod_ms);
    m_transferRateTimer.setSingleShot(false);
    connect(&m_transferRateTimer, &QTimer::timeout, this, &WSClient::computeTransferRates);
    m_transferRateTimer.start();
}

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
    return m_ws->peerAddress().toString().toStdString() + ":" +
           std::to_string(m_ws->peerPort());
}

bool WSClient::isReady() const
{
    return m_ws->isValid();
}

std::string WSClient::errorMsg() const
{
    return m_errorMsg;
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
    ROS_ERROR_STREAM("Internal websocket buffer is full for client "
                     << ipAddress() << ". Connection aborted");
    // Cannot call disconnect because it requires the current buffer data to be fully sent
    // before trying to gently terminate the TCP connection. It needs to be stopped
    // immediatly to stop filling the buffer. This will trigger a disconnected signal.
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
