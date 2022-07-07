#pragma once

#include <vector>

#include <QObject>
#include <QString>
#include <QTimer>

#include <ros/time.h>

/*!
 * Very simple wrapper on top of QWebsocket
 */

class QWebSocket;

class WSClient : public QObject
{
    Q_OBJECT
public:
    explicit WSClient(QWebSocket* ws, int64_t max_socket_buffer_size_bytes,
                      int transferRateUpdatePeriod_ms = 1000);
    virtual ~WSClient();
    void connectSignals();

    std::string ipAddress() const;
    ros::Time connectionTime() const;
    // Amount of data to be sent to the websocket client
    float webSocketInputKBytesSec() const;
    // Approximated Amount of data actually sent out on the TCP layer (the system
    // socket also has a buffer so writting on it doesn't exacly mean transmission as long
    // as it's not full)
    float networkOutputKBytesSec() const;
    qint64 pingTime_ms() const;

    virtual std::string name() const;
    virtual bool isReady() const;
    // Returns an error msg if something wrong happened or an empty string otherwise.
    std::string errorMsg() const;

    static constexpr int64_t DEFAULT_BUFFER_SIZE_1000MB = 1'000'000'000;

public slots:
    void onWSDisconnected();
    void onWSBytesWritten(qint64 bytes);
    virtual void sendMsg(const QString& msg);
    virtual void sendBinaryMsg(const QByteArray& binaryMsg);

signals:
    void onWSMessage(const QString& msg);
    void onWSBinaryMessage(const QByteArray& msg);
    void disconected();

private:
    void abortConnection();

    QWebSocket* m_ws = nullptr;
    QTimer m_pingTimer;
    ros::Time m_connectionTime;
    std::string m_name = "Not yet connected";

#if QT_VERSION < QT_VERSION_CHECK(5, 12, 0)
    // Bytes contained in QWebSocket internal buffer waiting to be written
    // on the system socket to be effectively sent on the network.
    // If too much data is sent compared to network capacity, the internal buffer
    // could become huge and take too much RAM on the system.
    // This number can be accessed with the QIODevice::bytesToWrite() function but the
    // QWebSocket interface doesn't expose it directly: we compute and update it on our
    // own everytime we send data on the QWebSocket and everytime the data is removed from
    // the internal buffer See onWSBytesWritten()

    // Starting with Qt5.12, can be accessed directly with bytesToWrite()
    int64_t m_socketBytesToWrite = 0;
#endif
    // If the internal buffer goes over max, it means more data than a client is able to
    // receive is being sent and the connection will aborted to avoid consumming too much
    // RAM. This can be due to a disconnection / network freeze (TCP timeout will take a
    // long time to happen) or a client too slow to process the data or network congestion
    // (bandwidth too low / packet loss)
    int64_t m_maxSocketBufferSize = DEFAULT_BUFFER_SIZE_1000MB;
    std::string m_errorMsg;

    // Monitor bytes coming in and out of the socket and compute an averaged speed
    QTimer m_transferRateTimer;
    int m_transferRateUpdatePeriod_ms;
    int64_t m_webSocketInputBytes = 0;
    float m_webSocketInputRateKBytesSec = 0;
    int64_t m_networkOutputBytes = 0;
    float m_networkOutputRateKBytesSec = 0;
    qint64 m_lastPingTime_ms = 0;
    void computeTransferRates();
};
