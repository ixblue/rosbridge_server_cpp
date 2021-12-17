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

    explicit WSClient(QWebSocket* ws, int64_t max_socket_buffer_size_bytes);
    virtual ~WSClient();
    void connectSignals();

    std::string ipAddress() const;
    ros::Time connectionTime() const;
    virtual std::string name() const;
    virtual bool isReady() const;

    static constexpr int64_t BUFFER_SIZE_1000MB = 1E9;

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
    // Bytes contained in QWebSocket internal buffer waiting to be written
    // on the system socket to be effectively sent on the network.
    // If too much data is sent compared to network capacity, the internal buffer
    // could become huge and take too much RAM on the system.
    // This number can be accessed with the QIODevice::bytesToWrite() function but the QWebSocket
    // interface doesn't expose it directly: we compute and update it on our own everytime
    // we send data on the QWebSocket and everytime the data is removed from the internal buffer
    // See onWSBytesWritten()
    int64_t m_socketBytesToWrite = 0;
    // If the internal buffer goes over max, it means more data than a client is able to receive
    // is being sent and the connection will aborted to avoid consumming too much RAM.
    // This can be due to a disconnection / network freeze (TCP timeout will take a long time to happen) or
    // a client too slow to process the data or network congestion (bandwidth too low / packet loss)
    int64_t m_maxSocketBufferSize = BUFFER_SIZE_1000MB;

};
