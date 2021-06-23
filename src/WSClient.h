#pragma once

#include <vector>

#include <QObject>
#include <QString>
#include <QTimer>

/*!
 * Very simple wrapper on top of QWebsocket
 */

class QWebSocket;

class WSClient : public QObject
{
    Q_OBJECT
public:
    explicit WSClient(QWebSocket* ws);
    ~WSClient();
    void connectSignals();

    std::string name() const;
    bool isReady() const;

public slots:
    void onWSDisconnected();
    void sendMsg(const QString& msg);
    void sendBinaryMsg(const QByteArray& binaryMsg);

signals:
    void onWSMessage(const QString& msg);
    void onWSBinaryMessage(const QByteArray& msg);
    void disconected();

private:
    QWebSocket* m_ws = nullptr;
    QTimer m_pingTimer;
};
