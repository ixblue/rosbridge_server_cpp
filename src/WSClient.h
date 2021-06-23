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
    virtual ~WSClient();
    void connectSignals();

    virtual std::string name() const;
    virtual bool isReady() const;

public slots:
    void onWSDisconnected();
    virtual void sendMsg(const QString& msg);
    virtual void sendBinaryMsg(const QByteArray& binaryMsg);

signals:
    void onWSMessage(const QString& msg);
    void onWSBinaryMessage(const QByteArray& msg);
    void disconected();

private:
    QWebSocket* m_ws = nullptr;
    QTimer m_pingTimer;
};
