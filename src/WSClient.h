#pragma once

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
    std::string name() const;
    bool isReady() const;

public slots:
    void onWSDisconnected();
    void sendMsg(const QString& msg);

signals:
    void onWSMessage(const QString& msg);
    void disconected();

private:
    QWebSocket* m_ws = nullptr;
    QTimer m_pingTimer;
};
