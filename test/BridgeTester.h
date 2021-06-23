#pragma once

#include <QObject>
#include <QtTest/QtTest>

class BridgeTester : public QObject
{
    Q_OBJECT
private slots:
    void canSubscribeToATopicAndSendJson();
    void canSubscribeToATopicAndSendCBOR();
};
