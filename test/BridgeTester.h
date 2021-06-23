#pragma once

#include <QObject>
#include <QtTest/QtTest>

class BridgeTester : public QObject
{
    Q_OBJECT
private slots:
    void canSubscribeToATopicAndSendJson();
    void twoClientsCanSubscribeToTheSameTopic();
    void canSubscribeToATopicWithoutThrottle();
    void canSubscribeToATopicWithThrottleRateBurst();
    void canSubscribeToATopicWithThrottleRate();
    void canSubscribeToATopicAndSendCBOR();
    void canSubscribeThenUnsubscribeToATopic();
    void canPublishOnATopicJSON();
    void canAdvertiseAndUnadvertiseATopic();
    void cannotUnadvertiseATopicNotAdvertised();
    void canCallAServiceJSON();
    void canCallAServiceWithId();
    void canCallAServiceCBOR();
};
