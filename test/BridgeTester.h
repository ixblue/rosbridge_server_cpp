#pragma once

#include <QObject>
#include <QtTest/QtTest>

class BridgeTester : public QObject
{
    Q_OBJECT
private slots:
    void canSubscribeToATopicAndSendJson();
    void twoClientsCanSubscribeToTheSameTopic();
    void twoClientsCanSubscribeToTheSameLatchedTopic();
    void canSubscribeToATopicWithoutThrottle();
    void canSubscribeToATopicWithThrottleRateBurst();
    void canSubscribeToATopicWithThrottleRate();
    void canSubscribeToATopicAndSendCBOR();
    void canSubscribeThenUnsubscribeToATopic();
    void canSubscribeToATopicWithoutType();
    void canSubscribeToATopicWithEmptyType();
    void canPublishOnATopicJSON();
    void canPublishOnALatchedTopicAndSubscribeLater();
    void cannotPublishOnANotLatchedTopicAndSubscribeLater();
    void canAdvertiseAndUnadvertiseATopic();
    void cannotUnadvertiseATopicNotAdvertised();
    void canChangeStatusLevel();
    void canCallAServiceJSON();
    void canCallAServiceWithId();
    void canCallAServiceCBOR();
    void cannotCallAServiceWithBadJSON();
    void cannotCallAServiceWithJSONBadKey();
};
