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
    void canAdvertiseAndPublishOnATopicInCBOR();
    void canPublishOnALatchedTopicAndSubscribeLater();
    void cannotPublishOnANotLatchedTopicAndSubscribeLater();
    void canAdvertiseAndUnadvertiseATopic();
    void cannotUnadvertiseATopicNotAdvertised();
    void canChangeStatusLevel();
    void canCallAServiceJSON();
    void canCallAServiceJSONWithBase64();
    void canCallAServiceJSONWithError();
    void canCallAServiceJSONWithTimeout();
    void canCallAServiceWithDisconnectDuringTimeout();
    void canCallAServiceJSONWithErrorAfterDisconnect();
    void canCallAServiceWithId();
    void canCallAServiceWithACBORResponse();
    void canCallAServiceWithACBORRequestAndResponse();
    void canCallAServiceCBORWithBinary();
    void cannotCallAServiceWithBadJSON();
    void cannotCallAServiceWithJSONBadKey();
};
