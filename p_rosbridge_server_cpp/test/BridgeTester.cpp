#include <QCoreApplication>
#include <QDebug>

#include <librosqt/QRosCallBackQueue.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <thread>

#include "ROSNode.h"
#include "WSClient.h"
#include "nlohmann/json.hpp"

#include "BridgeTester.h"

constexpr auto PUBLISH_WAIT_TIME_MS = 120;

using namespace std::string_literals;

class MockWSClient : public WSClient
{
    Q_OBJECT
public:
    MockWSClient() : WSClient{nullptr, WSClient::DEFAULT_BUFFER_SIZE_1000MB, 1000, 10.0}
    {}

    void sendMsg(const QString& msg) override { m_lastSentTextMsgs.push_back(msg); }
    void sendBinaryMsg(const QByteArray& msg) override { m_lastSentBinaryMsg = msg; }
    bool isReady() const override { return true; }
    std::string name() const override { return "mock"; }

    void receivedTextMessage(const QString& msg) { emit onWSMessage(msg); }

    std::vector<QString> m_lastSentTextMsgs;
    QByteArray m_lastSentBinaryMsg;
};

void BridgeTester::canSubscribeToATopicAndSendJson()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::twoClientsCanSubscribeToTheSameTopic()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client1 = new MockWSClient();
    auto client2 = new MockWSClient();
    ROSNode node;
    connect(client1, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    connect(client2, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client1->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");
    client2->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");

    QVERIFY(client1->m_lastSentTextMsgs.empty());
    QVERIFY(client2->m_lastSentTextMsgs.empty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client1->m_lastSentTextMsgs.size(), 1UL);
    QCOMPARE(client2->m_lastSentTextMsgs.size(), 1UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto json1Str =
        nlohmann::json::parse(client1->m_lastSentTextMsgs.at(0).toStdString()).dump();
    const auto json2Str =
        nlohmann::json::parse(client2->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(json1Str, expectedJsonStr);
    QCOMPARE(json2Str, expectedJsonStr);
}

void BridgeTester::twoClientsCanSubscribeToTheSameLatchedTopic()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello_latched", 10, true);

    // Create on heap because will call deleteLater in destructor
    auto client1 = new MockWSClient();
    auto client2 = new MockWSClient();
    ROSNode node;
    connect(client1, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    connect(client2, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    // Publish the latched topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    // First client subscribes
    client1->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello_latched","type":"std_msgs/String"})");

    QVERIFY(client1->m_lastSentTextMsgs.empty());
    QVERIFY(client2->m_lastSentTextMsgs.empty());

    QTest::qWait(20);

    QCOMPARE(client1->m_lastSentTextMsgs.size(), 1UL);
    QVERIFY(client2->m_lastSentTextMsgs.empty());

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello_latched","msg":{"data":"hello"}})"_json.dump();

    const auto json1Str =
        nlohmann::json::parse(client1->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(json1Str, expectedJsonStr);

    // Then client 2 subscribes
    client2->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello_latched","type":"std_msgs/String"})");

    QTest::qWait(20);

    // The message is latched and must be received by the second client
    QCOMPARE(client2->m_lastSentTextMsgs.size(), 1UL);

    const auto json2Str =
        nlohmann::json::parse(client2->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(json2Str, expectedJsonStr);
}

void BridgeTester::canSubscribeToATopicWithoutThrottle()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    for(size_t i = 0; i < 20; i++)
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 20UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    for(const auto& msg : client->m_lastSentTextMsgs)
    {
        const auto jsonStr = nlohmann::json::parse(msg.toStdString()).dump();
        QCOMPARE(jsonStr, expectedJsonStr);
    }
}

void BridgeTester::canSubscribeToATopicWithThrottleRateBurst()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String","throttle_rate":20})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    for(size_t i = 0; i < 20; i++)
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    for(const auto& msg : client->m_lastSentTextMsgs)
    {
        const auto jsonStr = nlohmann::json::parse(msg.toStdString()).dump();
        QCOMPARE(jsonStr, expectedJsonStr);
    }
}

void BridgeTester::canSubscribeToATopicWithThrottleRate()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String","throttle_rate":60})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    for(size_t i = 0; i < 4; i++)
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
        QTest::qWait(20);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    // Only 2 should be received
    QCOMPARE(client->m_lastSentTextMsgs.size(), 2UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    for(const auto& msg : client->m_lastSentTextMsgs)
    {
        const auto jsonStr = nlohmann::json::parse(msg.toStdString()).dump();
        QCOMPARE(jsonStr, expectedJsonStr);
    }
}

void BridgeTester::canSubscribeToATopicAndSendCBOR()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String","compression":"cbor"})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(!client->m_lastSentBinaryMsg.isEmpty());

    const auto expectedJson =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json;
    const auto expectedCborData = nlohmann::json::to_cbor(expectedJson);
    QCOMPARE(client->m_lastSentBinaryMsg,
             QByteArray(reinterpret_cast<const char*>(expectedCborData.data()),
                        expectedCborData.size()));
}

void BridgeTester::canSubscribeThenUnsubscribeToATopic()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");

    QVERIFY(client->m_lastSentTextMsgs.empty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);

    // Clear buffer
    client->m_lastSentTextMsgs.clear();
    QVERIFY(client->m_lastSentTextMsgs.empty());

    // Unsubscribe
    client->receivedTextMessage(R"({"op":"unsubscribe","topic":"/hello"})");

    // Publish
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    // We should not have received the message
    QVERIFY(client->m_lastSentTextMsgs.empty());
}

void BridgeTester::canSubscribeToATopicWithoutType()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(R"({"op":"subscribe","topic":"/hello"})");

    // Publish a message on the topic
    std_msgs::String msg;
    msg.data = "hello";
    pub.publish(msg);

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canSubscribeToATopicWithEmptyType()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(R"({"op":"subscribe","topic":"/hello","type":""})");

    // Publish a message on the topic
    std_msgs::String msg;
    msg.data = "hello";
    pub.publish(msg);

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canPublishOnATopicJSON()
{
    ros::NodeHandle nh;
    bool hasReceivedMsg = false;
    std::string receivedMsg;
    ros::Subscriber pub = nh.subscribe<std_msgs::String>(
        "/hello", 1,
        [&hasReceivedMsg, &receivedMsg](const std_msgs::StringConstPtr& msg) {
            receivedMsg = msg->data;
            hasReceivedMsg = true;
        });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"advertise","topic":"/hello","type":"std_msgs/String"})");

    client->receivedTextMessage(
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})");

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QVERIFY(hasReceivedMsg);
    QCOMPARE(receivedMsg, std::string{"hello"});
}

void BridgeTester::canAdvertiseAndUnadvertiseATopic()
{
    ros::NodeHandle nh;
    bool hasReceivedMsg = false;
    std::string receivedMsg;
    ros::Subscriber pub = nh.subscribe<std_msgs::String>(
        "/hello", 1,
        [&hasReceivedMsg, &receivedMsg](const std_msgs::StringConstPtr& msg) {
            receivedMsg = msg->data;
            hasReceivedMsg = true;
        });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"advertise","topic":"/hello","type":"std_msgs/String"})");

    QTest::qWait(20);

    {
        // Check that the topic is advertised using the rosmaster API
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        bool foundTopic = false;
        for(const auto& topicInfo : topics)
        {
            if(topicInfo.name == "/hello" && topicInfo.datatype == "std_msgs/String")
            {
                foundTopic = true;
                break;
            }
        }
        QVERIFY(foundTopic);
    }

    // Publish a topic
    client->receivedTextMessage(
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})");

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QVERIFY(hasReceivedMsg);
    QCOMPARE(receivedMsg, std::string{"hello"});

    hasReceivedMsg = false;
    receivedMsg.clear();

    // Unadvertise
    client->receivedTextMessage(R"({"op":"unadvertise","topic":"/hello"})");

    QTest::qWait(60);

    // Publish a topic
    client->receivedTextMessage(
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})");

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QVERIFY(!hasReceivedMsg);
    QVERIFY(receivedMsg.empty());
}

void BridgeTester::cannotUnadvertiseATopicNotAdvertised()
{
    ros::NodeHandle nh;
    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    // Unadvertise
    client->receivedTextMessage(R"({"op":"unadvertise","topic":"/hello"})");

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    const auto jsonRes =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString());
    QCOMPARE(jsonRes["op"].get<std::string>(), std::string{"status"});
    QCOMPARE(jsonRes["level"].get<std::string>(), std::string{"error"});
}

void BridgeTester::canChangeStatusLevel()
{
    ros::NodeHandle nh;
    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    // Set status level to none
    client->receivedTextMessage(R"({"op":"set_level","level":"none"})");

    // Unadvertise
    client->receivedTextMessage(R"({"op":"unadvertise","topic":"/hello"})");

    // No status message sent
    QVERIFY(client->m_lastSentTextMsgs.empty());
}

void BridgeTester::canPublishOnALatchedTopicAndSubscribeLater()
{
    ros::NodeHandle nh;

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Advertise a latched topic
    client->receivedTextMessage(
        R"({"op":"advertise","topic":"/hello","type":"std_msgs/String","latch":true})");

    // Publish a msg
    client->receivedTextMessage(
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})");

    QTest::qWait(10);

    // Create a subscriber
    bool hasReceivedMsg = false;
    std::string receivedMsg;
    ros::Subscriber pub = nh.subscribe<std_msgs::String>(
        "/hello", 1,
        [&hasReceivedMsg, &receivedMsg](const std_msgs::StringConstPtr& msg) {
            receivedMsg = msg->data;
            hasReceivedMsg = true;
        });

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    // Message should be received because topic is latched
    QVERIFY(hasReceivedMsg);
    QCOMPARE(receivedMsg, std::string{"hello"});
}

void BridgeTester::cannotPublishOnANotLatchedTopicAndSubscribeLater()
{
    ros::NodeHandle nh;

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Advertise a latched topic
    client->receivedTextMessage(
        R"({"op":"advertise","topic":"/hello","type":"std_msgs/String","latch":false})");

    // Publish a msg
    client->receivedTextMessage(
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})");

    QTest::qWait(10);

    // Create a subscriber
    bool hasReceivedMsg = false;
    std::string receivedMsg;
    ros::Subscriber pub = nh.subscribe<std_msgs::String>(
        "/hello", 1,
        [&hasReceivedMsg, &receivedMsg](const std_msgs::StringConstPtr& msg) {
            receivedMsg = msg->data;
            hasReceivedMsg = true;
        });

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    // Message should not be received because subscribed after publish on a topic not
    // latched
    QVERIFY(!hasReceivedMsg);
}

void BridgeTester::canCallAServiceJSON()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = true;
                res.message = "ok";
                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true}})");

    QTest::qWait(40);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"service_response","service":"/set_bool","values":{"success":true,"message":"ok"},"result":true})"_json
            .dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canCallAServiceJSONWithError()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = false;
                return false;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true}})");

    QTest::qWait(40);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 2UL);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"service_response","service":"/set_bool","values":["Failed to call service /set_bool"],"result":false})"_json
            .dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();

    // std::cout << "res : " << client->m_lastSentTextMsgs.at(0).toStdString() <<
    // std::endl;

    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canCallAServiceJSONWithTimeout()
{
    ros::CallbackQueue async_cb_queue;
    ros::AsyncSpinner ros_async_spinner(1, &async_cb_queue);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&async_cb_queue);
    ros_async_spinner.start();

    std::atomic<bool> serviceHasBeenCalled = false;
    std::atomic<bool> serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = false;

                usleep(1000000); // wait 1 sec

                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    node.SetServiceCallTimeout(0.100);
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true}})");

    QTest::qWait(40);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // make sure we received the timeout
    QTest::qWait(100);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"service_response","service":"/set_bool","values":["Service /set_bool call timeout"],"result":false})"_json
            .dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);

    // if nothing has crashed, shutdown properly
    ros_async_spinner.stop();
}

void BridgeTester::canCallAServiceJSONWithErrorAfterDisconnect()
{
    ros::CallbackQueue async_cb_queue;
    ros::AsyncSpinner ros_async_spinner(1, &async_cb_queue);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&async_cb_queue);
    ros_async_spinner.start();

    std::atomic<bool> serviceHasBeenCalled = false;
    std::atomic<bool> serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = false;

                usleep(100000); // wait 100 msecs

                return false;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    node.SetServiceCallTimeout(5.0);
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true}})");

    QTest::qWait(40);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // destroy client
    client->deleteLater();
    QTest::qWait(100);

    // if nothing has crashed, shutdown properly
    ros_async_spinner.stop();
}

void BridgeTester::canCallAServiceWithDisconnectDuringTimeout()
{
    ros::CallbackQueue async_cb_queue;
    ros::AsyncSpinner ros_async_spinner(1, &async_cb_queue);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&async_cb_queue);
    ros_async_spinner.start();

    std::atomic<bool> serviceHasBeenCalled = false;
    std::atomic<bool> serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = false;

                usleep(1000000); // wait 1 sec

                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    node.SetServiceCallTimeout(0.100);
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true}})");

    QTest::qWait(40);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // destroy client
    client->deleteLater();
    QTest::qWait(200);

    // if nothing has crashed, shutdown properly
    ros_async_spinner.stop();
}

void BridgeTester::canCallAServiceWithId()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = true;
                res.message = "ok";
                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());

    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true},"id":"my_id"})");

    QTest::qWait(40);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"service_response","service":"/set_bool","values":{"success":true,"message":"ok"},"result":true,"id":"my_id"})"_json
            .dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canCallAServiceCBOR()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = true;
                res.message = "ok";
                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"data":true},"compression":"cbor"})");

    QTest::qWait(40);

    QVERIFY(!client->m_lastSentBinaryMsg.isEmpty());

    QVERIFY(serviceHasBeenCalled);
    QVERIFY(serviceReq);

    // Encode and decode JSON to get comparable strings
    const auto expectedJson =
        R"({"op":"service_response","service":"/set_bool","values":{"success":true,"message":"ok"},"result":true})"_json;
    const auto expectedCborData = nlohmann::json::to_cbor(expectedJson);
    QCOMPARE(client->m_lastSentBinaryMsg,
             QByteArray(reinterpret_cast<const char*>(expectedCborData.data()),
                        expectedCborData.size()));
}

void BridgeTester::cannotCallAServiceWithBadJSON()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = true;
                res.message = "ok";
                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());

    // Faulty JSON, {true} instead of {"data": true}
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{true}})");

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 1UL);

    QVERIFY(!serviceHasBeenCalled);
    QVERIFY(!serviceReq);

    const auto status =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString());
    QCOMPARE(status["op"].get<std::string>(), "status"s);
}

void BridgeTester::cannotCallAServiceWithJSONBadKey()
{
    ros::NodeHandle nh;
    bool serviceHasBeenCalled = false;
    bool serviceReq = false;
    ros::ServiceServer service =
        nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            "/set_bool",
            [&serviceHasBeenCalled, &serviceReq](std_srvs::SetBoolRequest& req,
                                                 std_srvs::SetBoolResponse& res) {
                serviceHasBeenCalled = true;
                serviceReq = req.data;
                res.success = true;
                res.message = "ok";
                return true;
            });

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);

    QVERIFY(client->m_lastSentTextMsgs.empty());

    // Faulty JSON, "toto" field instead of "data"
    client->receivedTextMessage(
        R"({"op":"call_service","service":"/set_bool","type":"std_srvs/SetBool","args":{"toto": true}})");

    QTest::qWait(PUBLISH_WAIT_TIME_MS);

    QCOMPARE(client->m_lastSentTextMsgs.size(), 2UL);

    QVERIFY(!serviceHasBeenCalled);
    QVERIFY(!serviceReq);

    const auto status =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(0).toStdString());
    QCOMPARE(status["op"].get<std::string>(), "status"s);

    const auto response =
        nlohmann::json::parse(client->m_lastSentTextMsgs.at(1).toStdString());
    QCOMPARE(response["op"].get<std::string>(), "service_response"s);
    QCOMPARE(response["result"].get<bool>(), false);
}

/*!
 * Catkin ne gere que les tests unitaires avec gtest en C++
 * Or ici, nous utilisons QTest
 * Pour generer un fichier xml de resultat qui sera lu par l'integration continue,
 * nous devons passer des arguments a l'executable des tests unitaires.
 * De plus, la commande `catkin run_tests` ne lance que les tests declares avec
 * `catkin_add_gtest` dans le CMakeLists.txt.
 * Le probleme ici c'est que catkin va donc appeler les executables de tests avec
 * l'argument : `--gtest_output=xml:<chemin_du_fichier_xml>`
 * Or QTest ne comprend pas cet argument.
 *
 * A noter que si le dossier de resultats n'existe pas, Gtest le cree et non QTest.
 * Il y a donc un test si le dossier existe et le cree le cas echeant.
 */
int main(int argc, char* argv[])
{
    // Manipulate args to mimic gtest
    std::vector<std::string> newStringArgs;
    newStringArgs.reserve(argc);
    std::copy(argv, argv + argc, std::back_inserter(newStringArgs));
    const std::string gtestArg{"--gtest_output=xml:"};
    auto it = std::find_if(newStringArgs.begin(), newStringArgs.end(),
                           [&gtestArg](const auto& elem) {
                               return elem.find(gtestArg) != std::string::npos;
                           });
    if(it != newStringArgs.end())
    {
        const std::string path = it->substr(gtestArg.length());
        newStringArgs.erase(it);

        QFileInfo outputFile{QString::fromStdString(path)};
        if(!QDir().exists(outputFile.absolutePath()))
        {
            QDir().mkdir(outputFile.absolutePath());
        }
        // Select both XML file and stdout outputs
        newStringArgs.push_back("-o");
        newStringArgs.push_back(path + ",xunitxml");
        newStringArgs.push_back("-o");
        newStringArgs.push_back("-,txt");
    }
    std::vector<char*> newArgv;
    newArgv.reserve(newStringArgs.size());
    for(const auto& arg : newStringArgs)
    {
        newArgv.push_back(const_cast<char*>(arg.c_str()));
    }
    int newArgc = static_cast<int>(newArgv.size());

    QCoreApplication app(newArgc, newArgv.data());

    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(newArgc, newArgv.data(), "bridge_tester",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

    BridgeTester tester;

    return QTest::qExec(&tester, newArgc, newArgv.data());
}

#include "BridgeTester.moc"
