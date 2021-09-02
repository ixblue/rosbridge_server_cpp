#include <algorithm>
#include <limits>

#include <gtest/gtest.h>

#include <ros_babel_fish/babel_fish.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "nlohmann_to_ros.h"
#include "ros_to_nlohmann.h"

static const ros::Time g_rosTime{34325437, 432427};

// Helper functions to fill ROS msgs

void fillMessage(geometry_msgs::Pose& m)
{
    m.position.x = 1.0;
    m.position.y = 2.0;
    m.position.z = 3.0;
    m.orientation.x = 1.0;
    m.orientation.y = 2.0;
    m.orientation.z = 3.0;
    m.orientation.w = 4.0;
}

void fillMessage(geometry_msgs::PoseStamped& m)
{
    m.header.stamp.sec = 34325435;
    m.header.stamp.nsec = 432423;
    m.header.frame_id = "robot";
    fillMessage(m.pose);
}

void fillMessage(geometry_msgs::PoseWithCovariance& m)
{
    fillMessage(m.pose);
    std::iota(m.covariance.begin(), m.covariance.end(), 0);
}

void fillMessage(geometry_msgs::Vector3& m)
{
    m.x = 1.0;
    m.y = 2.0;
    m.z = 3.0;
}

void fillMessage(geometry_msgs::Twist& m)
{
    fillMessage(m.linear);
    fillMessage(m.angular);
}

void fillMessage(geometry_msgs::TwistWithCovariance& m)
{
    fillMessage(m.twist);
    std::iota(m.covariance.begin(), m.covariance.end(), 0);
}

void fillMessage(nav_msgs::Odometry& m)
{
    m.header.stamp.sec = 34325435;
    m.header.stamp.nsec = 432423;
    m.header.frame_id = "robot";
    m.header.seq = 456;
    m.child_frame_id = "child";
    fillMessage(m.pose);
    fillMessage(m.twist);
}

void fillMessage(sensor_msgs::NavSatFix& m)
{
    m.header.stamp.sec = 1000;
    m.header.stamp.nsec = 10000;
    m.header.frame_id = "frame_id";
    m.header.seq = 123;
    m.latitude = 123.456;
    m.longitude = 654.321;
    m.altitude = 100.101;
    m.position_covariance[0] = 1.1;
    m.position_covariance[4] = 2.2;
    m.position_covariance[8] = 3.3;
    m.position_covariance_type = 2;
    m.status.status = 1;
    m.status.service = 4;
}

void fillMessage(diagnostic_msgs::DiagnosticArray& m)
{
    m.header.stamp.sec = 123;
    m.header.stamp.nsec = 456;
    m.header.frame_id = "frame_id";

    diagnostic_msgs::DiagnosticStatus status1;
    status1.level = 2; // ERROR
    status1.name = "status1";
    status1.message = "message1";
    {
        diagnostic_msgs::KeyValue val1;
        val1.key = "key1";
        val1.value = "value1";
        status1.values.push_back(val1);
        diagnostic_msgs::KeyValue val2;
        val2.key = "key2";
        val2.value = "value2";
        status1.values.push_back(val2);
        diagnostic_msgs::KeyValue val3;
        val3.key = "key3";
        val3.value = "value3";
        status1.values.push_back(val3);
    }
    m.status.push_back(status1);

    diagnostic_msgs::DiagnosticStatus status2;
    status2.level = 1; // WARN;
    status2.name = "status2";
    status2.message = "message2";
    {
        diagnostic_msgs::KeyValue val1;
        val1.key = "key1";
        val1.value = "value11";
        status2.values.push_back(val1);
        diagnostic_msgs::KeyValue val2;
        val2.key = "key2";
        val2.value = "value22";
        status2.values.push_back(val2);
        diagnostic_msgs::KeyValue val3;
        val3.key = "key3";
        val3.value = "value33";
        status2.values.push_back(val3);
    }
    m.status.push_back(status2);
}

void fillMessage(sensor_msgs::Image& m)
{
    m.header.stamp.sec = 123;
    m.header.stamp.nsec = 456;
    m.header.frame_id = "frame_id";
    m.header.seq = 123;

    m.width = 2;
    m.height = 3;
    m.encoding = "bgr8";
    m.is_bigendian = 0;
    m.step = m.width * 3;
    m.data.resize(m.width * m.height * 3);
    std::iota(m.data.begin(), m.data.end(), 0);
}

void fillMessage(sensor_msgs::CompressedImage& m)
{
    m.header.stamp.sec = 123;
    m.header.stamp.nsec = 456;
    m.header.frame_id = "frame_id";
    m.header.seq = 123;

    m.format = "jpeg";
    m.data.resize(10);
    std::iota(m.data.begin(), m.data.end(), 0);
}

void fillMessage(sensor_msgs::ChannelFloat32& m)
{
    m.name = "channel name";
    m.values.resize(5);
    std::iota(m.values.begin(), m.values.end(), 0);
}

template<typename T>
ros_babel_fish::BabelFishMessage serializeMessage(ros_babel_fish::BabelFish& fish,
                                                  const T& msg)
{
    const std::string& datatype = ros::message_traits::DataType<T>::value();
    const std::string& definition = ros::message_traits::Definition<T>::value();
    const std::string& md5 = ros::message_traits::MD5Sum<T>::value();

    // Create serialized version of the message
    ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage(msg);

    ros_babel_fish::BabelFishMessage bfMsg;
    bfMsg.morph(md5, datatype, definition);

    fish.descriptionProvider()->getMessageDescription(bfMsg);
    ros::serialization::deserializeMessage(serialized_msg, bfMsg);
    return bfMsg;
}

// Interface to test several implementation
template<typename T> class JSONParser
{
public:
    virtual ros_babel_fish::BabelFishMessage::Ptr
    createMsgFromJson(ros_babel_fish::BabelFish& fish, const std::string& type,
                      const ros::Time& time, const std::string& jsonStr) = 0;

    virtual ros_babel_fish::BabelFishMessage::Ptr
    createServiceRequestFromJson(ros_babel_fish::BabelFish& fish, const std::string& type,
                                 const std::string& jsonStr) = 0;

    virtual std::string toJsonString(ros_babel_fish::BabelFish& fish,
                                     const ros_babel_fish::BabelFishMessage& msg) = 0;

    virtual std::string parseAndStringify(const std::string& jsonStr) = 0;
};

class NlohmannJSONParser : public JSONParser<nlohmann::json>
{
public:
    ros_babel_fish::BabelFishMessage::Ptr
    createMsgFromJson(ros_babel_fish::BabelFish& fish, const std::string& type,
                      const ros::Time& time, const std::string& jsonStr) override
    {
        auto json = nlohmann::json::parse(jsonStr);
        return ros_nlohmann_converter::createMsg(fish, type, time, json);
    }

    ros_babel_fish::BabelFishMessage::Ptr
    createServiceRequestFromJson(ros_babel_fish::BabelFish& fish, const std::string& type,
                                 const std::string& jsonStr) override
    {
        auto json = nlohmann::json::parse(jsonStr);
        ros_babel_fish::Message::Ptr req = fish.createServiceRequest(type);
        auto& compound = req->as<ros_babel_fish::CompoundMessage>();
        ros_nlohmann_converter::fillMessageFromJson(json, compound);
        return fish.translateMessage(req);
    }

    std::string toJsonString(ros_babel_fish::BabelFish& fish,
                             const ros_babel_fish::BabelFishMessage& msg) override
    {
        return ros_nlohmann_converter::toJson(fish, msg).dump();
    }

    std::string parseAndStringify(const std::string& jsonStr) override
    {
        return nlohmann::json::parse(jsonStr).dump();
    }
};

template<typename T> class JSONTester : public testing::Test
{
public:
    T parser;
};

// Lister les types a tester dans les arguments de template
using ParserTypes = testing::Types<NlohmannJSONParser>;

// Defini les types sur lesquels faire les tests
TYPED_TEST_CASE(JSONTester, ParserTypes);

TYPED_TEST(JSONTester, CanFillStringMsgFromJson)
{
    const auto jsonData = R"({"data": "hello"})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "std_msgs/String",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["data"].value<std::string>(), "hello");
}

TYPED_TEST(JSONTester, CanFillPoseMsgFromJson)
{
    const auto jsonData =
        R"({"orientation":{"w":4.5,"x":1.2,"y":2.3,"z":3.4},"position":{"x":5.6,"y":6.7,"z":7.8}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "geometry_msgs/Pose",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["position"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["position"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["position"]["z"].value<double>(), 7.8);
    EXPECT_EQ(compound["orientation"]["x"].value<double>(), 1.2);
    EXPECT_EQ(compound["orientation"]["y"].value<double>(), 2.3);
    EXPECT_EQ(compound["orientation"]["z"].value<double>(), 3.4);
    EXPECT_EQ(compound["orientation"]["w"].value<double>(), 4.5);
}

TYPED_TEST(JSONTester, CanFillPoseStampedMsgFromJson)
{
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2,"stamp":{"nsecs":432423,"secs":34325435}},"pose":{"orientation":{"w":4.5,"x":1.2,"y":2.3,"z":3.4},"position":{"x":5.6,"y":6.7,"z":7.8}}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "geometry_msgs/PoseStamped", g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "robot");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 2);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 34325435);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 432423);
    EXPECT_EQ(compound["pose"]["position"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["pose"]["position"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["pose"]["position"]["z"].value<double>(), 7.8);
    EXPECT_EQ(compound["pose"]["orientation"]["x"].value<double>(), 1.2);
    EXPECT_EQ(compound["pose"]["orientation"]["y"].value<double>(), 2.3);
    EXPECT_EQ(compound["pose"]["orientation"]["z"].value<double>(), 3.4);
    EXPECT_EQ(compound["pose"]["orientation"]["w"].value<double>(), 4.5);
}

TYPED_TEST(JSONTester, CanFillOdometryMsgFromJson)
{
    const auto jsonData = R"({
        "header":{
            "frame_id":"robot","seq":2,"stamp":{"nsecs":432423,"secs":34325435}
        },
        "child_frame_id":"child_frame",
        "pose":{
            "covariance":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35],
            "pose":{
                "orientation":{"w":4.5,"x":1.2,"y":2.3,"z":3.4},
                "position":{"x":5.6,"y":6.7,"z":7.8}
            }
        },
        "twist":{
            "covariance":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35],
            "twist":{
                "angular":{"x":12.3,"y":23.4,"z":34.5},
                "linear":{"x":45.6,"y":56.7,"z":67.8}
            }
        }
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "nav_msgs/Odometry",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "robot");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 2);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 34325435);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 432423);

    EXPECT_EQ(compound["child_frame_id"].value<std::string>(), "child_frame");

    EXPECT_EQ(compound["pose"]["pose"]["position"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["pose"]["pose"]["position"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["pose"]["pose"]["position"]["z"].value<double>(), 7.8);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["x"].value<double>(), 1.2);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["y"].value<double>(), 2.3);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["z"].value<double>(), 3.4);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["w"].value<double>(), 4.5);
    const auto& poseCov = compound["pose"]["covariance"]
                              .as<ros_babel_fish::ArrayMessageBase>()
                              .as<ros_babel_fish::ArrayMessage<double>>();
    double covVal = 0.0;
    for(size_t i = 0; i < poseCov.length(); ++i)
    {
        EXPECT_EQ(poseCov[i], covVal);
        covVal += 1.0;
    }

    EXPECT_EQ(compound["twist"]["twist"]["angular"]["x"].value<double>(), 12.3);
    EXPECT_EQ(compound["twist"]["twist"]["angular"]["y"].value<double>(), 23.4);
    EXPECT_EQ(compound["twist"]["twist"]["angular"]["z"].value<double>(), 34.5);
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["x"].value<double>(), 45.6);
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["y"].value<double>(), 56.7);
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["z"].value<double>(), 67.8);
    covVal = 0.0;
    const auto& twistCov = compound["twist"]["covariance"]
                               .as<ros_babel_fish::ArrayMessageBase>()
                               .as<ros_babel_fish::ArrayMessage<double>>();
    for(size_t i = 0; i < twistCov.length(); ++i)
    {
        EXPECT_EQ(twistCov[i], covVal);
        covVal += 1.0;
    }
}

TYPED_TEST(JSONTester, CanFillPointMsgFromPartialJson)
{
    // position.z is missing
    const auto jsonData = R"({"x":5.6,"y":6.7})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "geometry_msgs/Point",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["z"].value<double>(), 0.0);
}

TYPED_TEST(JSONTester, CanFillPointStampedMsgFromJsonWithHeaderMissing)
{
    // position.z is missing
    const auto jsonData = R"({"point":{"x":5.6,"y":6.7,"z":7.8}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "geometry_msgs/PointStamped", g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 0);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 34325437);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 432427);
    EXPECT_EQ(compound["point"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["point"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["point"]["z"].value<double>(), 7.8);
}

TYPED_TEST(JSONTester, CanFillPointStampedMsgFromJsonWithStampMissing)
{
    // position.z is missing
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2},"point":{"x":5.6,"y":6.7,"z":7.8}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "geometry_msgs/PointStamped", g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "robot");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 2);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 34325437);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 432427);
    EXPECT_EQ(compound["point"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["point"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["point"]["z"].value<double>(), 7.8);
}

TYPED_TEST(JSONTester, CanFillPointStampedMsggFromJsonWithStampMissing)
{
    // position.z is missing
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2},"point":{"x":5.6,"y":6.7,"z":7.8}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "geometry_msgs/PointStamped", g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "robot");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 2);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 34325437);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 432427);
    EXPECT_EQ(compound["point"]["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["point"]["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["point"]["z"].value<double>(), 7.8);
}

TYPED_TEST(JSONTester, CanFillNavSatFixFromJSON)
{
    const auto jsonData =
        R"({"header":{"seq":123,"stamp":{"secs":1000,"nsecs":10000},"frame_id":"frame_id"},"status":{"status":1,"service":4},"latitude":123.456,"longitude":654.321,"altitude":100.101,"position_covariance":[1.1,0,0,0,2.2,0,0,0,3.3],"position_covariance_type":2})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "sensor_msgs/NavSatFix",
                                                            g_rosTime, jsonData));

    sensor_msgs::NavSatFix expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    EXPECT_EQ(serializedExpectedMsg.message_start - serializedExpectedMsg.buf.get(), 4);
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["status"]["status"].value<int8_t>(), expectedMsg.status.status);
    EXPECT_EQ(compound["status"]["service"].value<uint16_t>(),
              expectedMsg.status.service);
    EXPECT_EQ(compound["latitude"].value<double>(), expectedMsg.latitude);
    EXPECT_EQ(compound["longitude"].value<double>(), expectedMsg.longitude);
    EXPECT_EQ(compound["position_covariance_type"].value<uint8_t>(),
              expectedMsg.position_covariance_type);
    auto& base = compound["position_covariance"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<double>>();
    EXPECT_EQ(array[0], expectedMsg.position_covariance[0]);
    EXPECT_EQ(array[4], expectedMsg.position_covariance[4]);
    EXPECT_EQ(array[8], expectedMsg.position_covariance[8]);
}

TYPED_TEST(JSONTester, CanFillDiagnosticArrayFromJSON)
{
    const auto jsonData = R"({
        "header":{"seq":0,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},
        "status": [
            {
                "level":2,
                "name":"status1",
                "message":"message1",
                "values":[
                    {
                        "key":"key1",
                        "value":"value1"
                    },
                    {
                        "key":"key2",
                        "value":"value2"
                    },
                    {
                        "key":"key3",
                        "value":"value3"
                    }
                ]
            },
            {
                "level":1,
                "name":"status2",
                "message":"message2",
                "values":[
                    {
                        "key":"key1",
                        "value":"value11"
                    },
                    {
                        "key":"key2",
                        "value":"value22"
                    },
                    {
                        "key":"key3",
                        "value":"value33"
                    }
                ]
            }
        ]
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "diagnostic_msgs/DiagnosticArray", g_rosTime, jsonData));

    diagnostic_msgs::DiagnosticArray expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    auto& status = compound["status"].as<ros_babel_fish::CompoundArrayMessage>();
    ASSERT_EQ(status.length(), expectedMsg.status.size());
    {
        auto& status1 = status[0];
        const auto& es1 = expectedMsg.status[0];
        EXPECT_EQ(status1["level"].value<uint8_t>(), es1.level);
        EXPECT_EQ(status1["name"].value<std::string>(), es1.name);
        EXPECT_EQ(status1["message"].value<std::string>(), es1.message);
        // empty, not in JSON
        EXPECT_EQ(status1["hardware_id"].value<std::string>(), es1.hardware_id);
        auto& values1 = status1["values"].as<ros_babel_fish::CompoundArrayMessage>();
        ASSERT_EQ(values1.length(), es1.values.size());
        EXPECT_EQ(values1[0]["key"].value<std::string>(), es1.values[0].key);
        EXPECT_EQ(values1[0]["value"].value<std::string>(), es1.values[0].value);
        EXPECT_EQ(values1[1]["key"].value<std::string>(), es1.values[1].key);
        EXPECT_EQ(values1[1]["value"].value<std::string>(), es1.values[1].value);
        EXPECT_EQ(values1[2]["key"].value<std::string>(), es1.values[2].key);
        EXPECT_EQ(values1[2]["value"].value<std::string>(), es1.values[2].value);
    }
    {
        auto& status2 = status[1];
        const auto& es2 = expectedMsg.status[1];
        EXPECT_EQ(status2["level"].value<uint8_t>(), es2.level);
        EXPECT_EQ(status2["name"].value<std::string>(), es2.name);
        EXPECT_EQ(status2["message"].value<std::string>(), es2.message);
        EXPECT_EQ(status2["hardware_id"].value<std::string>(), es2.hardware_id);
        auto& values2 = status2["values"].as<ros_babel_fish::CompoundArrayMessage>();
        ASSERT_EQ(values2.length(), es2.values.size());
        EXPECT_EQ(values2[0]["key"].value<std::string>(), es2.values[0].key);
        EXPECT_EQ(values2[0]["value"].value<std::string>(), es2.values[0].value);
        EXPECT_EQ(values2[1]["key"].value<std::string>(), es2.values[1].key);
        EXPECT_EQ(values2[1]["value"].value<std::string>(), es2.values[1].value);
        EXPECT_EQ(values2[2]["key"].value<std::string>(), es2.values[2].key);
        EXPECT_EQ(values2[2]["value"].value<std::string>(), es2.values[2].value);
    }
}

TYPED_TEST(JSONTester, CanFillImageFromJSON)
{
    const auto jsonData = R"({
        "header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},
        "width":2,
        "height":3,
        "encoding":"bgr8",
        "is_bigendian":0,
        "step":6,
        "data":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "sensor_msgs/Image",
                                                            g_rosTime, jsonData));

    sensor_msgs::Image expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["width"].value<uint32_t>(), expectedMsg.width);
    EXPECT_EQ(compound["height"].value<uint32_t>(), expectedMsg.height);
    EXPECT_EQ(compound["step"].value<uint32_t>(), expectedMsg.step);
    EXPECT_EQ(compound["is_bigendian"].value<uint8_t>(), expectedMsg.is_bigendian);
    EXPECT_EQ(compound["encoding"].value<std::string>(), expectedMsg.encoding);
    auto& base = compound["data"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<uint8_t>>();
    ASSERT_EQ(array.length(), expectedMsg.data.size());
    for(size_t i = 0; i < array.length(); ++i)
    {
        EXPECT_EQ(array[i], expectedMsg.data[i]);
    }
}

TYPED_TEST(JSONTester, CanFillCompressedImageFromJSON)
{
    const auto jsonData = R"({
        "header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},
        "format":"jpeg",
        "data":[0,1,2,3,4,5,6,7,8,9]
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/CompressedImage", g_rosTime, jsonData));

    sensor_msgs::CompressedImage expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["format"].value<std::string>(), expectedMsg.format);
    auto& base = compound["data"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<uint8_t>>();
    ASSERT_EQ(array.length(), expectedMsg.data.size());
    for(size_t i = 0; i < array.length(); ++i)
    {
        EXPECT_EQ(array[i], expectedMsg.data[i]);
    }
}

TYPED_TEST(JSONTester, CanFillCompressedImageFromBase64JSON)
{
    const auto jsonData = R"({
        "header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},
        "format":"jpeg",
        "data":"AAECAwQFBgcICQ=="
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/CompressedImage", g_rosTime, jsonData));

    sensor_msgs::CompressedImage expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["format"].value<std::string>(), expectedMsg.format);
    auto& base = compound["data"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<uint8_t>>();
    ASSERT_EQ(array.length(), expectedMsg.data.size());
    for(size_t i = 0; i < array.length(); ++i)
    {
        EXPECT_EQ(array[i], expectedMsg.data[i]);
    }
}

TYPED_TEST(JSONTester, CanFillCompressedImageFromNullJSON)
{
    const auto jsonData = R"({
        "header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},
        "format":"jpeg",
        "data":[null,null,null,null]
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/CompressedImage", g_rosTime, jsonData));

    sensor_msgs::CompressedImage expectedMsg;
    fillMessage(expectedMsg);
    expectedMsg.data.clear();
    expectedMsg.data.push_back(0);
    expectedMsg.data.push_back(0);
    expectedMsg.data.push_back(0);
    expectedMsg.data.push_back(0);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["format"].value<std::string>(), expectedMsg.format);
    auto& base = compound["data"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<uint8_t>>();
    ASSERT_EQ(array.length(), expectedMsg.data.size());
    for(size_t i = 0; i < array.length(); ++i)
    {
        EXPECT_EQ(array[i], expectedMsg.data[i]);
    }
}

TYPED_TEST(JSONTester, CanFillOdometryFromJson)
{
    const auto jsonData = R"({
        "child_frame_id": "child",
        "header": {
            "frame_id": "robot",
            "seq": 456,
            "stamp":{"secs":34325435,"nsecs":432423}
        },
        "pose": {
            "covariance": [0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0,31.0,32.0,33.0,34.0,35.0],
            "pose": {
                "orientation": {
                    "w": 4.0,
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0
                },
                "position": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0
                }
            }
        },
        "twist": {
            "covariance": [0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0,31.0,32.0,33.0,34.0,35.0],
            "twist": {
                "angular": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0
                },
                "linear": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0
                }
            }
        }
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "nav_msgs/Odometry",
                                                            g_rosTime, jsonData));

    nav_msgs::Odometry expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(),
              expectedMsg.header.frame_id);
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), expectedMsg.header.seq);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec,
              expectedMsg.header.stamp.sec);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec,
              expectedMsg.header.stamp.nsec);
    EXPECT_EQ(compound["child_frame_id"].value<std::string>(),
              expectedMsg.child_frame_id);
    EXPECT_EQ(compound["pose"]["pose"]["position"]["x"].value<double>(),
              expectedMsg.pose.pose.position.x);
    EXPECT_EQ(compound["pose"]["pose"]["position"]["y"].value<double>(),
              expectedMsg.pose.pose.position.y);
    EXPECT_EQ(compound["pose"]["pose"]["position"]["z"].value<double>(),
              expectedMsg.pose.pose.position.z);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["x"].value<double>(),
              expectedMsg.pose.pose.orientation.x);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["y"].value<double>(),
              expectedMsg.pose.pose.orientation.y);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["z"].value<double>(),
              expectedMsg.pose.pose.orientation.z);
    EXPECT_EQ(compound["pose"]["pose"]["orientation"]["w"].value<double>(),
              expectedMsg.pose.pose.orientation.w);
    {
        auto& base =
            compound["pose"]["covariance"].as<ros_babel_fish::ArrayMessageBase>();
        auto& array = base.as<ros_babel_fish::ArrayMessage<double>>();
        ASSERT_EQ(array.length(), expectedMsg.pose.covariance.size());
        for(size_t i = 0; i < array.length(); ++i)
        {
            EXPECT_EQ(array[i], expectedMsg.pose.covariance[i]);
        }
    }
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["x"].value<double>(),
              expectedMsg.twist.twist.linear.x);
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["y"].value<double>(),
              expectedMsg.twist.twist.linear.y);
    EXPECT_EQ(compound["twist"]["twist"]["linear"]["z"].value<double>(),
              expectedMsg.twist.twist.linear.z);
    EXPECT_EQ(compound["twist"]["twist"]["angular"]["x"].value<double>(),
              expectedMsg.twist.twist.angular.x);
    EXPECT_EQ(compound["twist"]["twist"]["angular"]["y"].value<double>(),
              expectedMsg.twist.twist.angular.y);
    EXPECT_EQ(compound["twist"]["twist"]["angular"]["z"].value<double>(),
              expectedMsg.twist.twist.angular.z);
    {
        auto& base =
            compound["twist"]["covariance"].as<ros_babel_fish::ArrayMessageBase>();
        auto& array = base.as<ros_babel_fish::ArrayMessage<double>>();
        ASSERT_EQ(array.length(), expectedMsg.twist.covariance.size());
        for(size_t i = 0; i < array.length(); ++i)
        {
            EXPECT_EQ(array[i], expectedMsg.twist.covariance[i]);
        }
    }
}

TYPED_TEST(JSONTester, CanFillFloat64MsgFromJson)
{
    const auto jsonData = R"({"data":3.14})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "std_msgs/Float64",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["data"].value<double>(), 3.14);
}

TYPED_TEST(JSONTester, CanFillFloat64MsgFromJsonNull)
{
    const auto jsonData = R"({"data":null})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "std_msgs/Float64",
                                                            g_rosTime, jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_TRUE(std::isnan(compound["data"].value<double>()));
}

TYPED_TEST(JSONTester, CanFillChannelFloat32FromJson)
{
    const auto jsonData = R"({"name":"channel name","values":[0.0,1.0,2.0,3.0,4.0]})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/ChannelFloat32", g_rosTime, jsonData));

    sensor_msgs::ChannelFloat32 expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["name"].value<std::string>(), expectedMsg.name);
    {
        auto& base = compound["values"].as<ros_babel_fish::ArrayMessageBase>();
        auto& array = base.as<ros_babel_fish::ArrayMessage<float>>();
        ASSERT_EQ(array.length(), expectedMsg.values.size());
        for(size_t i = 0; i < array.length(); ++i)
        {
            EXPECT_EQ(array[i], expectedMsg.values[i]);
        }
    }
}

TYPED_TEST(JSONTester, CanFillEmptyChannelFloat32FromJson)
{
    const auto jsonData = R"({"name":"channel name","values":[]})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/ChannelFloat32", g_rosTime, jsonData));

    sensor_msgs::ChannelFloat32 expectedMsg;
    expectedMsg.name = "channel name";
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["name"].value<std::string>(), expectedMsg.name);
    {
        auto& base = compound["values"].as<ros_babel_fish::ArrayMessageBase>();
        auto& array = base.as<ros_babel_fish::ArrayMessage<float>>();
        ASSERT_EQ(array.length(), expectedMsg.values.size());
    }
}

TYPED_TEST(JSONTester, CanFillChannelFloat32WithNullFromJson)
{
    const auto jsonData = R"({"name":"channel name","values":[null,null]})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(
                        fish, "sensor_msgs/ChannelFloat32", g_rosTime, jsonData));

    sensor_msgs::ChannelFloat32 expectedMsg;
    expectedMsg.name = "channel name";
    expectedMsg.values.push_back(std::numeric_limits<double>::quiet_NaN());
    expectedMsg.values.push_back(std::numeric_limits<double>::quiet_NaN());
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["name"].value<std::string>(), expectedMsg.name);
    {
        auto& base = compound["values"].as<ros_babel_fish::ArrayMessageBase>();
        auto& array = base.as<ros_babel_fish::ArrayMessage<float>>();
        ASSERT_EQ(array.length(), expectedMsg.values.size());
        for(size_t i = 0; i < array.length(); ++i)
        {
            EXPECT_TRUE(std::isnan(array[i]));
            EXPECT_TRUE(std::isnan(expectedMsg.values[i]));
        }
    }
}

TYPED_TEST(JSONTester, CanFillNegativeDurationFromJson)
{
    const auto jsonData = R"({"data":{"secs":-15,"nsecs":123}})";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createMsgFromJson(fish, "std_msgs/Duration",
                                                            g_rosTime, jsonData));

    std_msgs::Duration expectedMsg;
    expectedMsg.data = ros::Duration{-15, 123};
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes - 4);
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["data"].value<ros::Duration>().sec, expectedMsg.data.sec);
    EXPECT_EQ(compound["data"].value<ros::Duration>().nsec, expectedMsg.data.nsec);
}

TYPED_TEST(JSONTester, CanFillSetCameraInfoSrvReqFromPartialJson)
{
    const auto jsonData = R"({
        "camera_info":
        {
            "header":
            {
                "frame_id": "my_frame"
            },
            "height": 13,
            "width": 16
        }
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = this->parser.createServiceRequestFromJson(
                        fish, "sensor_msgs/SetCameraInfo", jsonData));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["camera_info"]["height"].value<double>(), 13);
    EXPECT_EQ(compound["camera_info"]["width"].value<double>(), 16);
}

TYPED_TEST(JSONTester, CannotFillSetCameraInfoSrvReqFromBadJson)
{
    // camera_info indent level missing, fields directly in the json root
    const auto jsonData = R"({
        "header":
        {
            "frame_id": "my_frame"
        },
        "height": 13,
        "width": 16
    })";

    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_THROW(rosMsg = this->parser.createServiceRequestFromJson(
                     fish, "sensor_msgs/SetCameraInfo", jsonData),
                 std::runtime_error);
}

/////////////////
/// ROS to JSON
/////////////////

TYPED_TEST(JSONTester, CanConvertSerializedPoseStampedToJson)
{
    ros_babel_fish::BabelFish fish;
    geometry_msgs::PoseStamped msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);

    const std::string json = this->parser.toJsonString(fish, bfMsg);
    const auto expectedJson =
        R"({"header":{"seq":0,"stamp":{"secs":34325435,"nsecs":432423},"frame_id":"robot"},"pose":{"position":{"x":1.0,"y":2.0,"z":3.0},"orientation":{"x":1.0,"y":2.0,"z":3.0,"w":4.0}}})";
    // Json serialization order might differ accros library, so parse and stringify the
    // expected string to get comparable result
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertSerializedPoseStampedToJsonTwice)
{
    ros_babel_fish::BabelFish fish;
    geometry_msgs::PoseStamped msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);

    const auto expectedJson =
        R"({"header":{"seq":0,"stamp":{"secs":34325435,"nsecs":432423},"frame_id":"robot"},"pose":{"position":{"x":1.0,"y":2.0,"z":3.0},"orientation":{"x":1.0,"y":2.0,"z":3.0,"w":4.0}}})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    {
        const std::string json = this->parser.toJsonString(fish, bfMsg);
        EXPECT_EQ(json, expectedOutput);
    }
    {
        const std::string json = this->parser.toJsonString(fish, bfMsg);
        EXPECT_EQ(json, expectedOutput);
    }
}

TYPED_TEST(JSONTester, CanConvertNavSatFixToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::NavSatFix msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson =
        R"({"header":{"seq":123,"stamp":{"secs":1000,"nsecs":10000},"frame_id":"frame_id"},"status":{"status":1,"service":4},"latitude":123.456,"longitude":654.321,"altitude":100.101,"position_covariance":[1.1,0.0,0.0,0.0,2.2,0.0,0.0,0.0,3.3],"position_covariance_type":2})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertDiagnosticArrayToJson)
{
    ros_babel_fish::BabelFish fish;
    diagnostic_msgs::DiagnosticArray msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson =
        R"({"header":{"seq":0,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},"status":[{"level":2,"name":"status1","message":"message1","hardware_id":"","values":[{"key":"key1","value":"value1"},{"key":"key2","value":"value2"},{"key":"key3","value":"value3"}]},{"level":1,"name":"status2","message":"message2","hardware_id":"","values":[{"key":"key1","value":"value11"},{"key":"key2","value":"value22"},{"key":"key3","value":"value33"}]}]})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertImageToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::Image msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);
    // uint8[] encoded as base64
    const auto expectedJson =
        R"({"header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},"height":3,"width":2,"encoding":"bgr8","is_bigendian":0,"step":6,"data":"AAECAwQFBgcICQoLDA0ODxAR"})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}
TYPED_TEST(JSONTester, CanConvertCompressedImageToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::CompressedImage msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);
    // uint8[] encoded as base64
    const auto expectedJson =
        R"({"header":{"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},"format":"jpeg","data":"AAECAwQFBgcICQ=="})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertFloat64ToJson)
{
    ros_babel_fish::BabelFish fish;
    std_msgs::Float64 msg;
    msg.data = 3.14;
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson = R"({"data":3.14})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertFloat64InfinityToJson)
{
    ros_babel_fish::BabelFish fish;
    std_msgs::Float64 msg;
    msg.data = std::numeric_limits<double>::infinity();
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson = R"({"data":null})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertFloat64NaNToJson)
{
    ros_babel_fish::BabelFish fish;
    std_msgs::Float64 msg;
    msg.data = std::numeric_limits<double>::quiet_NaN();
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson = R"({"data":null})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertChannelFloat32ToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::ChannelFloat32 msg;
    fillMessage(msg);
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);
    const auto expectedJson = R"({"name":"channel name","values":[0.0,1.0,2.0,3.0,4.0]})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertEmptyChannelFloat32ToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::ChannelFloat32 msg;
    msg.name = "channel name";
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson = R"({"name":"channel name","values":[]})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

TYPED_TEST(JSONTester, CanConvertChannelFloat32WithNaNAndInfinityToJson)
{
    ros_babel_fish::BabelFish fish;
    sensor_msgs::ChannelFloat32 msg;
    msg.name = "channel name";
    msg.values.push_back(1.0);
    msg.values.push_back(std::numeric_limits<double>::quiet_NaN());
    msg.values.push_back(std::numeric_limits<double>::infinity());
    auto bfMsg = serializeMessage(fish, msg);
    const std::string json = this->parser.toJsonString(fish, bfMsg);

    const auto expectedJson = R"({"name":"channel name","values":[1.0, null, null]})";
    const auto expectedOutput = this->parser.parseAndStringify(expectedJson);
    EXPECT_EQ(json, expectedOutput);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
