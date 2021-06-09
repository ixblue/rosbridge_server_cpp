#include <gtest/gtest.h>

#include <ros_babel_fish/babel_fish.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

// TODO remove to not depend on this msg
#include <wms_server/LayerInfoArray.h>

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/writer.h"

#include "rapidjson_to_ros.h"
#include "ros_to_rapidjson.h"

static const ros::Time g_rosTime{34325437, 432427};

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
    status1.message = "message";
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

/// TODO remove

inline wms_server::LayerInfo addLayer(const std::string& name)
{
    wms_server::LayerInfo l;
    l.layer_name = name;
    l.public_name = "public name";
    l.topic = "/topic_name";
    l.grid_map_layer = "gris_map_layer";
    l.colormap.auto_minmax = true;
    l.colormap.min_value = 10.0;
    l.colormap.max_value = 100.0;
    l.colormap.alpha = 1.0;
    l.colormap.invert_colormap = true;
    l.colormap.colormap_name = "RAINBOW";
    l.colormap.available_colormaps = {"AUTUMN", "BONE",   "COOL",   "HOT",  "HSV",
                                      "JET",    "OCEAN",  "PARULA", "PINK", "RAINBOW",
                                      "SPRING", "SUMMER", "WINTER"};
    l.colormap.colormap_values.reserve(256);
    for(int i = 0; i < 256; ++i)
    {
        wms_server::RGBValue value;
        value.r = i;
        value.g = i + 1;
        value.b = i + 2;
        l.colormap.colormap_values.push_back(std::move(value));
    }
    return l;
}

inline void fillMessage(wms_server::LayerInfoArray& m)
{
    m.layers.push_back(addLayer("drix_1_count"));
    m.layers.push_back(addLayer("drix_1_depth"));
    m.layers.push_back(addLayer("drix_1_std_dev"));
    m.layers.push_back(addLayer("drix_2_count"));
    m.layers.push_back(addLayer("drix_2_depth"));
    m.layers.push_back(addLayer("drix_2_std_dev"));
}
/// TODO remove

TEST(JsonToROSTester, CanFillStringMsgFromJson)
{
    const auto jsonData = R"({"data": "hello"})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(fish, "std_msgs/String",
                                                                g_rosTime, doc));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["data"].value<std::string>(), "hello");
}

TEST(JsonToROSTester, CanFillPoseMsgFromJson)
{
    const auto jsonData =
        R"({"orientation":{"w":4.5,"x":1.2,"y":2.3,"z":3.4},"position":{"x":5.6,"y":6.7,"z":7.8}})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/Pose", g_rosTime, doc));
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

TEST(JsonToROSTester, CanFillPoseStampedMsgFromJson)
{
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2,"stamp":{"nsecs":432423,"secs":34325435}},"pose":{"orientation":{"w":4.5,"x":1.2,"y":2.3,"z":3.4},"position":{"x":5.6,"y":6.7,"z":7.8}}})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/PoseStamped", g_rosTime, doc));
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

TEST(JsonToROSTester, CanFillOdometryMsgFromJson)
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
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(fish, "nav_msgs/Odometry",
                                                                g_rosTime, doc));
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

TEST(JsonToROSTester, CanFillPointMsgFromPartialJson)
{
    // position.z is missing
    const auto jsonData = R"({"x":5.6,"y":6.7})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/Point", g_rosTime, doc));
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["x"].value<double>(), 5.6);
    EXPECT_EQ(compound["y"].value<double>(), 6.7);
    EXPECT_EQ(compound["z"].value<double>(), 0.0);
}

TEST(JsonToROSTester, CanFillPointStampedMsgFromJsonWithHeaderMissing)
{
    // position.z is missing
    const auto jsonData = R"({"point":{"x":5.6,"y":6.7,"z":7.8}})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/PointStamped", g_rosTime, doc));
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

TEST(JsonToROSTester, CanFillPointStampedMsgFromJsonWithStampMissing)
{
    // position.z is missing
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2},"point":{"x":5.6,"y":6.7,"z":7.8}})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/PointStamped", g_rosTime, doc));
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

TEST(JsonToROSTester, CanFillNavSatFixFromJSON)
{
    const auto jsonData =
        R"({"header":{"seq":123,"stamp":{"secs":1000,"nsecs":10000},"frame_id":"frame_id"},"status":{"status":1,"service":4},"latitude":123.456,"longitude":654.321,"altitude":100.101,"position_covariance":[1.1,0,0,0,2.2,0,0,0,3.3],"position_covariance_type":2})";

    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "sensor_msgs/NavSatFix", g_rosTime, doc));

    sensor_msgs::NavSatFix expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    // ASSERT_EQ(rosMsg->size(), serializedExpectedMsg.num_bytes);
    ASSERT_EQ(rosMsg->size(),
              (serializedExpectedMsg.num_bytes -
               (serializedExpectedMsg.message_start - serializedExpectedMsg.buf.get())));
    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
                          rosMsg->size()),
              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "frame_id");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 123);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 1000);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 10000);
    EXPECT_EQ(compound["status"]["status"].value<int8_t>(), 1);
    EXPECT_EQ(compound["status"]["service"].value<uint16_t>(), 4);
    EXPECT_EQ(compound["latitude"].value<double>(), 123.456);
    EXPECT_EQ(compound["longitude"].value<double>(), 654.321);
    EXPECT_EQ(compound["position_covariance_type"].value<uint8_t>(), 2);
    auto& base = compound["position_covariance"].as<ros_babel_fish::ArrayMessageBase>();
    auto& array = base.as<ros_babel_fish::ArrayMessage<double>>();
    EXPECT_EQ(array[0], 1.1);
    EXPECT_EQ(array[4], 2.2);
    EXPECT_EQ(array[8], 3.3);
}

TEST(JsonToROSTester, CanFillDiagnosticArrayFromJSON)
{
    const auto jsonData = R"({
        "header":{"seq":0,"stamp":{"secs":1000,"nsecs":10000},"frame_id":"frame_id"},
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

    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "diagnostic_msgs/DiagnosticArray", g_rosTime, doc));

    sensor_msgs::NavSatFix expectedMsg;
    fillMessage(expectedMsg);
    ros::SerializedMessage serializedExpectedMsg =
        ros::serialization::serializeMessage(expectedMsg);

    // Compare ROS encoded message
    //    ASSERT_EQ(rosMsg->size(),
    //              (serializedExpectedMsg.num_bytes -
    //               (serializedExpectedMsg.message_start -
    //               serializedExpectedMsg.buf.get())));
    //    EXPECT_EQ(std::memcmp(rosMsg->buffer(), serializedExpectedMsg.message_start,
    //                          rosMsg->size()),
    //              0);

    // Compare values
    ros_babel_fish::TranslatedMessage::Ptr translated = fish.translateMessage(rosMsg);
    auto& compound =
        translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    EXPECT_EQ(compound["header"]["frame_id"].value<std::string>(), "frame_id");
    EXPECT_EQ(compound["header"]["seq"].value<uint32_t>(), 0);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().sec, 1000);
    EXPECT_EQ(compound["header"]["stamp"].value<ros::Time>().nsec, 10000);
    auto& status = compound["status"].as<ros_babel_fish::CompoundArrayMessage>();
    ASSERT_EQ(status.length(), 2);
    {
        auto& status1 = status[0];
        EXPECT_EQ(status1["level"].value<uint8_t>(), 2);
        EXPECT_EQ(status1["name"].value<std::string>(), "status1");
        EXPECT_EQ(status1["message"].value<std::string>(), "message1");
        auto& values1 = status1["values"].as<ros_babel_fish::CompoundArrayMessage>();
        ASSERT_EQ(values1.length(), 3);
        EXPECT_EQ(values1[0]["key"].value<std::string>(), "key1");
        EXPECT_EQ(values1[0]["value"].value<std::string>(), "value1");
        EXPECT_EQ(values1[1]["key"].value<std::string>(), "key2");
        EXPECT_EQ(values1[1]["value"].value<std::string>(), "value2");
        EXPECT_EQ(values1[2]["key"].value<std::string>(), "key3");
        EXPECT_EQ(values1[2]["value"].value<std::string>(), "value3");
    }
    {
        auto& status2 = status[1];
        EXPECT_EQ(status2["level"].value<uint8_t>(), 1);
        EXPECT_EQ(status2["name"].value<std::string>(), "status2");
        EXPECT_EQ(status2["message"].value<std::string>(), "message2");
        auto& values2 = status2["values"].as<ros_babel_fish::CompoundArrayMessage>();
        ASSERT_EQ(values2.length(), 3);
        EXPECT_EQ(values2[0]["key"].value<std::string>(), "key1");
        EXPECT_EQ(values2[0]["value"].value<std::string>(), "value11");
        EXPECT_EQ(values2[1]["key"].value<std::string>(), "key2");
        EXPECT_EQ(values2[1]["value"].value<std::string>(), "value22");
        EXPECT_EQ(values2[2]["key"].value<std::string>(), "key3");
        EXPECT_EQ(values2[2]["value"].value<std::string>(), "value33");
    }
}

TEST(JsonToROSTester, CanFillLayerInfoArrayMsgFromJsonWithStampMissing)
{
    // position.z is missing
    const auto jsonData =
        R"({"header":{"frame_id":"robot","seq":2},"point":{"x":5.6,"y":6.7,"z":7.8}})";
    rapidjson::Document doc;
    doc.Parse(jsonData);
    ASSERT_FALSE(doc.HasParseError());
    ros_babel_fish::BabelFish fish;
    ros_babel_fish::BabelFishMessage::Ptr rosMsg;
    ASSERT_NO_THROW(rosMsg = ros_rapidjson_converter::createMsg(
                        fish, "geometry_msgs/PointStamped", g_rosTime, doc));
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

/////////////////
/// ROS to JSON
/////////////////

TEST(ROSToJsonTester, CanConvertSerializedPoseStampedToJson)
{
    const std::string& datatype =
        ros::message_traits::DataType<geometry_msgs::PoseStamped>::value();
    const std::string& definition =
        ros::message_traits::Definition<geometry_msgs::PoseStamped>::value();
    const std::string& md5 =
        ros::message_traits::MD5Sum<geometry_msgs::PoseStamped>::value();

    geometry_msgs::PoseStamped msg;
    fillMessage(msg);

    // Create serialized version of the message
    ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage(msg);

    ros_babel_fish::BabelFishMessage bf_msg;
    bf_msg.morph(md5, datatype, definition);
    ros_babel_fish::BabelFish fish;

    fish.descriptionProvider()->getMessageDescription(bf_msg);
    ros::serialization::deserializeMessage(serialized_msg, bf_msg);

    rapidjson::Document doc;
    ros_rapidjson_converter::toJson(fish, bf_msg, doc, doc.GetAllocator());

    const std::string json = ros_rapidjson_converter::jsonToString(doc);

    EXPECT_EQ(
        json,
        R"({"header":{"seq":0,"stamp":{"secs":34325435,"nsecs":432423},"frame_id":"robot"},"pose":{"position":{"x":1.0,"y":2.0,"z":3.0},"orientation":{"x":1.0,"y":2.0,"z":3.0,"w":4.0}}})");
}

TEST(ROSToJsonTester, CanConvertSerializedPoseStampedToJsonTwice)
{
    const std::string& datatype =
        ros::message_traits::DataType<geometry_msgs::PoseStamped>::value();
    const std::string& definition =
        ros::message_traits::Definition<geometry_msgs::PoseStamped>::value();
    const std::string& md5 =
        ros::message_traits::MD5Sum<geometry_msgs::PoseStamped>::value();

    geometry_msgs::PoseStamped msg;
    fillMessage(msg);

    // Create serialized version of the message
    ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage(msg);

    ros_babel_fish::BabelFishMessage bf_msg;
    bf_msg.morph(md5, datatype, definition);
    ros_babel_fish::BabelFish fish;

    fish.descriptionProvider()->getMessageDescription(bf_msg);
    ros::serialization::deserializeMessage(serialized_msg, bf_msg);

    {
        rapidjson::Document doc;
        ros_rapidjson_converter::toJson(fish, bf_msg, doc, doc.GetAllocator());

        const std::string json = ros_rapidjson_converter::jsonToString(doc);

        EXPECT_EQ(
            json,
            R"({"header":{"seq":0,"stamp":{"secs":34325435,"nsecs":432423},"frame_id":"robot"},"pose":{"position":{"x":1.0,"y":2.0,"z":3.0},"orientation":{"x":1.0,"y":2.0,"z":3.0,"w":4.0}}})");
    }
    {
        rapidjson::Document doc;
        ros_rapidjson_converter::toJson(fish, bf_msg, doc, doc.GetAllocator());

        const std::string json = ros_rapidjson_converter::jsonToString(doc);

        EXPECT_EQ(
            json,
            R"({"header":{"seq":0,"stamp":{"secs":34325435,"nsecs":432423},"frame_id":"robot"},"pose":{"position":{"x":1.0,"y":2.0,"z":3.0},"orientation":{"x":1.0,"y":2.0,"z":3.0,"w":4.0}}})");
    }
}

TEST(ROSToJsonTester, CanConvertSerializedLayerInfoArrayToJson)
{
    const std::string& datatype =
        ros::message_traits::DataType<wms_server::LayerInfoArray>::value();
    const std::string& definition =
        ros::message_traits::Definition<wms_server::LayerInfoArray>::value();
    const std::string& md5 =
        ros::message_traits::MD5Sum<wms_server::LayerInfoArray>::value();

    wms_server::LayerInfoArray msg;
    fillMessage(msg);

    // Create serialized version of the message
    ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage(msg);

    ros_babel_fish::BabelFishMessage bf_msg;
    bf_msg.morph(md5, datatype, definition);
    ros_babel_fish::BabelFish fish;

    fish.descriptionProvider()->getMessageDescription(bf_msg);
    ros::serialization::deserializeMessage(serialized_msg, bf_msg);

    rapidjson::Document doc;
    ros_rapidjson_converter::toJson(fish, bf_msg, doc, doc.GetAllocator());

    const std::string json = ros_rapidjson_converter::jsonToString(doc);

    rapidjson::Document outDoc;
    outDoc.Parse(json);

    ASSERT_TRUE(outDoc["layers"].IsArray());
    const auto layersArray = outDoc["layers"].GetArray();

    EXPECT_EQ(layersArray.Size(), 6);
    for(auto& layerElem : layersArray)
    {
        ASSERT_TRUE(layerElem.IsObject());
        const auto layer = layerElem.GetObj();
        ASSERT_TRUE(layer.HasMember("colormap"));
        ASSERT_TRUE(layer["colormap"].IsObject());
        const auto colormap = layer["colormap"].GetObj();
        ASSERT_TRUE(colormap["colormap_values"].IsArray());
        const auto values = layer["colormap"]["colormap_values"].GetArray();
        EXPECT_EQ(values.Size(), 256);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
