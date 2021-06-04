#include <gtest/gtest.h>

#include <ros_babel_fish/babel_fish.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/writer.h"

#include "rapidjson_to_ros.h"

static const ros::Time g_rosTime{34325437, 432427};

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

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
