#pragma once

#include <cassert>
#include <optional>
#include <string>

#include <ros_babel_fish/babel_fish.h>

#include "rapidjson/document.h"

namespace ros_rapidjson_converter
{

template<typename T>
void fillIntArray(const rapidjson::Value::ConstArray& jsonArray,
                  ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<T>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.Size());
    }
    else
    {
        msgArray.reserve(jsonArray.Size());
    }
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        if(msgArray.isFixedSize())
        {
            msgArray.assign(i, jsonArray[i].GetInt());
        }
        else
        {
            msgArray.push_back(jsonArray[i].GetInt());
        }
    }
}

template<typename T>
void fillUintArray(const rapidjson::Value::ConstArray& jsonArray,
                   ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<T>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.Size());
    }
    else
    {
        msgArray.reserve(jsonArray.Size());
    }
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        if(msgArray.isFixedSize())
        {
            msgArray.assign(i, jsonArray[i].GetUint());
        }
        else
        {
            msgArray.push_back(jsonArray[i].GetUint());
        }
    }
}

void fillBoolArray(const rapidjson::Value::ConstArray& jsonArray,
                   ros_babel_fish::ArrayMessageBase& baseArray);
void fillUint64Array(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray);
void fillInt64Array(const rapidjson::Value::ConstArray& jsonArray,
                    ros_babel_fish::ArrayMessageBase& baseArray);
void fillFloatArray(const rapidjson::Value::ConstArray& jsonArray,
                    ros_babel_fish::ArrayMessageBase& baseArray);
void fillDoubleArray(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray);
void fillStringArray(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray);
void fillTimeArray(const rapidjson::Value::ConstArray& jsonArray,
                   ros_babel_fish::ArrayMessageBase& baseArray);
void fillDurationArray(const rapidjson::Value::ConstArray& jsonArray,
                       ros_babel_fish::ArrayMessageBase& baseArray);

void fillMessageFromJson(const rapidjson::Value& json,
                         ros_babel_fish::CompoundMessage& message);

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const rapidjson::Value& json);
} // namespace ros_rapidjson_converter
