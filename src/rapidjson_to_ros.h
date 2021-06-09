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
        assert(msgArray.length() >= jsonArray.Size());
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
        assert(msgArray.length() >= jsonArray.Size());
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

// template<typename T>
// std::optional<typename ros_babel_fish::message_type_traits::array_type<T>::ReturnType>
// toRosValue(ros_babel_fish::MessageType msgType, const rapidjson::Value& jsonVal)
//{
//    switch(msgType)
//    {
//    case ros_babel_fish::MessageTypes::None: return std::nullopt;
//    case ros_babel_fish::MessageTypes::Compound: return std::nullopt;
//    case ros_babel_fish::MessageTypes::Array: return std::nullopt;
//    case ros_babel_fish::MessageTypes::Bool: return jsonVal.GetBool(); break;
//    case ros_babel_fish::MessageTypes::UInt8:
//        return static_cast<uint8_t>(jsonVal.GetInt());
//        break;
//    case ros_babel_fish::MessageTypes::UInt16:
//        return static_cast<uint16_t>(jsonVal.GetInt());
//        break;
//    case ros_babel_fish::MessageTypes::UInt32:
//        return static_cast<uint32_t>(jsonVal.GetInt());
//        break;
//    case ros_babel_fish::MessageTypes::UInt64:
//        return static_cast<uint64_t>(jsonVal.GetInt64());
//        break;
//    case ros_babel_fish::MessageTypes::Int8:
//        return static_cast<int8_t>(jsonVal.GetUint());
//        break;
//    case ros_babel_fish::MessageTypes::Int16:
//        return static_cast<int16_t>(jsonVal.GetUint());
//        break;
//    case ros_babel_fish::MessageTypes::Int32:
//        return static_cast<int32_t>(jsonVal.GetUint());
//        break;
//    case ros_babel_fish::MessageTypes::Int64:
//        return static_cast<int64_t>(jsonVal.GetUint64());
//        break;
//    case ros_babel_fish::MessageTypes::Float32:
//        return static_cast<float>(jsonVal.GetFloat());
//        break;
//    case ros_babel_fish::MessageTypes::Float64: return jsonVal.GetDouble(); break;
//    case ros_babel_fish::MessageTypes::Time: {
//        ros::Time time;
//        const auto timeObj = jsonVal.GetObject();
//        time.sec = timeObj["secs"].GetInt();
//        time.nsec = timeObj["nsecs"].GetInt();
//        return time;
//        break;
//    }
//    case ros_babel_fish::MessageTypes::Duration: {
//        ros::Duration time;
//        const auto timeObj = jsonVal.GetObject();
//        time.sec = timeObj["secs"].GetInt();
//        time.nsec = timeObj["nsecs"].GetInt();
//        return time;
//        break;
//    }
//    case ros_babel_fish::MessageTypes::String: return jsonVal.GetString(); break;
//    }
//}

// template<typename T>
// void fillArray(ros_babel_fish::MessageType msgType,
//               const rapidjson::Value::ConstArray& jsonArray,
//               ros_babel_fish::ArrayMessage<T>& msgArray)
//{
//    assert(msgArray.length() >= jsonArray.Size());
//    for(size_t i = 0; i < jsonArray.Size(); ++i)
//    {
//        if(const auto rosVal = toRosValue<T>(msgType, jsonArray[i]); rosVal)
//        {
//            msgArray.assign(i, *rosVal);
//        }
//    }
//}

void fillMessageFromJson(const rapidjson::Value& json,
                         ros_babel_fish::CompoundMessage& message);

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const rapidjson::Value& json);
} // namespace ros_rapidjson_converter
