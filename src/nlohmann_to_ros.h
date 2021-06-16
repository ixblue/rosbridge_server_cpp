#pragma once

#include <cassert>
#include <string>

#include <ros_babel_fish/babel_fish.h>

#include "nlohmann/json.hpp"

namespace ros_nlohmann_converter
{

void fillMessageFromJson(const nlohmann::json& json,
                         ros_babel_fish::CompoundMessage& message);

template<typename T>
void fillArray(const nlohmann::json& jsonArray,
               ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<T>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.size());
    }

    for(size_t i = 0; i < jsonArray.size(); ++i)
    {
        if(msgArray.isFixedSize())
        {
            msgArray.assign(i, jsonArray[i]);
        }
        else
        {
            msgArray.push_back(jsonArray[i]);
        }
    }
}

template<>
void fillArray<ros::Time>(const nlohmann::json& jsonArray,
                          ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<ros::Time>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.size());
    }

    for(size_t i = 0; i < jsonArray.size(); ++i)
    {
        ros::Time time;
        time.sec = jsonArray[i]["secs"].get<uint32_t>();
        time.nsec = jsonArray[i]["nsecs"].get<uint32_t>();
        if(msgArray.isFixedSize())
        {
            msgArray.assign(i, time);
        }
        else
        {
            msgArray.push_back(time);
        }
    }
}

template<>
void fillArray<ros::Duration>(const nlohmann::json& jsonArray,
                              ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<ros::Duration>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.size());
    }

    for(size_t i = 0; i < jsonArray.size(); ++i)
    {
        ros::Duration time;
        time.sec = jsonArray[i]["secs"].get<uint32_t>();
        time.nsec = jsonArray[i]["nsecs"].get<uint32_t>();
        if(msgArray.isFixedSize())
        {
            msgArray.assign(i, time);
        }
        else
        {
            msgArray.push_back(time);
        }
    }
}

template<>
void fillArray<ros_babel_fish::CompoundArrayMessage>(
    const nlohmann::json& jsonArray, ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::CompoundArrayMessage>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.size());
    }

    for(size_t i = 0; i < jsonArray.size(); ++i)
    {
        if(msgArray.isFixedSize())
        {
            fillMessageFromJson(jsonArray[i],
                                msgArray[i].as<ros_babel_fish::CompoundMessage>());
        }
        else
        {
            auto& newItem = msgArray.appendEmpty();
            fillMessageFromJson(jsonArray[i],
                                newItem.as<ros_babel_fish::CompoundMessage>());
        }
    }
}

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const nlohmann::json& json);

} // namespace ros_nlohmann_converter
