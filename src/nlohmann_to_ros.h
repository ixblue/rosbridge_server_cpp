#pragma once

#include <cassert>
#include <string>

#include <ros/duration.h>
#include <ros/time.h>
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
                          ros_babel_fish::ArrayMessageBase& baseArray);
template<>
void fillArray<ros::Duration>(const nlohmann::json& jsonArray,
                              ros_babel_fish::ArrayMessageBase& baseArray);
template<>
void fillArray<ros_babel_fish::CompoundArrayMessage>(
    const nlohmann::json& jsonArray, ros_babel_fish::ArrayMessageBase& baseArray);

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const nlohmann::json& json);

} // namespace ros_nlohmann_converter
