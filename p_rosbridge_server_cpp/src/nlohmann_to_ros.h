#pragma once

#include <QByteArray>

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

void fillCompoundArray(const nlohmann::json& jsonArray,
                       ros_babel_fish::CompoundArrayMessage& msgArray);

template<typename T, typename U>
void fillArrayLoop(const U& jsonArray, ros_babel_fish::ArrayMessage<T>& msgArray,
                   bool isFixedSize)
{
    for(size_t i = 0; i < static_cast<size_t>(jsonArray.size()); ++i)
    {
        T val;
        if constexpr(std::is_same_v<T, ros::Time>)
        {
            val.sec = jsonArray[i]["secs"].template get<uint32_t>();
            val.nsec = jsonArray[i]["nsecs"].template get<uint32_t>();
        }
        else if constexpr(std::is_same_v<T, ros::Duration>)
        {
            val.sec = jsonArray[i]["secs"].template get<int32_t>();
            val.nsec = jsonArray[i]["nsecs"].template get<int32_t>();
        }
        else
        {
            try
            {
                // cast for the case when jsonArray is QByteArray (base64 decoding)
                val = jsonArray[static_cast<unsigned int>(i)];
            }
            catch(const nlohmann::detail::type_error& e)
            {
                (void)e;
                // quiet_NaN produces 0 for integer types, this is ok to convert
                // JSON null to 0
                val = std::numeric_limits<T>::quiet_NaN();
            }
        }

        if(isFixedSize)
        {
            msgArray.assign(i, val);
        }
        else
        {
            msgArray.push_back(val);
        }
    }
}

template<typename T>
void fillArray(const nlohmann::json& jsonArray,
               ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<T>>();
    if(msgArray.isFixedSize())
    {
        assert(msgArray.length() == jsonArray.size());
    }

    if constexpr(std::is_same_v<T, uint8_t> || std::is_same_v<T, int8_t>)
    {
        if(jsonArray.is_string())
        {
            // Special case is encoded as base64
            const QByteArray b64Data =
                QByteArray::fromStdString(jsonArray.get<std::string>());
            const QByteArray data = QByteArray::fromBase64(b64Data);
            fillArrayLoop(data, msgArray, msgArray.isFixedSize());
        }
        else
        {
            fillArrayLoop(jsonArray, msgArray, msgArray.isFixedSize());
        }
    }
    else
    {
        fillArrayLoop(jsonArray, msgArray, msgArray.isFixedSize());
    }
}

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& rosTime,
                                                const nlohmann::json& json,
                                                bool latched = false);

} // namespace ros_nlohmann_converter
