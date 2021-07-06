#include "nlohmann_to_ros.h"

using json = nlohmann::json;

namespace ros_nlohmann_converter
{

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

void fillMessageFromJson(const nlohmann::json& json,
                         ros_babel_fish::CompoundMessage& message)
{
    for(const auto& m : json.items())
    {
        auto& val = message[m.key()];
        switch(val.type())
        {
        case ros_babel_fish::MessageTypes::Array: {
            auto& base = val.as<ros_babel_fish::ArrayMessageBase>();
            const auto type = base.elementType();
            switch(type)
            {
            case ros_babel_fish::MessageTypes::None: break;
            case ros_babel_fish::MessageTypes::Bool:
                fillArray<bool>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::UInt8:
                fillArray<uint8_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::UInt16:
                fillArray<uint16_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::UInt32:
                fillArray<uint32_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::UInt64:
                fillArray<uint64_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Int8:
                fillArray<int8_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Int16:
                fillArray<int16_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Int32:
                fillArray<int32_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Int64:
                fillArray<int64_t>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Float32:
                fillArray<float>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Float64:
                fillArray<double>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Time:
                fillArray<ros::Time>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Duration:
                fillArray<ros::Duration>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::String:
                fillArray<std::string>(m.value(), base);
                break;
            case ros_babel_fish::MessageTypes::Array:
                // Arrays of arrays are actually not supported in the ROS msg format
                break;
            case ros_babel_fish::MessageTypes::Compound: {
                fillArray<ros_babel_fish::CompoundArrayMessage>(m.value(), base);
                break;
            }
            }
            break;
        }
        case ros_babel_fish::MessageTypes::Compound: {
            fillMessageFromJson(m.value(), val.as<ros_babel_fish::CompoundMessage>());
            break;
        }
        case ros_babel_fish::MessageTypes::None: break;
        case ros_babel_fish::MessageTypes::Bool: val = m.value().get<bool>(); break;
        case ros_babel_fish::MessageTypes::UInt8: val = m.value().get<uint8_t>(); break;
        case ros_babel_fish::MessageTypes::UInt16: val = m.value().get<uint16_t>(); break;
        case ros_babel_fish::MessageTypes::UInt32: val = m.value().get<uint32_t>(); break;
        case ros_babel_fish::MessageTypes::UInt64: val = m.value().get<uint64_t>(); break;
        case ros_babel_fish::MessageTypes::Int8: val = m.value().get<int8_t>(); break;
        case ros_babel_fish::MessageTypes::Int16: val = m.value().get<int16_t>(); break;
        case ros_babel_fish::MessageTypes::Int32: val = m.value().get<int32_t>(); break;
        case ros_babel_fish::MessageTypes::Int64: val = m.value().get<int64_t>(); break;
        case ros_babel_fish::MessageTypes::Float32:
            try
            {
                val = m.value().get<float>();
            }
            catch(const nlohmann::detail::type_error& e)
            {
                (void)e;
                val = std::numeric_limits<float>::quiet_NaN();
            }
            break;
        case ros_babel_fish::MessageTypes::Float64:
            try
            {
                val = m.value().get<double>();
            }
            catch(const nlohmann::detail::type_error& e)
            {
                (void)e;
                val = std::numeric_limits<double>::quiet_NaN();
            }
            break;
        case ros_babel_fish::MessageTypes::Time: {
            ros::Time time;
            time.sec = m.value()["secs"].get<uint32_t>();
            time.nsec = m.value()["nsecs"].get<uint32_t>();
            val = time;
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            ros::Time time;
            time.sec = m.value()["secs"].get<uint32_t>();
            time.nsec = m.value()["nsecs"].get<uint32_t>();
            val = time;
            break;
        }
        case ros_babel_fish::MessageTypes::String:
            val = m.value().get<std::string>();
            break;
        }
    }
}

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const nlohmann::json& json)
{
    ros_babel_fish::Message::Ptr message = fish.createMessage(type);
    auto& compound = message->as<ros_babel_fish::CompoundMessage>();
    fillMessageFromJson(json, compound);

    // From rosbridge protocol spec (3.4.3), if header is missing ou stamp in header is
    // missing, fill with current ROS time
    if(compound.containsKey("header"))
    {
        if(const auto it = json.find("header");
           it == json.end() || !it->contains("stamp"))
        {
            compound["header"]["stamp"] = time;
        }
    }

    return fish.translateMessage(message);
}

} // namespace ros_nlohmann_converter
