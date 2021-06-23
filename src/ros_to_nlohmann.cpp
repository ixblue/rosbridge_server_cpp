#include "ros_to_nlohmann.h"

using json = nlohmann::json;

namespace ros_nlohmann_converter
{

nlohmann::json translatedMsgtoJson(const ros_babel_fish::Message& message)
{
    nlohmann::json out;
    if(message.type() == ros_babel_fish::MessageTypes::Compound)
    {
        const auto& compound = message.as<ros_babel_fish::CompoundMessage>();
        for(size_t i = 0; i < compound.keys().size(); ++i)
        {
            const auto& keyStr = compound.keys()[i];
            out[keyStr] = translatedMsgtoJson(*compound.values()[i]);
        }
    }
    else if(message.type() == ros_babel_fish::MessageTypes::Array)
    {
        const auto& base = message.as<ros_babel_fish::ArrayMessageBase>();
        for(size_t i = 0; i < base.length(); ++i)
        {
            // Switch for each case is nt optimal at all
            switch(base.elementType())
            {
            case ros_babel_fish::MessageTypes::None: break;
            case ros_babel_fish::MessageTypes::Bool:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<bool>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::UInt8:
                out.push_back(static_cast<unsigned int>(
                    base.as<ros_babel_fish::ArrayMessage<uint8_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::UInt16:
                out.push_back(static_cast<unsigned int>(
                    base.as<ros_babel_fish::ArrayMessage<uint16_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::UInt32:
                out.push_back(static_cast<unsigned int>(
                    base.as<ros_babel_fish::ArrayMessage<uint32_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::UInt64:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<uint64_t>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::Int8:
                out.push_back(
                    static_cast<int>(base.as<ros_babel_fish::ArrayMessage<int8_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::Int16:
                out.push_back(static_cast<int>(
                    base.as<ros_babel_fish::ArrayMessage<int16_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::Int32:
                out.push_back(static_cast<int>(
                    base.as<ros_babel_fish::ArrayMessage<int32_t>>()[i]));

                break;
            case ros_babel_fish::MessageTypes::Int64:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<int64_t>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::Float32:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<float>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::Float64:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<double>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::Time: {
                const ros::Time& rosTime =
                    base.as<ros_babel_fish::ArrayMessage<ros::Time>>()[i];
                out.push_back(json{{"secs", rosTime.sec}, {"nsecs", rosTime.nsec}});
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                const ros::Duration& rosTime =
                    base.as<ros_babel_fish::ArrayMessage<ros::Duration>>()[i];
                out.push_back(json{{"secs", rosTime.sec}, {"nsecs", rosTime.nsec}});
                break;
            }
            case ros_babel_fish::MessageTypes::String:
                out.push_back(base.as<ros_babel_fish::ArrayMessage<std::string>>()[i]);
                break;
            case ros_babel_fish::MessageTypes::Array:
                // Arrays of arrays are actually not supported in the ROS msg format
                break;
            case ros_babel_fish::MessageTypes::Compound: {
                const auto& array = base.as<ros_babel_fish::CompoundArrayMessage>()[i];
                out.push_back(translatedMsgtoJson(array));
                break;
            }
            }
        }
    }
    else
    {
        switch(message.type())
        {
        case ros_babel_fish::MessageTypes::Array:
        case ros_babel_fish::MessageTypes::Compound:
        case ros_babel_fish::MessageTypes::None: break;
        case ros_babel_fish::MessageTypes::Bool: out = message.value<bool>(); break;
        case ros_babel_fish::MessageTypes::UInt8: out = message.value<uint8_t>(); break;
        case ros_babel_fish::MessageTypes::UInt16: out = message.value<uint16_t>(); break;
        case ros_babel_fish::MessageTypes::UInt32: out = message.value<uint32_t>(); break;
        case ros_babel_fish::MessageTypes::UInt64: out = message.value<uint64_t>(); break;
        case ros_babel_fish::MessageTypes::Int8: out = message.value<int8_t>(); break;
        case ros_babel_fish::MessageTypes::Int16: out = message.value<int16_t>(); break;
        case ros_babel_fish::MessageTypes::Int32: out = message.value<int32_t>(); break;
        case ros_babel_fish::MessageTypes::Int64: out = message.value<int64_t>(); break;
        case ros_babel_fish::MessageTypes::Float32: out = message.value<float>(); break;
        case ros_babel_fish::MessageTypes::Float64: out = message.value<double>(); break;
        case ros_babel_fish::MessageTypes::Time: {
            const auto time = message.value<ros::Time>();
            out = json{{"secs", time.sec}, {"nsecs", time.nsec}};
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            const auto time = message.value<ros::Duration>();
            out = json{{"secs", time.sec}, {"nsecs", time.nsec}};
            break;
        }
        case ros_babel_fish::MessageTypes::String:
            out = message.value<std::string>();
            break;
        }
    }
    return out;
}

nlohmann::json toJson(ros_babel_fish::BabelFish& fish,
                      const ros_babel_fish::BabelFishMessage& msg)
{
    ros_babel_fish::Message::Ptr babelFishMsg = fish.translateMessage(msg);
    return translatedMsgtoJson(*babelFishMsg);
}

} // namespace ros_nlohman_converter
