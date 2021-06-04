#include "ros_to_rapidjson.h"

namespace ros_rapidjson_converter
{

void translatedMsgtoJson(const ros_babel_fish::Message& message, rapidjson::Value& out,
                         rapidjson::Document::AllocatorType& alloc)
{
    if(message.type() == ros_babel_fish::MessageTypes::Compound)
    {
        out.SetObject();
        const auto& compound = message.as<ros_babel_fish::CompoundMessage>();
        for(size_t i = 0; i < compound.keys().size(); ++i)
        {
            const auto& keyStr = compound.keys()[i];
            const auto key = rapidjson::StringRef(keyStr.data(), keyStr.size());
            rapidjson::Value jsonVal;
            jsonVal.SetObject();
            translatedMsgtoJson(*compound.values()[i], jsonVal, alloc);
            out.AddMember(key, jsonVal, alloc);
        }
    }
    else if(message.type() == ros_babel_fish::MessageTypes::Array)
    {
        out.SetArray();
        const auto& base = message.as<ros_babel_fish::ArrayMessageBase>();
        out.Reserve(base.length(), alloc);
        for(size_t i = 0; i < base.length(); ++i)
        {
            // Switch for each case is nt optimal at all
            switch(base.elementType())
            {
            case ros_babel_fish::MessageTypes::None: break;
            case ros_babel_fish::MessageTypes::Bool:
                out.PushBack(base.as<ros_babel_fish::ArrayMessage<bool>>()[i], alloc);
                break;
            case ros_babel_fish::MessageTypes::UInt8:
                out.PushBack(static_cast<unsigned int>(
                                 base.as<ros_babel_fish::ArrayMessage<uint8_t>>()[i]),
                             alloc);
                break;
            case ros_babel_fish::MessageTypes::UInt16:
                out.PushBack(static_cast<unsigned int>(
                                 base.as<ros_babel_fish::ArrayMessage<uint16_t>>()[i]),
                             alloc);
                break;
            case ros_babel_fish::MessageTypes::UInt32:
                out.PushBack(static_cast<unsigned int>(
                                 base.as<ros_babel_fish::ArrayMessage<uint32_t>>()[i]),
                             alloc);
                break;
            case ros_babel_fish::MessageTypes::UInt64:
                out.PushBack(base.as<ros_babel_fish::ArrayMessage<uint64_t>>()[i], alloc);
                break;
            case ros_babel_fish::MessageTypes::Int8:
                out.PushBack(
                    static_cast<int>(base.as<ros_babel_fish::ArrayMessage<int8_t>>()[i]),
                    alloc);
                break;
            case ros_babel_fish::MessageTypes::Int16:
                out.PushBack(
                    static_cast<int>(base.as<ros_babel_fish::ArrayMessage<int16_t>>()[i]),
                    alloc);
                break;
            case ros_babel_fish::MessageTypes::Int32:
                out.PushBack(
                    static_cast<int>(base.as<ros_babel_fish::ArrayMessage<int32_t>>()[i]),
                    alloc);
                break;
            case ros_babel_fish::MessageTypes::Int64:
                out.PushBack(base.as<ros_babel_fish::ArrayMessage<int64_t>>()[i], alloc);
                break;
            case ros_babel_fish::MessageTypes::Float32:
                out.PushBack(base.as<ros_babel_fish::ArrayMessage<float>>()[i], alloc);
                break;
            case ros_babel_fish::MessageTypes::Float64:
                out.PushBack(base.as<ros_babel_fish::ArrayMessage<double>>()[i], alloc);
                break;
            case ros_babel_fish::MessageTypes::Time: {
                const ros::Time rosTime =
                    base.as<ros_babel_fish::ArrayMessage<ros::Time>>()[i];
                rapidjson::Value obj;
                obj.SetObject();
                obj.AddMember("secs", rosTime.sec, alloc);
                obj.AddMember("nsecs", rosTime.nsec, alloc);
                out.PushBack(obj, alloc);
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                const ros::Duration rosTime =
                    base.as<ros_babel_fish::ArrayMessage<ros::Duration>>()[i];
                rapidjson::Value obj;
                obj.SetObject();
                obj.AddMember("secs", rosTime.sec, alloc);
                obj.AddMember("nsecs", rosTime.nsec, alloc);
                out.PushBack(obj, alloc);
                break;
            }
            case ros_babel_fish::MessageTypes::String: {
                rapidjson::Value jsonStr;
                const std::string str =
                    base.as<ros_babel_fish::ArrayMessage<std::string>>()[i];
                jsonStr.SetString(str, alloc);
                out.PushBack(jsonStr, alloc);
            }
            break;
            case ros_babel_fish::MessageTypes::Array: // Arrays of arrays are actually
                                                      // not supported in the ROS msg
                                                      // format
            case ros_babel_fish::MessageTypes::Compound: {
                const auto& array =
                    base.as<ros_babel_fish::ArrayMessage<ros_babel_fish::Message>>();
                rapidjson::Value jsonArray;
                jsonArray.SetArray();
                jsonArray.Reserve(array.length(), alloc);
                for(size_t i = 0; i < array.length(); ++i)
                {
                    rapidjson::Value jsonVal;
                    jsonVal.SetObject();
                    translatedMsgtoJson(array[i], jsonVal, alloc);
                    jsonArray.PushBack(jsonVal, alloc);
                }
                out.PushBack(jsonArray, alloc);
                break;
            }
            }
        }
    }
    else
    {
        out.SetObject();
        switch(message.type())
        {
        case ros_babel_fish::MessageTypes::Array:
        case ros_babel_fish::MessageTypes::Compound:
        case ros_babel_fish::MessageTypes::None: break;
        case ros_babel_fish::MessageTypes::Bool:
            out.SetBool(message.value<bool>());
            break;
        case ros_babel_fish::MessageTypes::UInt8:
            out.SetUint(message.value<uint8_t>());
            break;
        case ros_babel_fish::MessageTypes::UInt16:
            out.SetUint(message.value<uint16_t>());
            break;
        case ros_babel_fish::MessageTypes::UInt32:
            out.SetUint(message.value<uint32_t>());
            break;
        case ros_babel_fish::MessageTypes::UInt64:
            out.SetUint64(message.value<uint64_t>());
            break;
        case ros_babel_fish::MessageTypes::Int8:
            out.SetInt(message.value<int8_t>());
            break;
        case ros_babel_fish::MessageTypes::Int16:
            out.SetInt(message.value<int16_t>());
            break;
        case ros_babel_fish::MessageTypes::Int32:
            out.SetInt(message.value<int32_t>());
            break;
        case ros_babel_fish::MessageTypes::Int64:
            out.SetInt64(message.value<int64_t>());
            break;
        case ros_babel_fish::MessageTypes::Float32:
            out.SetFloat(message.value<float>());
            break;
        case ros_babel_fish::MessageTypes::Float64:
            out.SetDouble(message.value<double>());
            break;
        case ros_babel_fish::MessageTypes::Time: {
            rapidjson::Value obj;
            obj.SetObject();
            obj.AddMember("secs", message.value<ros::Time>().sec, alloc);
            obj.AddMember("nsecs", message.value<ros::Time>().nsec, alloc);
            out = obj;
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            rapidjson::Value obj;
            obj.SetObject();
            obj.AddMember("secs", message.value<ros::Duration>().sec, alloc);
            obj.AddMember("nsecs", message.value<ros::Duration>().nsec, alloc);
            out = obj;
            break;
        }
        case ros_babel_fish::MessageTypes::String:
            out.SetString(message.value<std::string>(), alloc);
            break;
        }
    }
}

void toJson(ros_babel_fish::BabelFish& fish, const ros_babel_fish::BabelFishMessage& msg,
            rapidjson::Value& doc, rapidjson::Document::AllocatorType& alloc)
{
    ros_babel_fish::Message::Ptr babelFishMsg = fish.translateMessage(msg);
    translatedMsgtoJson(*babelFishMsg, doc, alloc);
}

} // namespace ros_rapidjson_converter
