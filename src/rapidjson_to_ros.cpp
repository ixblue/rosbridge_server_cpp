#include "rapidjson_to_ros.h"

namespace ros_rapidjson_converter
{

void fillBoolArray(const rapidjson::Value::ConstArray& jsonArray,
                   ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<bool>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetBool());
    }
}

void fillUint64Array(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<uint64_t>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetUint64());
    }
}

void fillInt64Array(const rapidjson::Value::ConstArray& jsonArray,
                    ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<int64_t>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetInt64());
    }
}

void fillFloatArray(const rapidjson::Value::ConstArray& jsonArray,
                    ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<float>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetFloat());
    }
}

void fillDoubleArray(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<double>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetDouble());
    }
}

void fillStringArray(const rapidjson::Value::ConstArray& jsonArray,
                     ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<std::string>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        msgArray.assign(i, jsonArray[i].GetString());
    }
}

void fillTimeArray(const rapidjson::Value::ConstArray& jsonArray,
                   ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<ros::Time>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        ros::Time time;
        const auto timeObj = jsonArray[i].GetObject();
        time.sec = timeObj["secs"].GetUint();
        time.nsec = timeObj["nsecs"].GetUint();
        msgArray.assign(i, time);
    }
}

void fillDurationArray(const rapidjson::Value::ConstArray& jsonArray,
                       ros_babel_fish::ArrayMessageBase& baseArray)
{
    auto& msgArray = baseArray.as<ros_babel_fish::ArrayMessage<ros::Duration>>();
    assert(msgArray.length() >= jsonArray.Size());
    for(size_t i = 0; i < jsonArray.Size(); ++i)
    {
        ros::Duration time;
        const auto timeObj = jsonArray[i].GetObject();
        time.sec = timeObj["secs"].GetUint();
        time.nsec = timeObj["nsecs"].GetUint();
        msgArray.assign(i, time);
    }
}

void fillMessageFromJson(const rapidjson::Value& json,
                         ros_babel_fish::CompoundMessage& message)
{
    for(const auto& m : json.GetObject())
    {
        auto& val = message[m.name.GetString()];
        switch(val.type())
        {
        case ros_babel_fish::MessageTypes::Array: {
            const auto jsonArray = m.value.GetArray();
            auto& base =
                message[m.name.GetString()].as<ros_babel_fish::ArrayMessageBase>();

            const auto type = base.elementType();
            switch(type)
            {
            case ros_babel_fish::MessageTypes::None: break;
            case ros_babel_fish::MessageTypes::Bool:
                fillBoolArray(jsonArray, base.as<ros_babel_fish::ArrayMessage<bool>>());
                break;
            case ros_babel_fish::MessageTypes::UInt8:
                fillUintArray<uint8_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::UInt16:
                fillUintArray<uint16_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::UInt32:
                fillUintArray<uint32_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::UInt64:
                fillUint64Array(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Int8:
                fillIntArray<int8_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Int16:
                fillIntArray<int16_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Int32:
                fillIntArray<int32_t>(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Int64:
                fillInt64Array(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Float32:
                fillFloatArray(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Float64:
                fillDoubleArray(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Time:
                fillTimeArray(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Duration:
                fillDurationArray(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::String:
                fillStringArray(jsonArray, base);
                break;
            case ros_babel_fish::MessageTypes::Array: // Arrays of arrays are actually
                                                      // not supported in the ROS msg
                                                      // format
            case ros_babel_fish::MessageTypes::Compound: {
                auto& array =
                    base.as<ros_babel_fish::ArrayMessage<ros_babel_fish::Message>>();
                for(size_t i = 0; i < jsonArray.Size(); ++i)
                {
                    fillMessageFromJson(jsonArray[i],
                                        array[i].as<ros_babel_fish::CompoundMessage>());
                }
                break;
            }
            }
            break;
        }
        case ros_babel_fish::MessageTypes::Compound: {
            auto& compound =
                message[m.name.GetString()].as<ros_babel_fish::CompoundMessage>();
            fillMessageFromJson(m.value, compound);
            break;
        }
        case ros_babel_fish::MessageTypes::None: break;
        case ros_babel_fish::MessageTypes::Bool: val = m.value.GetBool(); break;
        case ros_babel_fish::MessageTypes::UInt8:
        case ros_babel_fish::MessageTypes::UInt16:
        case ros_babel_fish::MessageTypes::UInt32:
        case ros_babel_fish::MessageTypes::UInt64: val = m.value.GetUint64(); break;
        case ros_babel_fish::MessageTypes::Int8:
        case ros_babel_fish::MessageTypes::Int16:
        case ros_babel_fish::MessageTypes::Int32:
        case ros_babel_fish::MessageTypes::Int64: val = m.value.GetInt64(); break;
        case ros_babel_fish::MessageTypes::Float32: val = m.value.GetFloat(); break;
        case ros_babel_fish::MessageTypes::Float64: val = m.value.GetDouble(); break;
        case ros_babel_fish::MessageTypes::Time: {
            ros::Time time;
            const auto timeObj = m.value.GetObject();
            time.sec = timeObj["secs"].GetUint();
            time.nsec = timeObj["nsecs"].GetUint();
            val = time;
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            ros::Time time;
            const auto timeObj = m.value.GetObject();
            time.sec = timeObj["secs"].GetUint();
            time.nsec = timeObj["nsecs"].GetUint();
            val = time;
            break;
        }
        case ros_babel_fish::MessageTypes::String: val = m.value.GetString(); break;
        }
    }
}

ros_babel_fish::BabelFishMessage::Ptr createMsg(ros_babel_fish::BabelFish& fish,
                                                const std::string& type,
                                                const ros::Time& time,
                                                const rapidjson::Value& json)
{
    ros_babel_fish::Message::Ptr message = fish.createMessage(type);
    auto& compound = message->as<ros_babel_fish::CompoundMessage>();
    fillMessageFromJson(json, compound);

    // From rosbridge protocol spec (3.4.3), if header is missing ou stamp in header is
    // missing, fill with current ROS time
    if(compound.containsKey("header"))
    {
        if(const auto it = json.FindMember("header");
           it == json.MemberEnd() || !it->value.HasMember("stamp"))
        {
            compound["header"]["stamp"] = time;
        }
    }

    return fish.translateMessage(message);
}

} // namespace ros_rapidjson_converter
