#include <QByteArray>

#include "ros_to_nlohmann.h"

using json = nlohmann::json;

namespace ros_nlohmann_converter
{

template<class T>
nlohmann::json
getBinaryJsonFromBabelFishArray(const ros_babel_fish::ArrayMessageBase& base,
                                std::int64_t subtype)
{
    size_t size = base.as<ros_babel_fish::ArrayMessage<T>>()._sizeInBytes();
    const uint8_t* dataPtr = base.as<ros_babel_fish::ArrayMessage<T>>()._stream();
    std::vector<uint8_t> vec(dataPtr, dataPtr + size);

    return json::binary(vec, subtype);
}

nlohmann::json
getBinaryJsonFromBabelFishArrayUInt8(const ros_babel_fish::ArrayMessageBase& base)
{
    size_t size = base.as<ros_babel_fish::ArrayMessage<uint8_t>>()._sizeInBytes();
    const uint8_t* dataPtr = base.as<ros_babel_fish::ArrayMessage<uint8_t>>()._stream();
    std::vector<uint8_t> vec(dataPtr, dataPtr + size);

    return json::binary(vec);
}

nlohmann::json translatedMsgtoJson(const ros_babel_fish::Message& message, bool useBinary)
{
    nlohmann::json out;
    if(message.type() == ros_babel_fish::MessageTypes::Compound)
    {
        const auto& compound = message.as<ros_babel_fish::CompoundMessage>();
        for(size_t i = 0; i < compound.keys().size(); ++i)
        {
            const auto& keyStr = compound.keys()[i];
            out[keyStr] = translatedMsgtoJson(*compound.values()[i], useBinary);
        }
    }
    else if(message.type() == ros_babel_fish::MessageTypes::Array)
    {
        const auto& base = message.as<ros_babel_fish::ArrayMessageBase>();

        // in CBOR, some types can be converted to a uint8 array with specific tags
        // see :
        // https://github.com/RobotWebTools/rosbridge_suite/blob/ef0e4172667068b4c582a7277fe0b3071fa06aaf/rosbridge_library/src/rosbridge_library/internal/cbor_conversion.py

        if(useBinary && (base.elementType() == ros_babel_fish::MessageTypes::UInt8 ||
                         base.elementType() == ros_babel_fish::MessageTypes::UInt16 ||
                         base.elementType() == ros_babel_fish::MessageTypes::UInt32 ||
                         base.elementType() == ros_babel_fish::MessageTypes::UInt64 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Int8 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Int16 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Int32 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Int64 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Float32 ||
                         base.elementType() == ros_babel_fish::MessageTypes::Float64))
        {
            switch(base.elementType())
            {
            case ros_babel_fish::MessageTypes::UInt8:
                out = getBinaryJsonFromBabelFishArrayUInt8(base);
                break;
            case ros_babel_fish::MessageTypes::UInt16:
                out = getBinaryJsonFromBabelFishArray<uint16_t>(base, 69);
                break;
            case ros_babel_fish::MessageTypes::UInt32:
                out = getBinaryJsonFromBabelFishArray<uint32_t>(base, 70);
                break;
            case ros_babel_fish::MessageTypes::UInt64:
                out = getBinaryJsonFromBabelFishArray<uint64_t>(base, 71);
                break;
            case ros_babel_fish::MessageTypes::Int8:
                out = getBinaryJsonFromBabelFishArray<int8_t>(base, 72);
                break;
            case ros_babel_fish::MessageTypes::Int16:
                out = getBinaryJsonFromBabelFishArray<int16_t>(base, 77);
                break;
            case ros_babel_fish::MessageTypes::Int32:
                out = getBinaryJsonFromBabelFishArray<int32_t>(base, 78);
                break;
            case ros_babel_fish::MessageTypes::Int64:
                out = getBinaryJsonFromBabelFishArray<int64_t>(base, 79);
                break;
            case ros_babel_fish::MessageTypes::Float32:
                out = getBinaryJsonFromBabelFishArray<float>(base, 85);
                break;
            case ros_babel_fish::MessageTypes::Float64:
                out = getBinaryJsonFromBabelFishArray<double>(base, 86);
                break;
            default:
                throw std::runtime_error(
                    "unimplemented cbor elementType: this is a programming error");
                break;
            }
        }
        else
        {
            if(base.length() == 0)
            {
                // Special case for empty array
                out = json::array();
            }
            else
            {
                if(base.elementType() == ros_babel_fish::MessageTypes::UInt8 ||
                   base.elementType() == ros_babel_fish::MessageTypes::Int8)
                {
                    // Special case for uint8[] and int8[] types, encode as base64 string
                    QByteArray buffer;
                    for(size_t i = 0; i < base.length(); ++i)
                    {
                        if(base.elementType() == ros_babel_fish::MessageTypes::UInt8)
                        {
                            buffer.push_back(static_cast<int8_t>(
                                base.as<ros_babel_fish::ArrayMessage<uint8_t>>()[i]));
                        }
                        else
                        {
                            buffer.push_back(
                                base.as<ros_babel_fish::ArrayMessage<int8_t>>()[i]);
                        }
                    }
                    out = buffer.toBase64();
                }
                else
                {
                    for(size_t i = 0; i < base.length(); ++i)
                    {
                        // Switch for each case is not optimal at all
                        switch(base.elementType())
                        {
                        case ros_babel_fish::MessageTypes::None:
                        case ros_babel_fish::MessageTypes::UInt8:
                        case ros_babel_fish::MessageTypes::Int8: break;
                        case ros_babel_fish::MessageTypes::Bool:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<bool>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::UInt16:
                            out.push_back(static_cast<unsigned int>(
                                base.as<ros_babel_fish::ArrayMessage<uint16_t>>()[i]));

                            break;
                        case ros_babel_fish::MessageTypes::UInt32:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<uint32_t>>()[i]);

                            break;
                        case ros_babel_fish::MessageTypes::UInt64:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<uint64_t>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::Int16:
                            out.push_back(static_cast<int>(
                                base.as<ros_babel_fish::ArrayMessage<int16_t>>()[i]));

                            break;
                        case ros_babel_fish::MessageTypes::Int32:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<int32_t>>()[i]);

                            break;
                        case ros_babel_fish::MessageTypes::Int64:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<int64_t>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::Float32:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<float>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::Float64:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<double>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::Time: {
                            const ros::Time& rosTime =
                                base.as<ros_babel_fish::ArrayMessage<ros::Time>>()[i];
                            out.push_back(
                                json{{"secs", rosTime.sec}, {"nsecs", rosTime.nsec}});
                            break;
                        }
                        case ros_babel_fish::MessageTypes::Duration: {
                            const ros::Duration& rosTime =
                                base.as<ros_babel_fish::ArrayMessage<ros::Duration>>()[i];
                            out.push_back(
                                json{{"secs", rosTime.sec}, {"nsecs", rosTime.nsec}});
                            break;
                        }
                        case ros_babel_fish::MessageTypes::String:
                            out.push_back(
                                base.as<ros_babel_fish::ArrayMessage<std::string>>()[i]);
                            break;
                        case ros_babel_fish::MessageTypes::Array:
                            // Arrays of arrays are actually not supported in the ROS msg
                            // format
                            break;
                        case ros_babel_fish::MessageTypes::Compound: {
                            const auto& array =
                                base.as<ros_babel_fish::CompoundArrayMessage>()[i];
                            out.push_back(translatedMsgtoJson(array, useBinary));
                            break;
                        }
                        }
                    }
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
            const auto rosTtime = message.value<ros::Time>();
            out = json{{"secs", rosTtime.sec}, {"nsecs", rosTtime.nsec}};
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            const auto rosTtime = message.value<ros::Duration>();
            out = json{{"secs", rosTtime.sec}, {"nsecs", rosTtime.nsec}};
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
    return translatedMsgtoJson(*babelFishMsg, false);
}

nlohmann::json toBinaryJson(ros_babel_fish::BabelFish& fish,
                            const ros_babel_fish::BabelFishMessage& msg)
{
    ros_babel_fish::Message::Ptr babelFishMsg = fish.translateMessage(msg);
    return translatedMsgtoJson(*babelFishMsg, true);
}

} // namespace ros_nlohman_converter
