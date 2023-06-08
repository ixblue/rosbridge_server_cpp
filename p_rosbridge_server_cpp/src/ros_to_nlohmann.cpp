#include <QByteArray>
#include <sstream>

#include "ros_to_nlohmann.h"

using json = nlohmann::json;

namespace ros_nlohmann_converter
{

template<class T>
nlohmann::json
getBinaryJsonFromBabelFishArray(const ros_babel_fish::ArrayMessageBase& base,
                                std::int64_t subtype)
{
    size_t size = base.as<ros_babel_fish::ArrayMessage<T>>().length() * sizeof(T);
    const uint8_t* dataPtr = base.as<ros_babel_fish::ArrayMessage<T>>()._stream();
    std::vector<uint8_t> vec(dataPtr, dataPtr + size);

    return json::binary(vec, subtype);
}

nlohmann::json
getBinaryJsonFromBabelFishArrayUInt8(const ros_babel_fish::ArrayMessageBase& base)
{
    size_t size = base.as<ros_babel_fish::ArrayMessage<uint8_t>>().length();
    const uint8_t* dataPtr = base.as<ros_babel_fish::ArrayMessage<uint8_t>>()._stream();
    std::vector<uint8_t> vec(dataPtr, dataPtr + size);

    return json::binary(vec);
}

template<typename T>
nlohmann::json::array_t
getJsonArrayFromBabelFishArray(const ros_babel_fish::ArrayMessageBase& base)
{
    nlohmann::json::array_t out;
    out.reserve(base.length());
    for(size_t i = 0; i < base.length(); ++i)
    {
        out.emplace_back(base.as<ros_babel_fish::ArrayMessage<T>>()[i]);
    }
    return out;
}

template<typename T>
QByteArray getBase64FromBabelFishArray(const ros_babel_fish::ArrayMessageBase& base)
{
    size_t size = base.as<ros_babel_fish::ArrayMessage<T>>().length();
    const uint8_t* dataPtr = base.as<ros_babel_fish::ArrayMessage<T>>()._stream();
    QByteArray buffer = QByteArray::fromRawData(reinterpret_cast<const char*>(dataPtr),
                                                static_cast<int>(size));

    // QByteArray toBase64 seems to be slow, use alternatives
    // https://github.com/powturbo/Turbo-Base64 : fast but GPL
    // https://github.com/aklomp/base64 : BSD license
    return buffer.toBase64();
}

template<typename T> nlohmann::json getJsonFromTimeOrDuration(const T& t)
{
    return json{{"secs", t.sec}, {"nsecs", t.nsec}};
}

template<typename T>
nlohmann::json
getJsonFromTimeOrDurationBabelFishMessage(const ros_babel_fish::Message& message)
{
    const auto rosTime = message.value<T>();
    return getJsonFromTimeOrDuration(rosTime);
}

template<typename T>
nlohmann::json::array_t
getTimeJsonArrayFromBabelFishArray(const ros_babel_fish::ArrayMessageBase& base)
{
    nlohmann::json::array_t outArray;
    outArray.reserve(base.length());
    for(size_t i = 0; i < base.length(); ++i)
    {
        const auto t = base.as<ros_babel_fish::ArrayMessage<T>>()[i];
        outArray.emplace_back(getJsonFromTimeOrDuration(t));
    }
    return outArray;
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

        bool parsed = false;
        if(useBinary)
        {
            switch(base.elementType())
            {
            case ros_babel_fish::MessageTypes::UInt8:
                out = getBinaryJsonFromBabelFishArrayUInt8(base);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::UInt16:
                out = getBinaryJsonFromBabelFishArray<uint16_t>(base, 69);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::UInt32:
                out = getBinaryJsonFromBabelFishArray<uint32_t>(base, 70);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::UInt64:
                out = getBinaryJsonFromBabelFishArray<uint64_t>(base, 71);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Int8:
                out = getBinaryJsonFromBabelFishArray<int8_t>(base, 72);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Int16:
                out = getBinaryJsonFromBabelFishArray<int16_t>(base, 77);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Int32:
                out = getBinaryJsonFromBabelFishArray<int32_t>(base, 78);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Int64:
                out = getBinaryJsonFromBabelFishArray<int64_t>(base, 79);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Float32:
                out = getBinaryJsonFromBabelFishArray<float>(base, 85);
                parsed = true;
                break;
            case ros_babel_fish::MessageTypes::Float64:
                out = getBinaryJsonFromBabelFishArray<double>(base, 86);
                parsed = true;
                break;
            default: parsed = false; break;
            }
        }

        if(!parsed)
        {
            if(Q_UNLIKELY(base.length() == 0))
            {
                // Special case for empty array
                out = json::array();
            }
            else
            {
                switch(base.elementType())
                {
                case ros_babel_fish::MessageTypes::None: break;
                case ros_babel_fish::MessageTypes::UInt8:
                    // Special case for uint8[], encode as base64 string
                    out = getBase64FromBabelFishArray<uint8_t>(base);
                    out.setAlreadyEscapedString(true);
                    break;
                case ros_babel_fish::MessageTypes::Int8:
                    // Special case for int8[], encode as base64 string
                    out = getBase64FromBabelFishArray<int8_t>(base);
                    out.setAlreadyEscapedString(true);
                    break;
                case ros_babel_fish::MessageTypes::Bool:
                    out = getJsonArrayFromBabelFishArray<bool>(base);
                    break;
                case ros_babel_fish::MessageTypes::UInt16:
                    out = getJsonArrayFromBabelFishArray<uint16_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::UInt32:
                    out = getJsonArrayFromBabelFishArray<uint32_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::UInt64:
                    out = getJsonArrayFromBabelFishArray<uint64_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::Int16:
                    out = getJsonArrayFromBabelFishArray<int16_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::Int32:
                    out = getJsonArrayFromBabelFishArray<int32_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::Int64:
                    out = getJsonArrayFromBabelFishArray<int64_t>(base);
                    break;
                case ros_babel_fish::MessageTypes::Float32:
                    out = getJsonArrayFromBabelFishArray<float>(base);
                    break;
                case ros_babel_fish::MessageTypes::Float64:
                    out = getJsonArrayFromBabelFishArray<double>(base);
                    break;
                case ros_babel_fish::MessageTypes::String:
                    out = getJsonArrayFromBabelFishArray<std::string>(base);
                    break;
                case ros_babel_fish::MessageTypes::Time:
                {
                    out = getTimeJsonArrayFromBabelFishArray<ros::Time>(base);
                    break;
                }
                case ros_babel_fish::MessageTypes::Duration:
                {
                    out = getTimeJsonArrayFromBabelFishArray<ros::Duration>(base);
                    break;
                }
                case ros_babel_fish::MessageTypes::Array:
                    // Arrays of arrays are not supported in the ROS msg format
                    break;
                case ros_babel_fish::MessageTypes::Compound:
                {
                    nlohmann::json::array_t outArray;
                    outArray.reserve(base.length());
                    for(size_t i = 0; i < base.length(); ++i)
                    {
                        const auto& array =
                            base.as<ros_babel_fish::CompoundArrayMessage>()[i];
                        outArray.push_back(translatedMsgtoJson(array, useBinary));
                    }
                    out = outArray;
                    break;
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
        case ros_babel_fish::MessageTypes::Time:
            out = getJsonFromTimeOrDurationBabelFishMessage<ros::Time>(message);
            break;
        case ros_babel_fish::MessageTypes::Duration:
            out = getJsonFromTimeOrDurationBabelFishMessage<ros::Duration>(message);
            break;
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

std::string dumpJson(const nlohmann::json& j)
{
    return j.dump(-1, ' ', false, nlohmann::json::basic_json::error_handler_t::replace);
}

} // namespace ros_nlohmann_converter
