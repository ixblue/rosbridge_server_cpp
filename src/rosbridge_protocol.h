#pragma once

#include <map>
#include <string>

#include "rapidjson/document.h"

namespace rosbridge_protocol
{

enum class StatusLevel
{
    None,
    Info,
    Warning,
    Error
};

enum class Encoding
{
    JSON,
    CBOR,
    CBOR_RAW,
    PNG, // Not-implemented
};

const std::map<StatusLevel, std::string> statusLevelStringMap{
    {StatusLevel::None, "none"},
    {StatusLevel::Info, "info"},
    {StatusLevel::Warning, "warning"},
    {StatusLevel::Error, "error"}};

struct AdvertiseArgs
{
    std::string id;
    std::string topic;
    std::string type;
    unsigned int queueSize = 10;
    bool latched = false;
};

struct UnadvertiseArgs
{
    std::string id;
    std::string topic;
};

struct PublishArgs
{
    std::string id;
    std::string topic;
    // Not easy to do deep-copy (need allocator) so for now, the json value is passed by
    // reference next to this struct rapidjson::Value msg;
};

struct SubscribeArgs
{
    std::string id;
    std::string topic;
    std::string type;
    unsigned int queueSize = 10;
    int throttleRate = 0;
    int fragmentSize;
    std::string compression;
};

struct UnsubscribeArgs
{
    std::string id;
    std::string topic;
};

struct CallServiceArgs
{
    std::string id;
    std::string serviceName;
    std::string serviceType;
    // Not easy to do deep-copy (need allocator) so for now, the json value is passed by
    // reference next to this struct
    // rapidjson::Value args;
    int fragmentSize;
    std::string compression;
};

struct ServiceResponseArgs
{
    std::string id;
    std::string serviceName;
    rapidjson::Value values;
    bool result;
};

} // namespace rosbridge_protocol
