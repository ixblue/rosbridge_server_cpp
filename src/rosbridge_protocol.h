#pragma once

#include <map>
#include <string>

#include "nlohmann/json.hpp"

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
    nlohmann::json msg;
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
    nlohmann::json args;
    int fragmentSize;
    std::string compression;
};

struct ServiceResponseArgs
{
    std::string id;
    std::string serviceName;
    nlohmann::json values;
    bool result;
};

} // namespace rosbridge_protocol
