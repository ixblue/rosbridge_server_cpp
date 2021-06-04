#pragma once

#include <map>
#include <string>

namespace rosbridge_protocol
{

enum class StatusLevel
{
    None,
    Info,
    Warning,
    Error
};

const std::map<StatusLevel, std::string> statusLevelStringMap{
    {StatusLevel::None, "none"},
    {StatusLevel::Info, "info"},
    {StatusLevel::Warning, "warning"},
    {StatusLevel::Error, "error"}};

} // namespace rosbridge_protocol
