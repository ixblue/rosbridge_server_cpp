#pragma once

#include <string>

#include <ros_babel_fish/babel_fish.h>

#include "nlohmann/json.hpp"

namespace ros_nlohmann_converter
{
nlohmann::json translatedMsgtoJson(const ros_babel_fish::Message& message,
                                   bool useBinary);
nlohmann::json toJson(ros_babel_fish::BabelFish& fish,
                      const ros_babel_fish::BabelFishMessage& msg);
nlohmann::json toBinaryJson(ros_babel_fish::BabelFish& fish,
                            const ros_babel_fish::BabelFishMessage& msg);
/*!
 * \brief dumpJson will configure intendation and utf8 settings as required
 * we will replace invalid UTF8 characters instead of throwing an exception
 * \param j input json object
 * \return std::string with utf8 json text
 */
std::string dumpJson(const nlohmann::json& j);

} // namespace ros_nlohman_converter
