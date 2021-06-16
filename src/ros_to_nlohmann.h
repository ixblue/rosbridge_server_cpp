#pragma once

#include <string>

#include <ros_babel_fish/babel_fish.h>

#include "nlohmann/json.hpp"

namespace ros_nlohmann_converter
{
void translatedMsgtoJson(const ros_babel_fish::Message& message, nlohmann::json& out);
nlohmann::json toJson(ros_babel_fish::BabelFish& fish,
                      const ros_babel_fish::BabelFishMessage& msg);

std::string jsonToString(const nlohmann::json& json);
} // namespace ros_nlohman_converter
