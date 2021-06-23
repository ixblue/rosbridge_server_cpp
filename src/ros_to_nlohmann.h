#pragma once

#include <string>

#include <ros_babel_fish/babel_fish.h>

#include "nlohmann/json.hpp"

namespace ros_nlohmann_converter
{
nlohmann::json translatedMsgtoJson(const ros_babel_fish::Message& message);
nlohmann::json toJson(ros_babel_fish::BabelFish& fish,
                      const ros_babel_fish::BabelFishMessage& msg);
} // namespace ros_nlohman_converter
