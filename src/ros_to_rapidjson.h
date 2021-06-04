#pragma once

#include <ros_babel_fish/babel_fish.h>

#include "rapidjson/document.h"

namespace ros_rapidjson_converter
{

void translatedMsgtoJson(const ros_babel_fish::Message& message, rapidjson::Value& out,
                         rapidjson::Document::AllocatorType& alloc);
void toJson(ros_babel_fish::BabelFish& fish, const ros_babel_fish::BabelFishMessage& msg,
            rapidjson::Value& doc, rapidjson::Document::AllocatorType& alloc);

} // namespace ros_rapidjson_converter
