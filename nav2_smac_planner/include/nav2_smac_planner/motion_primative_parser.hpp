// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__MOTION_PRIMATIVE_PARSER_HPP_
#define NAV2_SMAC_PLANNER__MOTION_PRIMATIVE_PARSER_HPP_

#include <iostream>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace nav2_smac_planner
{
  struct MotionPose
  {
    /**
     * @brief A constructor for nav2_smac_planner::MotionPose
     */
    MotionPose() {}

    /**
     * @brief A constructor for nav2_smac_planner::MotionPose
     * @param x X pose
     * @param y Y pose
     * @param theta Angle of pose
     */
    MotionPose(const float & x, const float & y, const float & theta)
    : _x(x), _y(y), _theta(theta)
    {}

    float _x;
    float _y;
    float _theta;
  };

      struct LatticeMetaData
    {
        float turningRadius;
        float stepDistance;
        float gridSeparation;
        float maxLength; 
        int numberOfHeadings; 
        std::string outputFile; 
        std::vector<float> headingAngles; 
        int numberOfTrajectories; 
    };

    void from_json_to_metaData(const json &j, LatticeMetaData &latticeMetaData)
    {
        j.at("turningRadius").get_to(latticeMetaData.turningRadius);
        j.at("stepDistance").get_to(latticeMetaData.stepDistance);
        j.at("gridSeparation").get_to(latticeMetaData.gridSeparation);
        j.at("maxLength").get_to(latticeMetaData.maxLength);
        j.at("numberOfHeadings").get_to(latticeMetaData.numberOfHeadings);
        j.at("outputFile").get_to(latticeMetaData.outputFile);
        j.at("headingAngles").get_to(latticeMetaData.headingAngles);
        j.at("numberOfTrajectories").get_to(latticeMetaData.numberOfTrajectories);
    }

    struct Primitive
    {
        unsigned int trajectoryId;
        float startAngle; 
        float endAngle; 
        float radius;
        float trajectoryLength; 
        float arcLength; 
        float straightLength; 
        std::vector<MotionPose> poses; 
    };

    void from_json_to_pose(const json &j, MotionPose &pose)
    {
        pose._x = j[0]; 
        pose._y = j[1]; 
        pose._theta = j[2];
    }

    void from_json_to_primitive(const json &j, Primitive &primitive)
    {
        j.at("trajectoryId").get_to(primitive.trajectoryId);
        j.at("startAngle").get_to(primitive.startAngle);
        j.at("endAngle").get_to(primitive.endAngle);   
        j.at("radius").get_to(primitive.radius);
        j.at("trajectoryLength").get_to(primitive.trajectoryLength);
        j.at("arcLength").get_to(primitive.arcLength);
        j.at("straightLength").get_to(primitive.straightLength);

        for(auto& jsonPose : j["poses"])
        {
            MotionPose pose;
            from_json_to_pose(jsonPose, pose);
            primitive.poses.push_back(pose);
        }
    }

}

#endif //NAV2_SMAC_PLANNER__MOTION_PRIMATIVE_PARSER_HPP_