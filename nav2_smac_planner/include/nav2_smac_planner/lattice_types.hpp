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

#ifndef NAV2_SMAC_PLANNER__LATTICE_TYPES_HPP_
#define NAV2_SMAC_PLANNER__LATTICE_TYPES_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "nav2_smac_planner/node_hybrid.hpp" // Motion Pose struct 
namespace nav2_smac_planner
{
    struct LatticeMetadata
    {
        float turningRadius;
        float stepDistance;
        float gridSeparation;
        float maxLength; 
        unsigned int numberOfHeadings; 
        std::string outputFile; 
        std::vector<float> headingAngles; 
        unsigned int numberOfTrajectories; 
    };

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

    typedef std::vector<Primitive> Primitives;
}

#endif //NAV2_SMAC_PLANNER__LATTICE_TYPES_HPP_

