// Copyright (c) 2020, Samsung Research America
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

#include "nav2_smac_planner/motion_primative_parser.hpp"
#include "gtest/gtest.h"

TEST(ParserTest, test_parser)
{
    std::cout << "Starting" << std::endl; 
    std::ifstream myJsonFile("/home/josh/nav2_ws/src/navigation2/nav2_smac_planner/test/output.json");
    std::string line; 

    ASSERT_TRUE(myJsonFile.is_open());
    json j; 
    myJsonFile >> j;


    nav2_smac_planner::LatticeMetaData metaData; 
    nav2_smac_planner::Primitive myPrimitive; 
    nav2_smac_planner::MotionPose pose; 

    std::cout << "Created empty data types " << std::endl; 
    json jsonMetaData = j["latticeMetadata"];
    json jsonPrimatives = j["primitives"];
    json jsonPose = jsonPrimatives[0]["poses"][0];

    nav2_smac_planner::from_json_to_metaData(jsonMetaData, metaData);

    //Checks for parsing meta data 
    EXPECT_NEAR(metaData.turningRadius, 0.4, 0.001);
    EXPECT_NEAR(metaData.stepDistance, 0.005, 0.0001);
    EXPECT_NEAR(metaData.gridSeparation, 0.05, 0.001);
    EXPECT_NEAR(metaData.maxLength, 1, 0.01);
    EXPECT_NEAR(metaData.numberOfHeadings, 16, 0.01);
    EXPECT_EQ(metaData.outputFile, "output.json");
    EXPECT_NEAR(metaData.headingAngles[0], -180.0, 0.01);


    std::vector<nav2_smac_planner::Primitive> myPrimitives;
    for(auto& primative : jsonPrimatives)
    {
        nav2_smac_planner::Primitive newPrimative; 
        nav2_smac_planner::from_json_to_primitive(primative, newPrimative );
        myPrimitives.push_back(newPrimative);
    }

    //Checks for parsing primitives 
    EXPECT_NEAR(myPrimitives[0].trajectoryId, 0, 0.01);
    EXPECT_NEAR(myPrimitives[0].startAngle, 0.0,0.01);
    EXPECT_NEAR(myPrimitives[0].endAngle, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].radius, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].trajectoryLength, 0.2, 0.01);
    EXPECT_NEAR(myPrimitives[0].arcLength, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].straightLength, 0.2, 0.01);

    EXPECT_NEAR(myPrimitives[0].poses[0]._x, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[0]._y, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[0]._theta, 0.0, 0.01);

    EXPECT_NEAR(myPrimitives[0].poses[1]._x, 0.06667, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[1]._y, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[1]._theta, 0.0, 0.01);
}