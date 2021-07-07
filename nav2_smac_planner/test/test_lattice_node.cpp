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

#include <fstream>

#include "nav2_smac_planner/node_lattice.hpp"
#include "gtest/gtest.h"

using json = nlohmann::json;


TEST(ParserTest, test_lattice_node)
{
    std::cout << "Starting" << std::endl; 
    std::string filePath = "/home/josh/nav2_ws/src/navigation2/nav2_smac_planner/test/output.json";
    std::ifstream myJsonFile(filePath);

    ASSERT_TRUE(myJsonFile.is_open());
    json j; 
    myJsonFile >> j;


    nav2_smac_planner::LatticeMetadata metaData; 
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

    nav2_smac_planner::SearchInfo info;
    info.minimum_turning_radius = 1.2; 
    info.non_straight_penalty = 1; 
    info.change_penalty = 1; 
    info.reverse_penalty = 1; 
    info.cost_penalty = 1; 
    info.analytic_expansion_ratio = 1; 
    info.lattice_filepath = filePath;
    info.cache_obstacle_heuristic = false; 
    info.obstacle_heuristic_cost_weight = 1; 
    unsigned int size_x = 100; 


    // //Test neighbors for each bin 
    // std::cout << "Init motion model" << std::endl; 
    nav2_smac_planner::NodeLattice::initMotionModel(
        nav2_smac_planner::MotionModel::STATE_LATTICE,
        size_x,
        size_x,
        size_x,
        info);

    // std::cout << "Motion model inited" << std::endl; 
    // //Create a dummy node to get projections
    unsigned int index = 0; 
    nav2_smac_planner::NodeLattice currentNode(index);
    currentNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0,0,0));

    std::cout << "Getting projections" << std::endl; 
    nav2_smac_planner::MotionPoses projections =  nav2_smac_planner::NodeLattice::motion_table.getProjections(&currentNode);
    std::cout << "Got projections" << std::endl; 
    for(const auto &projection : projections)
    {
        std::cout << projection._x << " ";
        std::cout << projection._y << " "; 
        std::cout << projection._theta << " "; 
        std::cout << std::endl; 
    }
    currentNode.setPose(
        nav2_smac_planner::NodeHybrid::Coordinates(
            0.0f,
            0.0f,
            projections[1]._theta
        )
    );
    projections = nav2_smac_planner::NodeLattice::motion_table.getProjections(&currentNode);
    std::cout << " " << std::endl; 
    for(const auto &projection : projections)
    {
        std::cout << projection._x << " ";
        std::cout << projection._y << " "; 
        std::cout << projection._theta << " "; 
        std::cout << std::endl; 
    }

}