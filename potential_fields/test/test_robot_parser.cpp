#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define COMPILE_ROBOT_PARSER_NO_MAIN
#include "ros/robot_parser.hpp"

// Ensure rclcpp is initialized for tests that create nodes
namespace {
  struct RclcppTestInitializer {
    RclcppTestInitializer() {
      if (!rclcpp::ok()) {
        int argc = 0;
        rclcpp::init(argc, nullptr);
      }
    }
    ~RclcppTestInitializer() {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  } rclcpp_test_initializer;
} // namespace

class RobotParserTestHelper : public RobotParser {
public:
  RobotParserTestHelper() : RobotParser(true) { this->planningTFPrefix = "planning"; }
  using RobotParser::createPlanningRobotDescription;
  using RobotParser::buildCollisionCatalog;
  using RobotParser::obstacleFromCollisionObject;
  using RobotParser::robotModel;
};

// Minimal URDF with multiple geometry types
static const char* kTestURDF = R"(
<robot name="test_bot">
	<link name="base_link"/>
	<link name="link1">
		<collision name="c_box">
			<origin xyz="0 0 0" rpy="0 0 0"/>
			<geometry><box size="0.5 0.4 0.3"/></geometry>
		</collision>
	</link>
	<link name="link2">
		<collision>
			<origin xyz="0 0 0.1" rpy="0 0 0"/>
			<geometry><sphere radius="0.2"/></geometry>
		</collision>
	</link>
	<joint name="joint1" type="fixed">
		<parent link="base_link"/>
		<child link="link1"/>
	</joint>
	<joint name="joint2" type="revolute">
		<parent link="link1"/>
		<child link="link2"/>
		<limit lower="-1.0" upper="1.0" effort="5" velocity="2"/>
	</joint>
</robot>
)";

TEST(RobotParserTests, PlanningDescriptionPrefixesNames) {
  RobotParserTestHelper helper;
  std::string planning = helper.createPlanningRobotDescription(kTestURDF);
  // Original names shouldn't appear as attribute values un-prefixed; check a sampling.
  EXPECT_NE(std::string::npos, planning.find("planning_base_link"));
  EXPECT_NE(std::string::npos, planning.find("planning_joint1"));
  EXPECT_EQ(std::string::npos, planning.find("\"link1\"")); // raw unprefixed attribute pattern
  // Ensure double prefixing didn't occur.
  EXPECT_EQ(std::string::npos, planning.find("planning_planning_"));
}

TEST(RobotParserTests, BuildCollisionCatalogAssignsIds) {
  RobotParserTestHelper helper;
  ASSERT_TRUE(helper.robotModel.initString(kTestURDF));
  auto catalog = helper.buildCollisionCatalog(helper.robotModel, false);
  // Expect two collision objects (one per link that has geometry)
  EXPECT_EQ(2u, catalog.size());
  // IDs should contain link name and either provided collision name or generated one
  EXPECT_NE(std::string::npos, catalog[0].id.find("link1::"));
  EXPECT_NE(std::string::npos, catalog[1].id.find("link2::"));
}

TEST(RobotParserTests, ObstacleFromCollisionObjectParsesBox) {
  RobotParserTestHelper helper;
  urdf::Model m; ASSERT_TRUE(m.initString(kTestURDF));
  auto linkIt = m.links_.find("link1"); ASSERT_NE(linkIt, m.links_.end());
  auto linkPtr = linkIt->second;
  ASSERT_TRUE(!linkPtr->collision_array.empty() || linkPtr->collision);
  urdf::CollisionSharedPtr col = linkPtr->collision_array.empty() ? linkPtr->collision : linkPtr->collision_array[0];
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Quaterniond q(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
  auto obs = helper.obstacleFromCollisionObject("link1::c_box", *col, pos, q);
  EXPECT_EQ(obs.type, "Box");
  EXPECT_FLOAT_EQ(obs.pose.position.x, 1.0f);
  EXPECT_FLOAT_EQ(obs.length, 0.5f);
  EXPECT_FLOAT_EQ(obs.width, 0.4f);
  EXPECT_FLOAT_EQ(obs.height, 0.3f);
}
