#include "nadie_control/wimble_robotics_motor_controller.h"
// Bring in gtest
#include <gtest/gtest.h>


// Declare a test
TEST(TestSuite, testCase1)
{
	ros::NodeHandle nh;
	WimbleRoboticsMotorController* c = new WimbleRoboticsMotorController(nh);
	ASSERT_TRUE(c);
}

// // Declare another test
// TEST(TestSuite, testCase2)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}