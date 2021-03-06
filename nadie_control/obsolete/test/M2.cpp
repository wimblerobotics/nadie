#include "nadie_control/motor_controller.h"
// Bring in gtest
#include <gtest/gtest.h>


// Declare a test
TEST(TestSuite_From_M2_cpp, testConstructorAndModelLoad)
{
	ros::NodeHandle nh;
	MotorController* c = new MotorController(nh);
	ASSERT_TRUE(c) << "Unable to construct a MotorController object";
	ASSERT_TRUE(c->modelLoaded()) << "Model failed to load";
	ASSERT_EQ(c->getTransmissions().size(), 2) << "Wrong number of transmissions found";
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