/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

#include "tnp_task_manager.h"
#include <gtest/gtest.h>

using namespace std;


//// -------- easy way using "TEST" ----------------------------- //
///  need to create your environment on your own all the time --- //

/**
 * Tests that the tnpTaskManager::testMethod() method does on negative values.
 */
TEST(testMethodTest, HandlesNegativeValues)
{
  TaskManagerNode node;

  EXPECT_EQ(false, node.testMethod(-1, -2));
  EXPECT_EQ(true, node.testMethod(-1, -1));
  EXPECT_NE(true, node.testMethod(-2, -1));
}



/// ----------- complex way using TEST_F ----------------------- //
///  build up your environment once, reuse it easily next time.  //

/**
 * The fixture class for testing class tnpTaskManager.
 */
class TnpTaskManagerTest : public ::testing::Test
{

protected:

  // Objects declared here can be used by all tests in the test case for tnpTaskManager.cpp.

  // instance the be tested
  TaskManagerNode instance;
  double a;

  TnpTaskManagerTest()
  {
    a = 42;
    // You can do set-up work for each test here.
  }

  virtual ~TnpTaskManagerTest()
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  /** If the constructor and destructor are not enough for setting up
   / and cleaning up each test, you can define the following methods:
   */
  virtual void SetUp()
  {
    // Code here will be called immediately after the constructor (and right
    // before each test).
  }

  virtual void TearDown()
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

}; /* fixture class end */


/**
 * Tests that the tnpTaskManager::testMethod() method does on negative values.
 */
TEST_F(TnpTaskManagerTest, testMethodOnNegativeValues)
{
  EXPECT_EQ(false, instance.testMethod(-1, -2));
  EXPECT_EQ(false, instance.testMethod(-1, -5));
  EXPECT_EQ(false, instance.testMethod(-8, -2));
  EXPECT_EQ(true, instance.testMethod(-2, -2));
}

/**
 * Tests that the tnpTaskManager::testMethod() method does on uneven values.
 */
TEST_F(TnpTaskManagerTest, testMethodTestUnevenValues)
{
  EXPECT_EQ(false, instance.testMethod(1, 3));
  EXPECT_EQ(false, instance.testMethod(1, 5));
}

/**
 * Tests that the tnpTaskManager::testMethod() method does on same values.
 */
TEST_F(TnpTaskManagerTest, testOnEqualNUmbers)
{
  EXPECT_EQ(true, instance.testMethod(1, 1));
  EXPECT_EQ(true, instance.testMethod(2, 2));
  EXPECT_EQ(true, instance.testMethod(5, 5));
}


/**
 * Need to be here to execute the tests in this file
 * @param argc
 * @param argv
 * @return
 */
GTEST_API_ int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_node");

  return RUN_ALL_TESTS();
}
