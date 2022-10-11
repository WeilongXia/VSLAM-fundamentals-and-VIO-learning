#include "hello.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_int32(print_times, 1, "Number of times to print Hello SLAM");

TEST(sayHelloTest, NonInput)
{
  EXPECT_TRUE(sayHello());
}

int main( int argc, char** argv) 
{
  FLAGS_logtostderr = 1;

  // Initialize Google's logging library
  google::InitGoogleLogging(argv[0]);

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  testing::InitGoogleTest(&argc, argv);

  for(int i = 1; i < FLAGS_print_times; i++)
  {
    sayHello();
  }

  return RUN_ALL_TESTS();
}
