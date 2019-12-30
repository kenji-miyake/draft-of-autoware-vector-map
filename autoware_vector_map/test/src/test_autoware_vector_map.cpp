#include <gtest/gtest.h>

#include <ros/ros.h>

#include <autoware_vector_map/io/gpkg_interface.h>

using autoware_vector_map::io::GpkgInterface;

class TestSuite : public ::testing::Test {
 public:
  TestSuite() : nh_(""), private_nh_("~") {}

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string vector_map_path_;

  void SetUp() override { private_nh_.getParam("vector_map_path", vector_map_path_); }

  void TearDown() override {}
};

TEST_F(TestSuite, toBinary) {
  GpkgInterface gpkg_interface(vector_map_path_.c_str());
  const auto bin_data = gpkg_interface.toBinary();

  ASSERT_EQ(bin_data.size(), 217088);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
