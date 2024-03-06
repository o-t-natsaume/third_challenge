#include "third_challenge/third_challenge_natsume.hpp"

int main(int argc, char * argv[])
{
  printf("start\n");
  rclcpp::init(argc, argv);
  std::shared_ptr<ThirdChallenge> schallenge = std::make_shared<ThirdChallenge>();
  rclcpp::spin(schallenge);
  rclcpp::shutdown();

  return 0;
}
