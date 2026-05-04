#include "imageProjection.h"
#include "featureAssociation.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(false);  // non-blocking: cloudHandler 不会因 mcl_fa 未消费而阻塞
  auto IP = std::make_shared<ImageProjection>("mcl_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("mcl_fa", projection_out_channel, association_out_channel);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(IP);
  executor.add_node(FA);
  IP->tfInitial();
  FA->tfInitial();
  executor.spin();

  rclcpp::shutdown();

  return 0;
}


