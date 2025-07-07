#include "geo_transformer/srv/set_origin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>

class SetOriginNode : public rclcpp::Node {
public:
  SetOriginNode() : Node("set_origin_node") {
    service_ = this->create_service<geo_transformer::srv::SetOrigin>(
      "set_origin",
      std::bind(&SetOriginNode::handle_set_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
  }
private:
  void handle_set_origin(const std::shared_ptr<geo_transformer::srv::SetOrigin::Request> request,
                        std::shared_ptr<geo_transformer::srv::SetOrigin::Response> response) {
    // Just echo back the values for now, real logic can be added later
    response->success = true;
    response->message = "Origin set to: " + std::to_string(request->latitude) + ", " +
      std::to_string(request->longitude) + ", " + std::to_string(request->altitude);
  }
  rclcpp::Service<geo_transformer::srv::SetOrigin>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetOriginNode>());
  rclcpp::shutdown();
  return 0;
}
