#include "geo_transformer/srv/set_origin.hpp"
#include "geo_transformer/srv/from_ll.hpp"
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>


class SetOriginNode : public rclcpp::Node {
public:
  SetOriginNode() : Node("set_origin_node"), origin_set_(false), origin_lat_(0.0), origin_lon_(0.0), origin_alt_(0.0) {
    set_origin_service_ = this->create_service<geo_transformer::srv::SetOrigin>(
      "set_origin",
      std::bind(&SetOriginNode::handle_set_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
    from_ll_service_ = this->create_service<geo_transformer::srv::FromLL>(
      "from_ll",
      std::bind(&SetOriginNode::handle_from_ll, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  void handle_set_origin(const std::shared_ptr<geo_transformer::srv::SetOrigin::Request> request,
                        std::shared_ptr<geo_transformer::srv::SetOrigin::Response> response) {
    origin_lat_ = request->latitude;
    origin_lon_ = request->longitude;
    origin_alt_ = request->altitude;
    origin_set_ = true;
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(origin_lat_, origin_lon_, origin_alt_);
    response->success = true;
    response->message = "Origin set to: " + std::to_string(origin_lat_) + ", " +
      std::to_string(origin_lon_) + ", " + std::to_string(origin_alt_);
  }

  void handle_from_ll(const std::shared_ptr<geo_transformer::srv::FromLL::Request> request,
                      std::shared_ptr<geo_transformer::srv::FromLL::Response> response) {
    if (!origin_set_ || !local_cartesian_) {
      response->success = false;
      response->message = "Origin not set. Please call set_origin first.";
      return;
    }
    double x, y, z;
    try {
      local_cartesian_->Forward(request->latitude, request->longitude, request->altitude, x, y, z);
      response->x = x;
      response->y = y;
      response->z = z;
      response->success = true;
      response->message = "Transformation successful.";
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Transformation failed: ") + e.what();
    }
  }

  rclcpp::Service<geo_transformer::srv::SetOrigin>::SharedPtr set_origin_service_;
  rclcpp::Service<geo_transformer::srv::FromLL>::SharedPtr from_ll_service_;
  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetOriginNode>());
  rclcpp::shutdown();
  return 0;
}
