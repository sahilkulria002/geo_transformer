#include "geo_transformer/srv/set_origin.hpp"      // Service for setting the origin
#include "geo_transformer/srv/get_origin.hpp"      // Service for getting the origin
#include "geo_transformer/srv/from_ll.hpp"         // Service for WGS84 to local conversion
#include "geo_transformer/srv/to_ll.hpp"           // Service for local to WGS84 conversion
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>

// Main node class for coordinate transformation services
class GeoTransformationNode : public rclcpp::Node {
public:
  GeoTransformationNode() : Node("geo_transformer_node"), origin_set_(false), origin_lat_(0.0), origin_lon_(0.0), origin_alt_(0.0) {
    // Advertise the /local_coordinate/set service
    set_origin_service_ = this->create_service<geo_transformer::srv::SetOrigin>(
      "/local_coordinate/set",
      std::bind(&GeoTransformationNode::handle_set_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Advertise the /local_coordinate/get service
    get_origin_service_ = this->create_service<geo_transformer::srv::GetOrigin>(
      "/local_coordinate/get",
      std::bind(&GeoTransformationNode::handle_get_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Advertise the /from_ll service
    from_ll_service_ = this->create_service<geo_transformer::srv::FromLL>(
      "/from_ll",
      std::bind(&GeoTransformationNode::handle_from_ll, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Advertise the /to_ll service
    to_ll_service_ = this->create_service<geo_transformer::srv::ToLL>(
      "/to_ll",
      std::bind(&GeoTransformationNode::handle_to_ll, this, std::placeholders::_1, std::placeholders::_2)
    );
  }
  // Handler for /local_coordinate/get service
  void handle_get_origin(const std::shared_ptr<geo_transformer::srv::GetOrigin::Request> /*request*/,
                        std::shared_ptr<geo_transformer::srv::GetOrigin::Response> response) {
    if (!origin_set_) {
      response->success = false;
      response->message = "Origin not set.";
      response->latitude = 0.0;
      response->longitude = 0.0;
      response->altitude = 0.0;
      return;
    }
    response->latitude = origin_lat_;
    response->longitude = origin_lon_;
    response->altitude = origin_alt_;
    response->success = true;
    response->message = "Origin retrieved successfully.";
  }
  // Handler for /to_ll service: converts local (x, y, z) to WGS84 (lat, lon, alt)
  void handle_to_ll(const std::shared_ptr<geo_transformer::srv::ToLL::Request> request,
                    std::shared_ptr<geo_transformer::srv::ToLL::Response> response) {
    if (!origin_set_ || !local_cartesian_) {
      response->success = false;
      response->message = "Origin not set. Please call set_origin first.";
      return;
    }
    double lat, lon, alt;
    try {
      local_cartesian_->Reverse(request->x, request->y, request->z, lat, lon, alt);
      response->latitude = lat;
      response->longitude = lon;
      response->altitude = alt;
      response->success = true;
      response->message = "Transformation successful.";
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Transformation failed: ") + e.what();
    }
  }

private:
  // Handler for /local_coordinate/set service: sets the local frame origin
  void handle_set_origin(const std::shared_ptr<geo_transformer::srv::SetOrigin::Request> request,
                        std::shared_ptr<geo_transformer::srv::SetOrigin::Response> response) {
    // Check latitude and longitude bounds
    if (request->latitude < -90.0 || request->latitude > 90.0) {
      response->success = false;
      response->message = "Latitude must be in [-90, 90] degrees.";
      return;
    }
    if (request->longitude < -180.0 || request->longitude > 180.0) {
      response->success = false;
      response->message = "Longitude must be in [-180, 180] degrees.";
      return;
    }
    // Altitude: allow any value, but warn if extreme
    if (request->altitude < -500.0 || request->altitude > 10000.0) {
      RCLCPP_WARN(this->get_logger(), "Unusual altitude: %f meters", request->altitude);
    }
    origin_lat_ = request->latitude;
    origin_lon_ = request->longitude;
    origin_alt_ = request->altitude;
    origin_set_ = true;
    // Initialize the GeographicLib LocalCartesian object with the new origin
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(origin_lat_, origin_lon_, origin_alt_);
    response->success = true;
    response->message = "Origin set to: " + std::to_string(origin_lat_) + ", " +
      std::to_string(origin_lon_) + ", " + std::to_string(origin_alt_);
  }

  // Handler for /from_ll service: converts WGS84 (lat, lon, alt) to local (x, y, z)
  void handle_from_ll(const std::shared_ptr<geo_transformer::srv::FromLL::Request> request,
                      std::shared_ptr<geo_transformer::srv::FromLL::Response> response) {
    if (!origin_set_ || !local_cartesian_) {
      response->success = false;
      response->message = "Origin not set. Please call set_origin first.";
      return;
    }
    // Check latitude and longitude bounds
    if (request->latitude < -90.0 || request->latitude > 90.0) {
      response->success = false;
      response->message = "Latitude must be in [-90, 90] degrees.";
      return;
    }
    if (request->longitude < -180.0 || request->longitude > 180.0) {
      response->success = false;
      response->message = "Longitude must be in [-180, 180] degrees.";
      return;
    }
    if (request->altitude < -500.0 || request->altitude > 10000.0) {
      RCLCPP_WARN(this->get_logger(), "Unusual altitude: %f meters", request->altitude);
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

  // Service objects
  rclcpp::Service<geo_transformer::srv::SetOrigin>::SharedPtr set_origin_service_;
  rclcpp::Service<geo_transformer::srv::GetOrigin>::SharedPtr get_origin_service_;
  rclcpp::Service<geo_transformer::srv::FromLL>::SharedPtr from_ll_service_;
  rclcpp::Service<geo_transformer::srv::ToLL>::SharedPtr to_ll_service_;
  // State for the origin and transformation
  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
};

// Main entry point
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoTransformationNode>());
  rclcpp::shutdown();
  return 0;
}
