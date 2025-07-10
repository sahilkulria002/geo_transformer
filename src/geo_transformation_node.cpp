#include "geo_transformer/srv/set_origin.hpp"      // Service for setting the origin
#include "geo_transformer/srv/get_origin.hpp"      // Service for getting the origin
#include "geo_transformer/srv/from_ll.hpp"         // Service for WGS84 to local conversion
#include "geo_transformer/srv/to_ll.hpp"           // Service for local to WGS84 conversion
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>
#include <limits>

class GeoTransformerNode : public rclcpp::Node {
public:
  GeoTransformerNode()
  : Node("geo_transformer_node"),
    origin_set_(false),
    origin_lat_(std::numeric_limits<double>::quiet_NaN()),
    origin_lon_(std::numeric_limits<double>::quiet_NaN()),
    origin_alt_(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "GeoTransformerNode started.");

    // Advertise services
    set_origin_service_ = create_service<geo_transformer::srv::SetOrigin>(
      "/local_coordinate/set",
      std::bind(&GeoTransformerNode::handle_set_origin, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_origin_service_ = create_service<geo_transformer::srv::GetOrigin>(
      "/local_coordinate/get",
      std::bind(&GeoTransformerNode::handle_get_origin, this, std::placeholders::_1, std::placeholders::_2)
    );

    from_ll_service_ = create_service<geo_transformer::srv::FromLL>(
      "/from_ll",
      std::bind(&GeoTransformerNode::handle_from_ll, this, std::placeholders::_1, std::placeholders::_2)
    );

    to_ll_service_ = create_service<geo_transformer::srv::ToLL>(
      "/to_ll",
      std::bind(&GeoTransformerNode::handle_to_ll, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  // === Set Origin ===
  void handle_set_origin(
    const std::shared_ptr<geo_transformer::srv::SetOrigin::Request> request,
    std::shared_ptr<geo_transformer::srv::SetOrigin::Response> response)
  {
    // Validate latitude and longitude
    if (request->latitude < -90.0 || request->latitude > 90.0) {
      response->success = false;
      response->message = "Invalid latitude. Must be in [-90, 90] degrees.";
      return;
    }
    if (request->longitude < -180.0 || request->longitude > 180.0) {
      response->success = false;
      response->message = "Invalid longitude. Must be in [-180, 180] degrees.";
      return;
    }

    if (request->altitude < -500.0 || request->altitude > 10000.0) {
      RCLCPP_WARN(this->get_logger(), "Unusual altitude: %.2f meters", request->altitude);
    }

    // Set origin
    origin_lat_ = request->latitude;
    origin_lon_ = request->longitude;
    origin_alt_ = request->altitude;
    origin_set_ = true;

    try {
      local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
        origin_lat_, origin_lon_, origin_alt_);
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("GeographicLib error: ") + e.what();
      return;
    }

    response->success = true;
    response->message = "Origin set to (" + std::to_string(origin_lat_) + ", " +
                        std::to_string(origin_lon_) + ", " +
                        std::to_string(origin_alt_) + ")";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // === Get Origin ===
  void handle_get_origin(
    const std::shared_ptr<geo_transformer::srv::GetOrigin::Request> /*request*/,
    std::shared_ptr<geo_transformer::srv::GetOrigin::Response> response)
  {
    if (!origin_set_) {
      response->success = false;
      response->message = "Origin not set. Call /local_coordinate/set first.";
      response->latitude = std::numeric_limits<double>::quiet_NaN();
      response->longitude = std::numeric_limits<double>::quiet_NaN();
      response->altitude = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    response->latitude = origin_lat_;
    response->longitude = origin_lon_;
    response->altitude = origin_alt_;
    response->success = true;
    response->message = "Origin retrieved successfully.";
  }

  // === WGS84 → Local ===
  void handle_from_ll(
    const std::shared_ptr<geo_transformer::srv::FromLL::Request> request,
    std::shared_ptr<geo_transformer::srv::FromLL::Response> response)
  {
    if (!origin_set_ || !local_cartesian_) {
      response->success = false;
      response->message = "Origin not set. Cannot perform conversion.";
      return;
    }

    if (request->latitude < -90.0 || request->latitude > 90.0 ||
        request->longitude < -180.0 || request->longitude > 180.0)
    {
      response->success = false;
      response->message = "Invalid GPS input.";
      return;
    }
    if (request->altitude < -500.0 || request->altitude > 10000.0) {
      RCLCPP_WARN(this->get_logger(), "Unusual altitude: %.2f meters", request->altitude);
    }

    try {
      double x, y, z;
      local_cartesian_->Forward(request->latitude, request->longitude, request->altitude, x, y, z);
      response->x = x;
      response->y = y;
      response->z = z;
      response->success = true;
      response->message = "Conversion to local coordinates successful.";
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Error during conversion: ") + e.what();
    }
  }

  // === Local → WGS84 ===
  void handle_to_ll(
    const std::shared_ptr<geo_transformer::srv::ToLL::Request> request,
    std::shared_ptr<geo_transformer::srv::ToLL::Response> response)
  {
    if (!origin_set_ || !local_cartesian_) {
      response->success = false;
      response->message = "Origin not set. Cannot perform conversion.";
      return;
    }

    try {
      double lat, lon, alt;
      local_cartesian_->Reverse(request->x, request->y, request->z, lat, lon, alt);
      response->latitude = lat;
      response->longitude = lon;
      response->altitude = alt;
      response->success = true;
      response->message = "Conversion to GPS successful.";
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Error during conversion: ") + e.what();
    }
  }

  // === Internal State ===
  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;

  // === Service Handles ===
  rclcpp::Service<geo_transformer::srv::SetOrigin>::SharedPtr set_origin_service_;
  rclcpp::Service<geo_transformer::srv::GetOrigin>::SharedPtr get_origin_service_;
  rclcpp::Service<geo_transformer::srv::FromLL>::SharedPtr from_ll_service_;
  rclcpp::Service<geo_transformer::srv::ToLL>::SharedPtr to_ll_service_;
};

// === Entry point ===
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
