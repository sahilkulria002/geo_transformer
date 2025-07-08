#include "geo_transformer/srv/set_origin.hpp"      // Service for setting the origin
#include "geo_transformer/srv/get_origin.hpp"      // Service for getting the origin
#include "geo_transformer/srv/from_ll.hpp"         // Service for WGS84 to local conversion
#include "geo_transformer/srv/to_ll.hpp"           // Service for local to WGS84 conversion
#include "geo_transformer/srv/add_named_origin.hpp" // Add a named origin
#include "geo_transformer/srv/switch_origin.hpp"    // Switch to a named origin
#include "geo_transformer/srv/list_origins.hpp"     // List all named origins
#include "geo_transformer/srv/get_current_origin_name.hpp" // Get current origin name
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>

// Main node class for coordinate transformation services
class GeoTransformationNode : public rclcpp::Node {
public:
  GeoTransformationNode() : Node("geo_transformer_node"), origin_set_(false), origin_lat_(0.0), origin_lon_(0.0), origin_alt_(0.0), current_origin_name_("default") {
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

    // New: Named origin services
    add_named_origin_service_ = this->create_service<geo_transformer::srv::AddNamedOrigin>(
      "/origin_store/add_named_origin",
      std::bind(&GeoTransformationNode::handle_add_named_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
    switch_origin_service_ = this->create_service<geo_transformer::srv::SwitchOrigin>(
      "/origin_store/switch_origin",
      std::bind(&GeoTransformationNode::handle_switch_origin, this, std::placeholders::_1, std::placeholders::_2)
    );
    list_origins_service_ = this->create_service<geo_transformer::srv::ListOrigins>(
      "/origin_store/list_origins",
      std::bind(&GeoTransformationNode::handle_list_origins, this, std::placeholders::_1, std::placeholders::_2)
    );
    get_current_origin_name_service_ = this->create_service<geo_transformer::srv::GetCurrentOriginName>(
      "/origin_store/get_current_origin_name",
      std::bind(&GeoTransformationNode::handle_get_current_origin_name, this, std::placeholders::_1, std::placeholders::_2)
    );
  }
  // Handler for /origin_store/add_named_origin
  void handle_add_named_origin(const std::shared_ptr<geo_transformer::srv::AddNamedOrigin::Request> request,
                              std::shared_ptr<geo_transformer::srv::AddNamedOrigin::Response> response) {
    std::lock_guard<std::mutex> lock(named_mutex_);
    if (named_origins_.count(request->name) > 0) {
      response->success = false;
      response->message = "Origin name already exists.";
      return;
    }
    named_origins_[request->name] = {request->latitude, request->longitude, request->altitude};
    named_cartesians_[request->name] = std::make_unique<GeographicLib::LocalCartesian>(request->latitude, request->longitude, request->altitude);
    response->success = true;
    response->message = "Named origin added: " + request->name;
  }

  // Handler for /origin_store/switch_origin
  void handle_switch_origin(const std::shared_ptr<geo_transformer::srv::SwitchOrigin::Request> request,
                            std::shared_ptr<geo_transformer::srv::SwitchOrigin::Response> response) {
    std::lock_guard<std::mutex> lock(named_mutex_);
    auto it = named_origins_.find(request->name);
    if (it == named_origins_.end()) {
      response->success = false;
      response->message = "Origin name not found.";
      return;
    }
    // Switch current origin to the named one
    origin_lat_ = it->second[0];
    origin_lon_ = it->second[1];
    origin_alt_ = it->second[2];
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(origin_lat_, origin_lon_, origin_alt_);
    origin_set_ = true;
    current_origin_name_ = request->name;
    response->success = true;
    response->message = "Switched to origin: " + request->name;
  }

  // Handler for /origin_store/list_origins
  void handle_list_origins(const std::shared_ptr<geo_transformer::srv::ListOrigins::Request> /*request*/,
                           std::shared_ptr<geo_transformer::srv::ListOrigins::Response> response) {
    std::lock_guard<std::mutex> lock(named_mutex_);
    response->names.clear();
    for (const auto& kv : named_origins_) {
      response->names.push_back(kv.first);
    }
  }

  // Handler for /origin_store/get_current_origin_name
  void handle_get_current_origin_name(const std::shared_ptr<geo_transformer::srv::GetCurrentOriginName::Request> /*request*/,
                                      std::shared_ptr<geo_transformer::srv::GetCurrentOriginName::Response> response) {
    response->name = current_origin_name_;
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
  // New: Named origin services
  rclcpp::Service<geo_transformer::srv::AddNamedOrigin>::SharedPtr add_named_origin_service_;
  rclcpp::Service<geo_transformer::srv::SwitchOrigin>::SharedPtr switch_origin_service_;
  rclcpp::Service<geo_transformer::srv::ListOrigins>::SharedPtr list_origins_service_;
  rclcpp::Service<geo_transformer::srv::GetCurrentOriginName>::SharedPtr get_current_origin_name_service_;
  // State for the origin and transformation
  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  // Named origins
  std::map<std::string, std::array<double, 3>> named_origins_;
  std::map<std::string, std::unique_ptr<GeographicLib::LocalCartesian>> named_cartesians_;
  std::string current_origin_name_;
  std::mutex named_mutex_;
};

// Main entry point
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoTransformationNode>());
  rclcpp::shutdown();
  return 0;
}
