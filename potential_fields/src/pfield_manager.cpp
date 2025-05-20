#include "rclcpp/rclcpp.hpp"
#include "pfield_manager.hpp"
#include "pfield.hpp"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [N]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [N]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [N]
  this->maxForce = this->declare_parameter("max_force", 10.0f); // [N]
  this->influenceRadiusScalar = this->declare_parameter("influence_radius_scale", 2.0f); // Scale for influence radius
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxForce = this->get_parameter("max_force").as_double();
  this->influenceRadiusScalar = this->get_parameter("influence_radius_scale").as_double();

  // Initialize the potential field
  this->pField = PotentialField(SpatialVector{}, this->attractiveGain, this->rotationalAttractiveGain);
  this->pField.addObstacle(SphereObstacle(0, Eigen::Vector3d(3, 3, 0), 1.0f, 2.0f, this->repulsiveGain));
  this->pField.addObstacle(SphereObstacle(1, Eigen::Vector3d(-5, -5, 0), 1.5f, 2.5f, this->repulsiveGain));

  // Setup marker publisher
  this->markerPub = this->create_publisher<MarkerArray>("visualization_marker_array", 10);
  std::cout << "Markers publishing on: " << this->markerPub->get_topic_name() << std::endl;

  this->timer = this->create_wall_timer(
    std::chrono::seconds(static_cast<int>(1.0f / this->timerFreq)),
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  this->visualizePF();
}

void PotentialFieldManager::visualizePF() {
  MarkerArray markerArray;
  markerArray.markers.push_back(this->createGoalMarker());
  auto obstacleMarkers = this->createObstacleMarkers();
  markerArray.markers.insert(markerArray.markers.cend(), obstacleMarkers.markers.cbegin(),
    obstacleMarkers.markers.cend());
  auto potentialVectorMarkers = this->createPotentialVectorMarkers();
  markerArray.markers.insert(markerArray.markers.cend(), potentialVectorMarkers.markers.cbegin(),
    potentialVectorMarkers.markers.cend());
  // Publish the marker array
  this->markerPub->publish(markerArray);
}

MarkerArray PotentialFieldManager::createObstacleMarkers() {
  MarkerArray markerArray;
  int id = 0;
  for (const auto& obstacle : this->pField.getObstacles()) {
    Marker obstacleMarker;
    obstacleMarker.header.frame_id = "map";
    obstacleMarker.header.stamp = this->now();
    obstacleMarker.ns = "obstacle";
    obstacleMarker.id = id++;
    obstacleMarker.type = Marker::SPHERE;
    obstacleMarker.action = Marker::ADD;
    SpatialVector position = obstacle.getPosition();
    obstacleMarker.pose.position.x = position.getPosition().x();
    obstacleMarker.pose.position.y = position.getPosition().y();
    obstacleMarker.pose.position.z = position.getPosition().z();
    obstacleMarker.pose.orientation.x = position.getOrientation().x();
    obstacleMarker.pose.orientation.y = position.getOrientation().y();
    obstacleMarker.pose.orientation.z = position.getOrientation().z();
    obstacleMarker.pose.orientation.w = position.getOrientation().w();
    // Scale is the Diameter of the Sphere
    obstacleMarker.scale.x = obstacle.getRadius() * 2.0f;
    obstacleMarker.scale.y = obstacle.getRadius() * 2.0f;
    obstacleMarker.scale.z = obstacle.getRadius() * 2.0f;
    obstacleMarker.color.r = 1.0f;
    obstacleMarker.color.g = 0.0f;
    obstacleMarker.color.b = 0.0f;
    obstacleMarker.color.a = 1.0f; // Opaque
    obstacleMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(obstacleMarker);
    // Create a transparent sphere representing the influence radius
    Marker influenceMarker;
    influenceMarker.header.frame_id = "map";
    influenceMarker.header.stamp = this->now();
    influenceMarker.ns = "obstacle_influence";
    influenceMarker.id = id++;
    influenceMarker.type = Marker::SPHERE;
    influenceMarker.action = Marker::ADD;
    influenceMarker.pose.position.x = position.getPosition().x();
    influenceMarker.pose.position.y = position.getPosition().y();
    influenceMarker.pose.position.z = position.getPosition().z();
    influenceMarker.pose.orientation.w = 1.0;
    // Scale is the Diameter of the Sphere
    influenceMarker.scale.x = obstacle.getInfluenceRadius() * 2.0f;
    influenceMarker.scale.y = obstacle.getInfluenceRadius() * 2.0f;
    influenceMarker.scale.z = obstacle.getInfluenceRadius() * 2.0f;
    influenceMarker.color.r = 1.0f;
    influenceMarker.color.g = 1.0f;
    influenceMarker.color.b = 0.0f;
    influenceMarker.color.a = 0.5f; // Semi-transparent
    influenceMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(influenceMarker);
  }
  return markerArray;
}

Marker PotentialFieldManager::createGoalMarker() {
  // Create a green sphere marker
  Marker goalMarker;
  goalMarker.header.frame_id = "map";
  goalMarker.header.stamp = this->now();
  goalMarker.ns = "goal";
  goalMarker.id = 0;
  goalMarker.type = Marker::SPHERE;
  goalMarker.action = Marker::ADD;
  SpatialVector goalPose = this->pField.getGoalPose();
  goalMarker.pose.position.x = goalPose.getPosition().x();
  goalMarker.pose.position.y = goalPose.getPosition().y();
  goalMarker.pose.position.z = goalPose.getPosition().z();
  goalMarker.pose.orientation.x = goalPose.getOrientation().x();
  goalMarker.pose.orientation.y = goalPose.getOrientation().y();
  goalMarker.pose.orientation.z = goalPose.getOrientation().z();
  goalMarker.pose.orientation.w = goalPose.getOrientation().w();
  goalMarker.scale.x = 0.5;
  goalMarker.scale.y = 0.5;
  goalMarker.scale.z = 0.5;
  goalMarker.color.r = 0.0f;
  goalMarker.color.g = 1.0f;
  goalMarker.color.b = 0.0f;
  goalMarker.color.a = 1.0f; // Opaque
  goalMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
  return goalMarker;
}

MarkerArray PotentialFieldManager::createPotentialVectorMarkers() {
  MarkerArray markerArray;
  int id = 0;
  // Discretize the space around the goal till the farthest obstacle
  // For now, let's hard-code 10x10 grid at a single Z-height
  double Z = this->pField.getGoalPose().getPosition().z();
  for (double x = -5.0f; x <= 5.0f; x += 1.0f) {
    for (double y = -5.0f; y <= 5.0f; y += 1.0f) {
      Marker vectorMarker;
      SpatialVector position{Eigen::Vector3d(x, y, Z)};
      SpatialVector velocity = this->pField.evaluateVelocityAtPose(position);
      double magnitude = velocity.getPosition().norm();
      // Normalize the velocity vector since we want to show direction
      velocity.normalizePosition();
      vectorMarker.header.frame_id = "map";
      vectorMarker.header.stamp = this->now();
      vectorMarker.ns = "potential_vectors";
      vectorMarker.id = id++;
      vectorMarker.type = Marker::ARROW;
      vectorMarker.action = Marker::ADD;
      vectorMarker.pose.position.x = position.getPosition().x();
      vectorMarker.pose.position.y = position.getPosition().y();
      vectorMarker.pose.position.z = position.getPosition().z();
      // Set the orientation of the arrow to point in the direction of the velocity vector
      double yaw = std::atan2(velocity.getPosition().y(), velocity.getPosition().x());
      vectorMarker.pose.orientation = PotentialFieldManager::getQuaternionFromYaw(yaw);
      vectorMarker.scale.x = 0.15f; // Shaft diameter
      vectorMarker.scale.y = 0.3f; // Head diameter
      vectorMarker.scale.z = 0.75f; // Length of the arrow
      // Color the arrows using a gradient depending on
      // the magnitude of the velocity vector
      double colorScale = std::min<double>(magnitude / this->maxForce, 1.0f);
      vectorMarker.color.r = 1.0f - colorScale; // Red to yellow
      vectorMarker.color.g = colorScale; // Yellow to green
      vectorMarker.color.b = 0.0f; // No blue
      vectorMarker.color.a = 0.75f; // Semi-transparent
      vectorMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
      markerArray.markers.push_back(vectorMarker);
    }
  }
  return markerArray;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
