#include "robot_parser.hpp"


RobotParser::RobotParser() : Node("robot_parser") {
  RCLCPP_INFO(this->get_logger(), "RobotParser Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 50.0f); // [Hz]
  this->urdfFilePath = this->declare_parameter("robot_description", "urdf/robot.urdf"); // Path to the URDF file
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->urdfFilePath = this->get_parameter("robot_description").as_string();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  // Setup obstacle publisher
  this->obstaclePub = this->create_publisher<Obstacle>("pfield/obstacles", 10);

  // Setup TF2 broadcaster, buffer, and listener
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);
  RCLCPP_INFO(this->get_logger(), "TF2 broadcaster, buffer, and listener initialized");

  // Run the timer to periodically update the robot state
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / this->timerFreq)),
    std::bind(&RobotParser::timerCallback, this)
  );
}

void RobotParser::timerCallback() {
  std::vector<Obstacle> obstacles = this->parseURDF();
  for (const auto& obstacle : obstacles) {
    this->obstaclePub->publish(obstacle);
  }
}

std::vector<Obstacle> RobotParser::parseURDF() {
  std::vector<Obstacle> collisionObjects;
  urdf::Model robotModel;
  if (!robotModel.initFile(this->urdfFilePath)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from %s", this->urdfFilePath.c_str());
  } else {
    // Extract collision geometries from the URDF model and add them as obstacles
    int obstacleID = 0;
    for (const auto& link : robotModel.links_) {
      // Create a potential field obstacle from the collision geometry
      // Get the link's transform in the world frame
      std::string link_name = link.second->name;
      if (!link.second->collision) {
        RCLCPP_WARN(this->get_logger(), "Link '%s' has no collision geometry, skipping", link_name.c_str());
        continue;
      }
      // Get the transform from the world frame to the link's frame
      // Since robot_state_publisher should be publishing transforms
      TransformStamped linkTransform;
      try {
        linkTransform = this->tfBuffer->lookupTransform(this->fixedFrame, link_name, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform for link '%s': %s", link_name.c_str(), ex.what());
      }
      const urdf::Pose& collisionOrigin = link.second->collision->origin;
      // Convert TF to Eigen::Affine3d
      Eigen::Affine3d world_T_link = tf2::transformToEigen(linkTransform.transform);
      // Construct link_T_collision from URDF collision origin
      Eigen::Affine3d link_T_col =
        Eigen::Translation3d(
          collisionOrigin.position.x,
          collisionOrigin.position.y,
          collisionOrigin.position.z) *
        Eigen::Quaterniond(
          collisionOrigin.rotation.w,
          collisionOrigin.rotation.x,
          collisionOrigin.rotation.y,
          collisionOrigin.rotation.z);

      // Compose to get world_T_collision
      Eigen::Affine3d world_T_col = world_T_link * link_T_col;

      // Extract final obstacle pose
      Eigen::Vector3d obstCenter = world_T_col.translation();
      Eigen::Quaterniond obstOrientation(world_T_col.rotation());
      Obstacle obst = this->obstacleFromCollisionObject(
        obstacleID++,
        *link.second->collision,
        obstCenter,
        obstOrientation
      );
      collisionObjects.push_back(obst);
    }
  }
  return collisionObjects;
}

Obstacle RobotParser::obstacleFromCollisionObject(
  int id, const urdf::Collision& collisionObject,
  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  ObstacleType type;
  ObstacleGeometry geom;

  auto* geometry = collisionObject.geometry.get();
  if (urdf::Box* b = dynamic_cast<urdf::Box*>(geometry)) {
    type = ObstacleType::BOX;
    geom.length = b->dim.x;
    geom.width = b->dim.y;
    geom.height = b->dim.z;
  } else if (urdf::Sphere* s = dynamic_cast<urdf::Sphere*>(geometry)) {
    type = ObstacleType::SPHERE;
    geom.radius = s->radius;
  } else if (urdf::Cylinder* c = dynamic_cast<urdf::Cylinder*>(geometry)) {
    type = ObstacleType::CYLINDER;
    geom.radius = c->radius;
    geom.height = c->length;
  } else if (urdf::Mesh* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    // Approximate mesh as a box for now
    type = ObstacleType::BOX;
    // TODO: Handle mesh geometry to assign Box size
  } else {
    // throw std::runtime_error("Unhandled URDF geometry type");
    RCLCPP_ERROR(this->get_logger(),
      "Unhandled URDF geometry type for collision object with id %d", id);
  }
  Obstacle obstacle;
  obstacle.id = id;
  obstacle.type = obstacleTypeToString(type);
  obstacle.pose.position.x = position.x();
  obstacle.pose.position.y = position.y();
  obstacle.pose.position.z = position.z();
  obstacle.pose.orientation.x = orientation.x();
  obstacle.pose.orientation.y = orientation.y();
  obstacle.pose.orientation.z = orientation.z();
  obstacle.pose.orientation.w = orientation.w();
  obstacle.radius = geom.radius;
  obstacle.length = geom.length;
  obstacle.width = geom.width;
  obstacle.height = geom.height;
  return obstacle;
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotParser>());
  rclcpp::shutdown();
  return 0;
}
