// inlcude ROS library
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
// Include ROS message_type which will be published
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <float.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

class LASER_CHECKER {
public:
  LASER_CHECKER(ros::NodeHandle &n);
  ~LASER_CHECKER();
  void init();
  void run();

  void CalculatePredictivePath();
  void CallbackOccupancyGridMap(const nav_msgs::OccupancyGridConstPtr &msg);
  void
  CallbackAckermann(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);
  void CallbackCollisionFreePath(const nav_msgs::PathConstPtr& msg);
  void CallbackMissionState(const std_msgs::StringConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher PubPredictivePathMarker;
  ros::Publisher PubCollision;

  ros::Subscriber SubOccupancyGridMap;
  ros::Subscriber SubVehicleState;
  ros::Subscriber SubCollisionFree;
  ros::Subscriber SubMissionName;

  std::shared_ptr<ackermann_msgs::AckermannDriveStamped> m_VehicleState_ptr;
  std::shared_ptr<nav_msgs::OccupancyGrid> m_CostMap_ptr;
  visualization_msgs::Marker m_PredictivePathMarker;
  bool bVehicleState;
  bool bCostMap;
  bool bCollisionFreePath;

  double m_WheelBase;
  double m_radius;
  double m_SteeringRatio;
  double m_NeglectableArea;
  double m_tooCloseArea;
  geometry_msgs::Point point_prev;
  geometry_msgs::PoseStamped pose_prev;
  nav_msgs::Path m_collisionFreePath;
  std::vector<geometry_msgs::Pose> m_Obstacles;

  bool m_Collision;
  double m_normalDist;
  bool m_DirectWheelControl;
  std::string m_MissionState;
};

LASER_CHECKER::LASER_CHECKER(ros::NodeHandle &n)
    : nh_(n), bVehicleState(false), bCostMap(false), m_Collision(false), bCollisionFreePath(false) {
  // Parameters
  nh_.param<bool>("/laser_collision_checker_node/direct_wheel_control",
                  m_DirectWheelControl, false);
  nh_.param<double>("/laser_collision_checker_node/steering_ratio",
                    m_SteeringRatio, 22.4);
  nh_.param<double>("/laser_collision_checker_node/wheel_base", m_WheelBase,
                    1.5);
  nh_.param<double>("/laser_collision_checker_node/distance_upper_limit",
                    m_NeglectableArea, 2); // UGV: 2, TRAM: 5
  nh_.param<double>("/laser_collision_checker_node/distance_lower_limit", m_tooCloseArea,
                    0.5);
  nh_.param<double>("/laser_collision_checker_node/normal_dist", m_normalDist,
                    1.0);

  // nh_.param<std::string>("/ugv_odom_lanelet2/gps_topic", gps_topic,
  // "/gps/fix");

  PubPredictivePathMarker = nh_.advertise<visualization_msgs::Marker>(
      "/Marker/System/PredictivePath", 10);
  PubCollision = nh_.advertise<std_msgs::Bool>("/Bool/collision", 10);
  SubOccupancyGridMap =
      nh_.subscribe("/semantics/costmap_generator/occupancy_grid", 10,
                    &LASER_CHECKER::CallbackOccupancyGridMap, this);
  SubVehicleState = nh_.subscribe("/Ackermann/veh_state", 10,
                                  &LASER_CHECKER::CallbackAckermann, this);
  SubCollisionFree = nh_.subscribe("/collision_free_path", 10,
                                  &LASER_CHECKER::CallbackCollisionFreePath, this);
  SubMissionName = nh_.subscribe("/decision/mission_state", 10,
                                  &LASER_CHECKER::CallbackMissionState, this);


}

LASER_CHECKER::~LASER_CHECKER() {
  // ROS_INFO("LASER_CHECKER destructor.");
}

void LASER_CHECKER::CalculatePredictivePath() {
  double range = M_PI / 3;
  double increment = 0.1;
  m_PredictivePathMarker.header.frame_id = "base_link";
  m_PredictivePathMarker.header.stamp = ros::Time::now();
  m_PredictivePathMarker.id = 0;
  m_PredictivePathMarker.type = visualization_msgs::Marker::LINE_STRIP;
  m_PredictivePathMarker.action = visualization_msgs::Marker::ADD;
  m_PredictivePathMarker.scale.x = m_normalDist * 2,
  m_PredictivePathMarker.scale.y = m_normalDist * 2,
  m_PredictivePathMarker.scale.z = 0.5;
  m_PredictivePathMarker.pose.position.x = 0;
  m_PredictivePathMarker.pose.position.y = 0;
  m_PredictivePathMarker.pose.orientation.w = 1;
  m_PredictivePathMarker.points.clear();
  m_PredictivePathMarker.colors.clear();
  for (double i = 0; i < range; i += increment) {
    // calc a point of circumference
    geometry_msgs::Point p;
    if (m_radius == DBL_MAX) {
      p.y = 0;
      p.x = i * 20; // Just for visualization
    } else {
      if (m_radius > 0) {
        p.x = m_radius * sin(i);
        p.y = -m_radius * cos(i) + m_radius;
      } else {
        p.x = -m_radius * sin(i);
        p.y = -m_radius * cos(i) + m_radius;
      }
    }
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 1;
    color.b = 1;
    color.a = 0.5;

    m_PredictivePathMarker.points.push_back(p);
    m_PredictivePathMarker.colors.push_back(color);
  }
  PubPredictivePathMarker.publish(m_PredictivePathMarker);
}

void LASER_CHECKER::CallbackAckermann(
    const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
  m_VehicleState_ptr =
      std::make_shared<ackermann_msgs::AckermannDriveStamped>(*msg);
  bVehicleState = true;
  double wheel_angle_rad;
  if (m_DirectWheelControl) // Direct Wheel angle control robot : UGV
  {
    wheel_angle_rad = msg->drive.steering_angle;
  } else // Steering wheel angle control robot : TRAM, Eurecar...
  {
    wheel_angle_rad = msg->drive.steering_angle / m_SteeringRatio;
  }

  if (wheel_angle_rad == 0) {
    m_radius = DBL_MAX;
  } else {
    m_radius = m_WheelBase / tan(wheel_angle_rad);
  }
}

void LASER_CHECKER::CallbackCollisionFreePath(const nav_msgs::PathConstPtr& msg)
{
  m_collisionFreePath = *msg;
  bCollisionFreePath = true;
}

void LASER_CHECKER::CallbackMissionState(const std_msgs::StringConstPtr& msg)
{
  m_MissionState = msg->data;
}


void LASER_CHECKER::CallbackOccupancyGridMap(
    const nav_msgs::OccupancyGridConstPtr &msg) {
  m_CostMap_ptr = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
  bCostMap = true;

  m_Obstacles.clear();
  for (unsigned int width = 0; width < msg->info.width; width++) {
    for (unsigned int height = 0; height < msg->info.height; height++) {
      if (msg->data[height * msg->info.width + width] > 80) {
        geometry_msgs::Pose obstacle;
        obstacle.position.x = width * msg->info.resolution +
                              msg->info.resolution / 2 +
                              msg->info.origin.position.x;
        obstacle.position.y = height * msg->info.resolution +
                              msg->info.resolution / 2 +
                              msg->info.origin.position.y;
        m_Obstacles.push_back(obstacle);
      }
    }
  }
}

void LASER_CHECKER::run() {
  // std::cout << "----------------------" << std::endl;
  m_Collision = false;
  if (bCostMap && bVehicleState && m_MissionState=="Pede_Kick") {
    CalculatePredictivePath();
    for (auto obstacle : m_Obstacles) {
      double cost_distance =
          sqrt(pow(obstacle.position.x, 2) + pow(obstacle.position.y, 2));
      if (cost_distance > m_NeglectableArea ||
          fabs(obstacle.position.x) < m_tooCloseArea ||
          obstacle.position.x < 0) {
        continue;
      }
      
      if (!m_PredictivePathMarker.points.empty()) {
        bool fistloop = true;
        for (auto point : m_PredictivePathMarker.points) {
          double slope;
          if(fistloop)
          {
            point_prev = point;
            fistloop = false;
            continue;
          }

          slope = (point.y - point_prev.y) / (point.x - point_prev.x);

          double bias = point.y - slope * point.x;
          double distance =
              fabs(slope * obstacle.position.x - obstacle.position.y + bias) /
              sqrt(pow(slope, 2) + 1);

          double prev_distance =
              sqrt(pow(point_prev.x - obstacle.position.x, 2) +
                   pow(point_prev.y - obstacle.position.y, 2));
          double prev_to_node = sqrt(pow(point.x - point_prev.x, 2) +
                                     pow(point.y - point_prev.y, 2));

          double prod1 =
              (obstacle.position.x - point_prev.x) * (point.x - point_prev.x) +
              (obstacle.position.y - point_prev.y) * (point.y - point_prev.y);
          double prod2 =
              (obstacle.position.x - point.x) * (point_prev.x - point.x) +
              (obstacle.position.y - point.y) * (point_prev.y - point.y);
          
          if (prod1 < 0 || prod2 < 0)
            continue;

          //   if (distance < m_normalDist && obstacle.position.x > point_prev.x
          //   &&
          //       obstacle.position.x < point.x) {
          if (distance < m_normalDist) {

            // std::cout << obstacle.position << std::endl;
            // std::cout << "collision!, dist: " << distance << std::endl;
            // std::cout << "bias: " << bias << std::endl;
            // std::cout << "slope: " << slope << std::endl;

            m_Collision = true;
            break;
          }
          point_prev = point;
        }
      }


      // if(bCollisionFreePath)
      // {
      //   m_Collision = false;
      //   // std::cout << "check collision free paths" << std::endl;
      //   bool fistloop = true;
      //   for (auto pose : m_collisionFreePath.poses) {
      //     double slope;
      //     if(fistloop)
      //     {
      //       pose_prev = pose;
      //       fistloop = false;
      //       continue;
      //     }

      //     slope = (pose.pose.position.y - pose_prev.pose.position.y) / 
      //             (pose.pose.position.x - pose_prev.pose.position.x);

      //     double bias = pose.pose.position.y - slope * pose.pose.position.x;
      //     double distance =
      //         fabs(slope * obstacle.position.x - obstacle.position.y + bias) /
      //         sqrt(pow(slope, 2) + 1);

      //     double prev_distance =
      //         sqrt(pow(pose_prev.pose.position.x - obstacle.position.x, 2) +
      //               pow(pose_prev.pose.position.y - obstacle.position.y, 2));
      //     double prev_to_node = sqrt(pow(pose.pose.position.x - pose_prev.pose.position.x, 2) +
      //                                 pow(pose.pose.position.y - pose_prev.pose.position.y, 2));

      //     double prod1 =
      //         (obstacle.position.x - pose_prev.pose.position.x) * (pose.pose.position.x - pose_prev.pose.position.x) +
      //         (obstacle.position.y - pose_prev.pose.position.y) * (pose.pose.position.y - pose_prev.pose.position.y);
      //     double prod2 =
      //         (obstacle.position.x - pose.pose.position.x) * (pose_prev.pose.position.x - pose.pose.position.x) +
      //         (obstacle.position.y - pose.pose.position.y) * (pose_prev.pose.position.y - pose.pose.position.y);
          
      //     if (prod1 < 0 || prod2 < 0)
      //       continue;

      //     if (distance < m_normalDist) {

      //       std::cout << "collision" << std::endl;
      //       m_Collision = true;
      //     }
      //     pose_prev = pose;
      //   }    
      // }

    }
  }
  else {
    // ROS_WARN("Check costmap generator or control command publisher.");
  }


  std_msgs::Bool collision_msg;
  collision_msg.data = m_Collision;
  PubCollision.publish(collision_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_collision_checker");
  ros::NodeHandle nh;

  ros::Rate r(25);

  LASER_CHECKER laser_checker(nh);

  while (ros::ok()) {
    laser_checker.run();
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
