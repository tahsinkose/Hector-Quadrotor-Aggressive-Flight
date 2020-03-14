#include <cassert>
#include <map>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>
#include <ros/ros.h>

#include <starmac/starmac.h>



#define OPERATION_ALTITUDE 20

typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;

class AggressiveTrajectoryTracker {
 public:
  AggressiveTrajectoryTracker() {
    pose_client_.reset(new PoseClient("/action/pose", true));
    landing_client_.reset(new LandingClient("/action/landing", true));
    pose_client_->waitForServer();
    landing_client_->waitForServer();
    // Advertise cmd_vel topic to give velocity commands. Note that given 3.0 m/s in x direction does
    // not necessarily mean that quadcopter will be moving with 3.0 m/s in that time instance. In
    // fact, these are acceleration commands. In other applications, they may be negligible, but in
    // this case they are not since aggressive flight requires ultimate obedience to the computed
    // velocities. Therefore, this is an open-loop algorithm.
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }
  void run() {
    std::vector<geometry_msgs::Pose> trajectory = starmac::createPattern("SQUARE");
    std::vector<double> v_i_allow = starmac::constrainCrossTrackAcceleration(trajectory);

    // Compute velocity, acceleration and time profiles.
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> profiles =
        starmac::velocityPlanSweep(trajectory, v_i_allow);

    std::vector<double> speed_profile = std::get<0>(profiles);
    std::vector<double> acc_profile = std::get<1>(profiles);
    std::vector<double> time_profile = std::get<2>(profiles);
    //assert(speed_profile.size() == acc_profile.size() == time_profile.size());
    ROS_INFO("Sizes: %lu, %lu, %lu",speed_profile.size(),acc_profile.size(),time_profile.size());
    size_t sz = speed_profile.size();
    std::ostringstream toss, soss, aoss;
    for(size_t i = 0; i < sz;i++){
      toss << time_profile[i] << ",";
      soss << speed_profile[i] << ",";
      aoss << acc_profile[i] << ",";
    }
    toss << "\n";
    soss << "\n";
    aoss << "\n";

    ROS_INFO("Time profile: %s",toss.str().c_str());
    ROS_INFO("Speed profile: %s", soss.str().c_str());
    ROS_INFO("Acceleration profile: %s", aoss.str().c_str());

    // Move the quadcopter to required altitude.
    hector_uav_msgs::PoseGoal pose_goal;
    geometry_msgs::Pose p;
    p.position.z = OPERATION_ALTITUDE;
    p.orientation.w = 1;
    pose_goal.target_pose.pose = p;
    pose_goal.target_pose.header.frame_id = "world";
    pose_client_->sendGoal(pose_goal);
    ROS_INFO("Going to pose");
    pose_client_->waitForResult();

    std::thread hover_thread(&AggressiveTrajectoryTracker::hover, this);

    for (int i = 0; i < speed_profile.size() - 1; i++) {
      double diff_x = trajectory[i + 1].position.x - trajectory[i].position.x;
      double diff_y = trajectory[i + 1].position.y - trajectory[i].position.y;

      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x = diff_x;
      vel_msg.linear.y = diff_y;
      double length = sqrt(pow(vel_msg.linear.x, 2) + pow(vel_msg.linear.y, 2));
      vel_msg.linear.x /= length;
      vel_msg.linear.y /= length;
      vel_msg.linear.x *= speed_profile[i];
      vel_msg.linear.y *= speed_profile[i];

      double step = (speed_profile[i + 1] - speed_profile[i]) / (double)ACCELERATION_STEP;
      // Discretize acceleration. Since it is impossible to give continuous commands, it is necessary
      // to gradually lower velocity commands.
      for (int j = 0; j < ACCELERATION_STEP; j++) {
        geometry_msgs::Twist acced_vel_msg;
        acced_vel_msg.linear.x = vel_msg.linear.x + j * step * (diff_x);
        acced_vel_msg.linear.y = vel_msg.linear.y + j * step * (diff_y);
        if (fabs(step) > EPSILON)
          ROS_INFO("Step: %lf | Accelerated velocity %lf,%lf", step, acced_vel_msg.linear.x, acced_vel_msg.linear.y);
        pub_vel_.publish(acced_vel_msg);
        double longer = 0.0;
        if (i == speed_profile.size() - 2 && j == ACCELERATION_STEP - 1) longer = 0.05;
        ros::Duration((time_profile[i] / (double)ACCELERATION_STEP) + longer).sleep();
      }
      // This has an improving effect on the final error.
    }

    hector_uav_msgs::LandingGoal landing_goal;
    p.position.x = p.position.y = 0;
    p.position.z = 0.3;
    landing_goal.landing_zone.header.frame_id = "world";
    landing_goal.landing_zone.pose = p;
    landing_client_->sendGoal(landing_goal);
    ROS_INFO("Landing...");
    landing_client_->waitForResult();
    landing_finished_ = true;
    hover_thread.join();
  }

 private:
  void hover() {
    geometry_msgs::Twist empty;

    empty.linear.x = empty.linear.y = empty.linear.z = 0;
    empty.angular.x = empty.angular.y = empty.angular.z = 0;
    while (!landing_finished_) {
      pub_vel_.publish(empty);
      ros::Duration(0.5).sleep();
    }
  }
  std::shared_ptr<PoseClient> pose_client_;
  std::shared_ptr<LandingClient> landing_client_;
  bool landing_finished_ = false;
  ros::NodeHandle nh_;
  ros::Publisher pub_vel_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_controller");
  AggressiveTrajectoryTracker runner;
  runner.run();
  return 0;
}