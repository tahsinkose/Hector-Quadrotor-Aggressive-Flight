#include <ros/ros.h>
#include <starmac/starmac.h>

namespace starmac {

namespace {
std::map<std::string, std::vector<XYZ>> patterns = {
    {"SQUARE",
     {XYZ(0, 0, OPERATION_ALTITUDE), XYZ(15, 0, OPERATION_ALTITUDE), XYZ(15, 15, OPERATION_ALTITUDE),
      XYZ(0, 15, OPERATION_ALTITUDE), XYZ(0, 0, OPERATION_ALTITUDE)}},
    {"HEXAGON",
     {XYZ(0, 0, OPERATION_ALTITUDE), XYZ(15, 0, OPERATION_ALTITUDE), XYZ(30, 15, OPERATION_ALTITUDE),
      XYZ(15, 30, OPERATION_ALTITUDE), XYZ(0, 30, OPERATION_ALTITUDE), XYZ(-15, 15, OPERATION_ALTITUDE),
      XYZ(0, 0, OPERATION_ALTITUDE)}},
    {"STAR",
     {XYZ(0, 0, OPERATION_ALTITUDE), XYZ(10, 20, OPERATION_ALTITUDE), XYZ(20, 0, OPERATION_ALTITUDE),
      XYZ(-5, 15, OPERATION_ALTITUDE), XYZ(25, 15, OPERATION_ALTITUDE), XYZ(0, 0, OPERATION_ALTITUDE)}},
    {"TOOTHED",
     {XYZ(0, 0, OPERATION_ALTITUDE), XYZ(5, 10, OPERATION_ALTITUDE), XYZ(10, 0, OPERATION_ALTITUDE),
      XYZ(15, 10, OPERATION_ALTITUDE), XYZ(20, 0, OPERATION_ALTITUDE), XYZ(25, 10, OPERATION_ALTITUDE),
      XYZ(30, 0, OPERATION_ALTITUDE), XYZ(35, 10, OPERATION_ALTITUDE), XYZ(40, 0, OPERATION_ALTITUDE),
      XYZ(35, -10, OPERATION_ALTITUDE), XYZ(30, 0, OPERATION_ALTITUDE), XYZ(25, -10, OPERATION_ALTITUDE),
      XYZ(20, 0, OPERATION_ALTITUDE), XYZ(15, -10, OPERATION_ALTITUDE), XYZ(10, 0, OPERATION_ALTITUDE),
      XYZ(5, -10, OPERATION_ALTITUDE), XYZ(0, 0, OPERATION_ALTITUDE)}}};
}
// clang-format off
std::vector<geometry_msgs::Pose> sample(std::vector<geometry_msgs::Pose> &traj) {
  // clang-format on
  std::vector<geometry_msgs::Pose> result;
  for (int i = 0; i < traj.size() - 1; i++) {
    double norm =
        sqrt(pow(traj[i + 1].position.x - traj[i].position.x, 2) + pow(traj[i + 1].position.y - traj[i].position.y, 2) +
             pow(traj[i + 1].position.z - traj[i].position.z, 2));
    int step = (int)ceil(norm / SPACE);
    double x_factor = (traj[i + 1].position.x - traj[i].position.x) / step;
    double y_factor = (traj[i + 1].position.y - traj[i].position.y) / step;
    double z_factor = (traj[i + 1].position.z - traj[i].position.z) / step;
    for (int j = 0; j < step; j++) {
      geometry_msgs::Pose wp;
      wp.position.x = traj[i].position.x + j * x_factor;
      wp.position.y = traj[i].position.y + j * y_factor;
      wp.position.z = traj[i].position.z + j * z_factor;
      wp.orientation.w = 1.0;
      result.push_back(wp);
    }
  }
  result.push_back(traj[traj.size() - 1]);
  return result;
}

std::vector<geometry_msgs::Pose> createPattern(std::string &&pattern) {
  std::vector<geometry_msgs::Pose> traj;
  geometry_msgs::Pose waypoint;
  waypoint.orientation.w = 1;
  const auto &wps = patterns.at(pattern);
  for (const auto &wp : wps) {
    waypoint.position.x = wp.x;
    waypoint.position.y = wp.y;
    waypoint.position.z = wp.z;
    traj.push_back(waypoint);
    ROS_INFO("Waypoint: %lf,%lf,%lf",wp.x,wp.y,wp.z);
  }
  traj = sample(traj);
  return traj;
}

std::vector<double> constrainCrossTrackAcceleration(const std::vector<geometry_msgs::Pose> &trajectory) {
  std::vector<double> c_i, r_i, v_i_allow;
  c_i.resize(trajectory.size());
  r_i.resize(trajectory.size(), 0);
  v_i_allow.resize(trajectory.size());

  // Compute maximum allowed velocity for each waypoint.
  for (int i = 0; i < trajectory.size(); i++) {
    if (i == 0) {
      geometry_msgs::Point current_position = trajectory[i].position;
      geometry_msgs::Point next_position = trajectory[i + 1].position;
      c_i[i] = 2 * ((next_position.y * current_position.x) - (current_position.y * next_position.x));
      if (fabs(c_i[i]) > EPSILON) {
        double x_center, y_center;
        double curr_x_i_y_i_square = pow(current_position.x, 2) + pow(current_position.y, 2);
        double next_x_i_y_i_square = pow(next_position.x, 2) + pow(next_position.y, 2);
        x_center = curr_x_i_y_i_square * next_position.y - next_x_i_y_i_square * current_position.y;
        y_center = curr_x_i_y_i_square * -next_position.x + next_x_i_y_i_square * current_position.x;
        x_center /= c_i[i];
        y_center /= c_i[i];

        r_i[i] = sqrt(pow(current_position.x - x_center, 2) + pow(current_position.y - y_center, 2));

        ROS_INFO(
            "Midpoint of curvature at waypoint[%d] is %lf,%lf - radius: "
            "%lf, max allowed velocity: "
            "%lf",
            i, x_center, y_center, r_i[i], v_i_allow[i]);
        v_i_allow[i] = sqrt(MAX_ACCELERATION * r_i[i]);
      } else
        v_i_allow[i] = MAX_SPEED;
    } else if (i == trajectory.size() - 1)
      v_i_allow[i] = 0;
    else {
      geometry_msgs::Point prev_position = trajectory[i - 1].position;
      geometry_msgs::Point current_position = trajectory[i].position;
      geometry_msgs::Point next_position = trajectory[i + 1].position;
      c_i[i] = (next_position.y * (current_position.x - prev_position.x)) +
               (current_position.y * (prev_position.x - next_position.x)) +
               (prev_position.y * (next_position.x - current_position.x));
      c_i[i] *= 2;
      if (fabs(c_i[i]) > EPSILON) {
        double x_center, y_center;
        double prev_x_i_y_i_square = pow(prev_position.x, 2) + pow(prev_position.y, 2);
        double curr_x_i_y_i_square = pow(current_position.x, 2) + pow(current_position.y, 2);
        double next_x_i_y_i_square = pow(next_position.x, 2) + pow(next_position.y, 2);
        x_center = prev_x_i_y_i_square * (current_position.y - next_position.y) +
                   curr_x_i_y_i_square * (next_position.y - prev_position.y) +
                   next_x_i_y_i_square * (prev_position.y - current_position.y);
        y_center = prev_x_i_y_i_square * (next_position.x - current_position.x) +
                   curr_x_i_y_i_square * (prev_position.x - next_position.x) +
                   next_x_i_y_i_square * (current_position.x - prev_position.x);

        x_center /= c_i[i];
        y_center /= c_i[i];

        r_i[i] = sqrt(pow(current_position.x - x_center, 2) + pow(current_position.y - y_center, 2));

        v_i_allow[i] = sqrt(MAX_ACCELERATION * r_i[i]);
        ROS_INFO(
            "Midpoint of curvature between waypoints [%lf,%lf,%lf] - "
            "[%lf,%lf,%lf] - [%lf,%lf,%lf] "
            "is %lf,%lf - radius: %lf, max allowed velocity: %lf",
            prev_position.x, prev_position.y, prev_position.z, current_position.x, current_position.y,
            current_position.z, next_position.x, next_position.y, next_position.z, x_center, y_center, r_i[i],
            v_i_allow[i]);
      } else
        v_i_allow[i] = MAX_SPEED;
    }
  }

  return v_i_allow;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> velocityPlanSweep(
    const std::vector<geometry_msgs::Pose> &traj, const std::vector<double> &max_vels) {
  std::vector<double> velocity_profile(traj.size(), 0);
  std::vector<double> acceleration_profile(traj.size(), 0);
  std::vector<double> time_profile(traj.size(), 0);
  velocity_profile[0] = MAX_SPEED;
  for (int i = 0; i < traj.size() - 1; i++) {
    double max_vel = max_vels[i + 1] < MAX_SPEED ? max_vels[i + 1] : MAX_SPEED;
    // ROS_INFO("Max velocity at waypoint[%d] = %lf",i,max_vel);
    double along_segment_acc = pow(velocity_profile[i], 2) - pow(max_vel, 2);
    double norm =
        sqrt(pow(traj[i].position.x - traj[i + 1].position.x, 2) + pow(traj[i].position.y - traj[i + 1].position.y, 2));
    along_segment_acc /= -2 * norm;
    along_segment_acc = along_segment_acc < MAX_ACCELERATION ? along_segment_acc : MAX_ACCELERATION;
    acceleration_profile[i] = along_segment_acc;

    if (along_segment_acc > 0) {
      velocity_profile[i + 1] = sqrt(pow(velocity_profile[i], 2) + 2 * along_segment_acc * norm);
      time_profile[i] = (-velocity_profile[i] + velocity_profile[i + 1]) / acceleration_profile[i];
    } else {
      if (velocity_profile[i] > max_vel) {
        velocity_profile[i + 1] = max_vel;
        along_segment_acc = pow(velocity_profile[i], 2) - pow(velocity_profile[i + 1], 2);
        along_segment_acc /= -2 * norm;
        acceleration_profile[i] = along_segment_acc;
        double delta = pow(velocity_profile[i], 2) + 2 * along_segment_acc * norm;

        if (fabs(delta) < EPSILON)  // Watch against floating point errors, sqrt should not take a
                                    // negative value as its input.
          delta = 0;
        time_profile[i] = -velocity_profile[i] + sqrt(delta);
        time_profile[i] /= along_segment_acc;
      } else {
        velocity_profile[i + 1] = velocity_profile[i];
        acceleration_profile[i] = 0;
        time_profile[i] = norm / velocity_profile[i];
      }
    }
  }

  return make_tuple(velocity_profile, acceleration_profile, time_profile);
}

}  // namespace starmac