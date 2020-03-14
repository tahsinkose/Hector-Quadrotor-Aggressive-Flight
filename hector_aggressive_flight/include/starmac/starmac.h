#include <geometry_msgs/Pose.h>
#include <map>

#define OPERATION_ALTITUDE 20
#define EPSILON 1e-4
#define SPACE 0.8
#define MAX_ACCELERATION 10.0  // 10 m/sn^2
#define MAX_SPEED 4.0          // 4.0 m/sn
#define ACCELERATION_STEP 10

namespace starmac {
struct XYZ {
  double x;
  double y;
  double z;
  XYZ(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};


/**
 * @brief Sample the trajectory conforming to the defined spacing.
 */
std::vector<geometry_msgs::Pose> sample(std::vector<geometry_msgs::Pose> &traj);

std::vector<geometry_msgs::Pose> createPattern(std::string &&pattern);

std::vector<double> constrainCrossTrackAcceleration(const std::vector<geometry_msgs::Pose> &trajectory);

/**
 * @brief Sweeping algorithm proposed in the paper.
 */
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> velocityPlanSweep(
    const std::vector<geometry_msgs::Pose> &traj, const std::vector<double> &max_vels);

}  // namespace starmac