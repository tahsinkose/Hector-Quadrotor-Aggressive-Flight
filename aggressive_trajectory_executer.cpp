#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_uav_msgs/PoseAction.h>

#define EPSILON 1e-4
#define SPACE 0.8
#define MAX_ACCELERATION 10.0 // 10 m/sn^2
#define MAX_SPEED 4.0 // 4.0 m/sn
#define ACCELERATION_STEP 10

typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

/*
    Sample the trajectory conforming to the defined spacing.
*/
std::vector<geometry_msgs::Pose> sample(std::vector<geometry_msgs::Pose>& traj){
    std::vector<geometry_msgs::Pose> result;
    for(int i=0;i<traj.size()-1;i++){
        double norm = sqrt(pow(traj[i+1].position.x - traj[i].position.x,2) + pow(traj[i+1].position.y - traj[i].position.y,2) + pow(traj[i+1].position.z - traj[i].position.z,2));
        int step = (int) ceil(norm / SPACE);
        double x_factor = (traj[i+1].position.x - traj[i].position.x) / step;
        double y_factor = (traj[i+1].position.y - traj[i].position.y) / step;
        double z_factor = (traj[i+1].position.z - traj[i].position.z) / step;
        for(int j=0;j<step;j++){
            geometry_msgs::Pose wp;
            wp.position.x = traj[i].position.x + j*x_factor;
            wp.position.y = traj[i].position.y + j*y_factor;
            wp.position.z = traj[i].position.z + j*z_factor;
            wp.orientation.w = 1.0;
            result.push_back(wp);
        }
    }
    result.push_back(traj[traj.size()-1]);
    return result;
}
/*
    Sweeping algorithm proposed in the paper.
*/
std::tuple<std::vector<double>,std::vector<double>,std::vector<double> > VelocityPlanSweep(std::vector<geometry_msgs::Pose>& traj,std::vector<double>& max_vels){
    std::vector<double> velocity_profile(traj.size(),0);
    std::vector<double> acceleration_profile(traj.size(),0);
    std::vector<double> time_profile(traj.size(),0);
    velocity_profile[0] = MAX_SPEED;
    for(int i=0;i<traj.size()-1;i++){
        double min_vel = max_vels[i+1] < MAX_SPEED ? max_vels[i+1] : MAX_SPEED;
        //ROS_INFO("Max velocity at waypoint[%d] = %lf",i,min_vel);
        double along_segment_acc = pow(velocity_profile[i],2) - pow(min_vel,2);
        double norm = sqrt(pow(traj[i].position.x - traj[i+1].position.x,2) + pow(traj[i].position.y - traj[i+1].position.y,2));
        along_segment_acc /= -2*norm;
        along_segment_acc = along_segment_acc < MAX_ACCELERATION ? along_segment_acc : MAX_ACCELERATION;
        acceleration_profile[i] = along_segment_acc;
       
        if(along_segment_acc > 0){
            velocity_profile[i+1] = sqrt(pow(velocity_profile[i],2) + 2*along_segment_acc*norm);
            time_profile[i] = (-velocity_profile[i] + velocity_profile[i+1]) / acceleration_profile[i];
        }
        else{
            if(velocity_profile[i] > min_vel){
                velocity_profile[i+1] = min_vel;
                along_segment_acc = pow(velocity_profile[i],2) - pow(velocity_profile[i+1],2);
                along_segment_acc /= -2*norm;
                acceleration_profile[i] = along_segment_acc;
                double delta = pow(velocity_profile[i],2) +  2*along_segment_acc*norm;

                if(fabs(delta) < EPSILON) // Watch against floating point errors, sqrt should not take a negative value as its input.
                    delta = 0;
                time_profile[i] = -velocity_profile[i] + sqrt(delta);
                time_profile[i] /= along_segment_acc;
            }
            else{
                velocity_profile[i+1] = velocity_profile[i];
                acceleration_profile[i] = 0;
                time_profile[i] = norm / velocity_profile[i];
            }
        }
    }

    return make_tuple(velocity_profile,acceleration_profile,time_profile);
}

void createPattern(std::vector<geometry_msgs::Pose>& traj){
    
    geometry_msgs::Pose wp;
    wp.position.x = 0;
    wp.position.y = 0;
    wp.position.z = 10;
    wp.orientation.w = 1;
    traj.push_back(wp);
    // Square pattern
    /*wp.position.x = 15;
    traj.push_back(wp);
    wp.position.y = 15;
    traj.push_back(wp);
    wp.position.x = 0;
    traj.push_back(wp);
    */
    
    // Hexagon pattern
    /*wp.position.x = 15;
    traj.push_back(wp);
    wp.position.x = 30;
    wp.position.y = 15;
    traj.push_back(wp);
    wp.position.x = 15;
    wp.position.y = 30;
    traj.push_back(wp);
    wp.position.x = 0;
    traj.push_back(wp);
    wp.position.x = -15;
    wp.position.y = 15;
    traj.push_back(wp);
    */

    // Star pattern
    /*wp.position.x = 10;
    wp.position.y = 20;
    traj.push_back(wp);
    wp.position.x = 20;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = -5;
    wp.position.y = 15;
    traj.push_back(wp);
    wp.position.x = 25;
    traj.push_back(wp);
    */
    
    // Toothed pattern
    wp.position.x = 5;
    wp.position.y = 10;
    traj.push_back(wp);
    wp.position.x = 10;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 15;
    wp.position.y = 10;
    traj.push_back(wp);
    wp.position.x = 20;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 25;
    wp.position.y = 10;
    traj.push_back(wp);
    wp.position.x = 30;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 35;
    wp.position.y = 10;
    traj.push_back(wp);
    wp.position.x = 40;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 35;
    wp.position.y = -10;
    traj.push_back(wp);
    wp.position.x = 30;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 25;
    wp.position.y = -10;
    traj.push_back(wp);
    wp.position.x = 20;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 15;
    wp.position.y = -10;
    traj.push_back(wp);
    wp.position.x = 10;
    wp.position.y = 0;
    traj.push_back(wp);
    wp.position.x = 5;
    wp.position.y = -10;
    traj.push_back(wp);
    
    wp.position.x = 0;
    wp.position.y = 0;
    traj.push_back(wp);


    traj = sample(traj);
}
int main(int argc,char** argv){
    ros::init(argc,argv,"trajectory_controller");
    ros::NodeHandle nh;
    PoseClient pose_client_("/action/pose",true);
    std::vector<double> c_i,r_i,v_i_allow;
    std::vector<geometry_msgs::Pose> trajectory;

    createPattern(trajectory);
    

    c_i.resize(trajectory.size());
    r_i.resize(trajectory.size(),0);
    v_i_allow.resize(trajectory.size());

    // Compute maximum allowed velocity for each waypoint.
    for(int i=0;i<trajectory.size();i++){
        if(i==0){
            geometry_msgs::Point current_position = trajectory[i].position;
            geometry_msgs::Point next_position = trajectory[i+1].position;
            c_i[i] = 2*((next_position.y * current_position.x) - (current_position.y * next_position.x));
            if(fabs(c_i[i])>EPSILON){
                double x_center, y_center;
                double curr_x_i_y_i_square = pow(current_position.x,2) + pow(current_position.y,2);
                double next_x_i_y_i_square = pow(next_position.x,2) + pow(next_position.y,2);
                x_center = curr_x_i_y_i_square * next_position.y - next_x_i_y_i_square * current_position.y;
                y_center = curr_x_i_y_i_square * -next_position.x + next_x_i_y_i_square * current_position.x;
                x_center /= c_i[i];
                y_center /= c_i[i];
                
                r_i[i] = sqrt(pow(current_position.x - x_center,2) + pow(current_position.y - y_center,2));
                
                ROS_INFO("Midpoint of curvature at waypoint[%d] is %lf,%lf - radius: %lf, max allowed velocity: %lf",i,x_center,y_center,r_i[i],v_i_allow[i]);
                v_i_allow[i] = sqrt(MAX_ACCELERATION * r_i[i]);
            }
            else
                v_i_allow[i] = MAX_SPEED; 
        }
        else if(i==trajectory.size()-1)
            v_i_allow[i] = 0;
        else{
            geometry_msgs::Point prev_position = trajectory[i-1].position;
            geometry_msgs::Point current_position = trajectory[i].position;
            geometry_msgs::Point next_position = trajectory[i+1].position;
            c_i[i] = (next_position.y * (current_position.x - prev_position.x)) + (current_position.y *(prev_position.x - next_position.x))
                     + (prev_position.y * (next_position.x - current_position.x));
            c_i[i] *= 2;
            if(fabs(c_i[i])>EPSILON){
                double x_center, y_center;
                double prev_x_i_y_i_square = pow(prev_position.x,2) + pow(prev_position.y,2);
                double curr_x_i_y_i_square = pow(current_position.x,2) + pow(current_position.y,2);
                double next_x_i_y_i_square = pow(next_position.x,2) + pow(next_position.y,2);
                x_center = prev_x_i_y_i_square * (current_position.y - next_position.y) + curr_x_i_y_i_square * (next_position.y - prev_position.y)
                           + next_x_i_y_i_square * (prev_position.y - current_position.y);
                y_center = prev_x_i_y_i_square * (next_position.x - current_position.x) + curr_x_i_y_i_square * (prev_position.x - next_position.x)
                           + next_x_i_y_i_square * (current_position.x - prev_position.x);
                
                x_center /= c_i[i];
                y_center /= c_i[i];
                
                r_i[i] = sqrt(pow(current_position.x - x_center,2) + pow(current_position.y - y_center,2));
               
                v_i_allow[i] = sqrt(MAX_ACCELERATION * r_i[i]);
                ROS_INFO("Midpoint of curvature between waypoints [%lf,%lf,%lf] - [%lf,%lf,%lf] - [%lf,%lf,%lf] is %lf,%lf - radius: %lf, max allowed velocity: %lf",
                            prev_position.x,prev_position.y,prev_position.z,current_position.x,current_position.y,current_position.z,
                            next_position.x,next_position.y,next_position.z,x_center,y_center,r_i[i],v_i_allow[i]);
            }
            else
                v_i_allow[i] = MAX_SPEED;
        }
        
    }

    // Compute velocity, acceleration and time profiles.
    std::tuple<std::vector<double>,std::vector<double>,std::vector<double> > profiles = VelocityPlanSweep(trajectory,v_i_allow);
    
    std::vector<double> speed_profile = std::get<0>(profiles);
    std::vector<double> acc_profile = std::get<1>(profiles);
    std::vector<double> time_profile = std::get<2>(profiles);
    
    /*for(int i=0;i<speed_profile.size();i++)
        ROS_INFO("Radius, Speed, acceleration and timing at waypoint[%d]: %lf m, %lf m/s, %lf m/s^2, %lf s",i,r_i[i],speed_profile[i],acc_profile[i],time_profile[i]);
    */
    // Move the quadcopter to 10 meters of height.
    hector_uav_msgs::PoseGoal pose_goal;
    geometry_msgs::Pose p;
    p.position.z=10;
    p.orientation.w=1;
    pose_goal.target_pose.pose = p;
    pose_goal.target_pose.header.frame_id="world";
    pose_client_.waitForServer();
    pose_client_.sendGoal(pose_goal);
    ROS_INFO("Sending goal");
    pose_client_.waitForResult();
    ros::Duration(5.0).sleep();

    // Advertise cmd_vel topic to give velocity commands. Note that given 3.0 m/s in x direction does not necessarily mean that
    // quadcopter will be moving with 3.0 m/s in that time instance. In fact, these are acceleration commands. In other applications,
    // they may be negligible, but in this case they are not since aggressive flight requires ultimate obedience to the computed velocities.
    // Therefore, this is an open-loop algorithm.
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::ServiceClient enable_motors = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    hector_uav_msgs::EnableMotors srv;
    srv.request.enable = true;
    if(enable_motors.call(srv)){
        if(srv.response.success)
            ROS_INFO("Motors are enabled");
    }

    for(int i=0;i<speed_profile.size()-1;i++){
        double diff_x = trajectory[i+1].position.x - trajectory[i].position.x;
        double diff_y = trajectory[i+1].position.y - trajectory[i].position.y;
       
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = diff_x;
        vel_msg.linear.y = diff_y;
        double length = sqrt(pow(vel_msg.linear.x,2) + pow(vel_msg.linear.y,2));
        vel_msg.linear.x /= length;
        vel_msg.linear.y /= length;
        vel_msg.linear.x *= speed_profile[i];
        vel_msg.linear.y *= speed_profile[i];
        
        double step =(speed_profile[i+1] - speed_profile[i]) / (double)ACCELERATION_STEP;
        // Discretize acceleration. Since it is impossible to give continuous commands, it is necessary to gradually lower
        // velocity commands.
        for(int j=0;j<ACCELERATION_STEP;j++){
            geometry_msgs::Twist acced_vel_msg;
            acced_vel_msg.linear.x = vel_msg.linear.x + j*step*(diff_x);
            acced_vel_msg.linear.y = vel_msg.linear.y + j*step*(diff_y);
            if(fabs(step)>EPSILON)
                ROS_INFO("Step: %lf | Accelerated velocity %lf,%lf",step,acced_vel_msg.linear.x,acced_vel_msg.linear.y);
            pub_vel.publish(acced_vel_msg);
            double longer = 0.0;
            if(i==speed_profile.size()-2 && j==ACCELERATION_STEP-1)
                longer = 0.05;
            ros::Duration((time_profile[i]/(double)ACCELERATION_STEP) + longer).sleep();
        }
        // This has an improving effect on the final error.   
    }
    
    while(ros::ok()){
        geometry_msgs::Twist empty;
        empty.linear.x = 0; empty.linear.y = 0; empty.linear.z = 0;
        pub_vel.publish(empty);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 0;
}