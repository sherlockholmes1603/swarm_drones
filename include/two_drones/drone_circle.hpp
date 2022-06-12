#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>



class circle
{

public:
    circle(ros::NodeHandle nh);

    ~circle();

    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);

    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);

    geometry_msgs::Point get_current_location();

    float get_current_heading();

    void set_heading(float heading);

    void set_destination(float x, float y, float z, float psi);

    void set_destination_lla(float lat, float lon, float alt, float heading);

    void set_destination_lla_raw(float lat, float lon, float alt, float heading);

    int wait4connect();

    int init_publisher_subscriber();

    int takeoff_global(float lat, float lon, float alt);

    int set_yaw(float angle, float speed, float dir, float absolute_rel);

    int auto_set_current_waypoint(int seq);

    int set_speed(float speed__mps);

    int land();

    int set_mode(std::string mode);

    int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);

    int takeoff(float takeoff_alt);

    int arm();

    int initialize_local_frame();

    void getParams();

    void timerCallback_(const ros::TimerEvent &event);

    struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
    };

private:
    mavros_msgs::State current_state_g;
    nav_msgs::Odometry current_pose_g;
    geometry_msgs::Pose correction_vector_g;
    geometry_msgs::Point local_offset_pose_g;
    geometry_msgs::PoseStamped waypoint_g;
    std::string droneName;

    float current_heading_g;
    float local_offset_g;
    float correction_heading_g = 0;
    float local_desired_heading_g; 
    bool armed_hua;
    bool takeoff_hua;
    bool connect_hua;

    ros::Timer *timer_;



    ros::NodeHandle _nh;
    ros::Publisher local_pos_pub;
    ros::Publisher global_lla_pos_pub;
    ros::Publisher global_lla_pos_pub_raw;
    ros::Subscriber currentPos;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient land_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::ServiceClient command_client;
    ros::ServiceClient auto_waypoint_pull_client;
    ros::ServiceClient auto_waypoint_push_client;
    ros::ServiceClient auto_waypoint_set_current_client;
};


