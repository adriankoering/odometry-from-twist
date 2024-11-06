#include <mutex>
#include <string>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

struct Pose {
  double x, y, theta; // 2d-pose
  double vx, vy, omega; // 2d-velocity
};

struct OdometryFromTwist {
  OdometryFromTwist () {
    ros::NodeHandle nh;
    ros::NodeHandle pnh{"~"};
    
    pnh.param("odom_frame", odom_frame, odom_frame);
    pnh.param("base_frame", base_frame, base_frame);
    pnh.param("publish_odom_tf", publish_odom_tf, true);


    last_time_ = ros::Time(0);

    // initiate subscribing / publishing objects
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    twist_sub_ = nh.subscribe("twist", 10, &geometry_msgs::TwistWithCovarianceStamped, this);

    // Periodic timer republishes initial / last known state without relying on can messages
    downtime = nh.createTimer(ros::Duration(1.0), &OdometryFromTwist::timerCallback, this);
  }

  ~OdometryFromTwist() = default;

  void msgCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twst_msg) {
    std::lock_guard<std::mutex> guard(callback_mutex);
    updateHeader(ros::Time::now());
    // Calculate the time delta for which current velocities are applicable
    double dt = (hdr.stamp - last_time).toSec();
    
    // Integrate position through velocities obtained from canbus.
    // Double Ackermann Odometry implemented by halving vehicles wheel-base [m]
    accumulate(kinemantics_msg->velocity, 
               kinemantics_msg->front_steering_angle, 
               0.5 * wheel_base, dt);
  
    if (publish_odom_tf) {
      odom_broadcaster.sendTransform(createTransform());
    }
    odom_pub_.publish(createMsg());
    
    // refresh last timestamp
    last_time = hdr.stamp; 
  }

  void timerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> guard(callback_mutex);
    updateHeader(ros::Time::now());
    if (publish_odom_tf) 
      odom_broadcaster.sendTransform(createTransform());
    odom_pub_.publish(createMsg());
  }

protected:
  double updateHeader(const ros::Time& now) {
    hdr.seq += 1;
    hdr.stamp = now;
  }

  void accumulate(double velocity, double steering_angle, double wheel_base, double dt) {
    // set new velocities
    pose.vx = velocity * 1.0; 
    pose.vy = velocity * 0.0;
    pose.omega = pose.vx / ( wheel_base / tan(steering_angle) );

    // derive new position based on current velocities and old position
    pose.x += (pose.vx * cos(pose.theta) - pose.vy * sin(pose.theta)) * dt; 
    pose.y += (pose.vx * sin(pose.theta) + pose.vy * cos(pose.theta)) * dt;
    pose.theta += pose.omega * dt;
  }

  auto createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  geometry_msgs::TransformStamped createTransform() {
    geometry_msgs::TransformStamped tf;

    tf.header = hdr;
    tf.child_frame_id = base_frame;

    tf.transform.translation.x = pose.x;
    tf.transform.translation.y = pose.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = createQuaternionMsgFromYaw(pose.theta);

    return tf;
  }

  nav_msgs::Odometry createMsg() {
    nav_msgs::Odometry odom;
    odom.header = hdr;
    odom.child_frame_id = base_frame;

    odom.pose.pose.position.x = pose.x; // set the position
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0; // 'coz just movement in a x-y-plane
    odom.pose.pose.orientation = createQuaternionMsgFromYaw(pose.theta);

    odom.twist.twist.linear.x = pose.vx; // set the velocities
    odom.twist.twist.linear.y = pose.vy;
    odom.twist.twist.angular.z = pose.omega;

    double tvar = 0.02, avar = 0.07;
    odom.twist.covariance = {
      tvar, 0, 0, 0, 0, 0,
      0, tvar, 0, 0, 0, 0,
      0, 0, tvar, 0, 0, 0,
      0, 0, 0, avar, 0, 0,
      0, 0, 0, 0, avar, 0,
      0, 0, 0, 0, 0, avar,
    };

    double tvar = 0.02, avar = 0.07;
    odom.twist.covariance = {
      tvar, 0, 0, 0, 0, 0,
      0, tvar, 0, 0, 0, 0,
      0, 0, tvar, 0, 0, 0,
      0, 0, 0, avar, 0, 0,
      0, 0, 0, 0, avar, 0,
      0, 0, 0, 0, 0, avar,
    };

    return odom;
  }


private:
  ros::Publisher odom_pub_;
  ros::Subscriber twist_sub_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  // Accumulated position = velocity * dtime
  Pose pose_;
  ros::Time last_time_;

  // One downside of the interrupt based can-reader node is the lack of (empty) messages
  // during shuttle startup. This timer keeps publishing the initial / last known state
  ros::Timer downtime_;
  // Mutex ensures only one (publishing) callback is active at the same time
  // (s.t. the msgCallback doesnt write into pose while the timer attempts to publish)
  std::mutex callback_mutex_;

  // ROS Parameters
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  bool publish_odom_tf_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_from_twistnode");

  OdometryFromTwist node;
  ros::spin();
}
