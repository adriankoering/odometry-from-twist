#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

using Covariance6d = Eigen::Matrix<double, 6, 6>;

struct OdometryFromTwist {
  OdometryFromTwist() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh{"~"};

    pnh.param("odom_frame", odom_frame_, odom_frame_);
    pnh.param("base_frame", base_frame_, base_frame_);
    pnh.param("publish_odom_tf", publish_odom_tf_, publish_odom_tf_);

    // initiate publisher first s.t. its ready before the first callback arrives
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    twist_sub_ =
        nh.subscribe("twist", 10, &OdometryFromTwist::msgCallback, this);
  }

  ~OdometryFromTwist() = default;

  void msgCallback(
      const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &twist_msg) {

    const auto &hdr = twist_msg->header;
    if (last_time_.isZero()) {
      last_time_ = hdr.stamp;
      return;
    }

    // Calculate the time delta for which the current velocities are applicable
    double dt = (hdr.stamp - last_time_).toSec();

    // Integrate position through velocities obtained from twist_msg
    accumulate(twist_msg->twist, dt);

    if (publish_odom_tf_)
      odom_broadcaster_.sendTransform(createTransform(hdr));
    odom_pub_.publish(createOdometry(hdr));

    // update time.stamp for next iteration
    last_time_ = hdr.stamp;
  }

protected:
  void accumulate(const geometry_msgs::TwistWithCovariance &twist, double dt) {

    // Structured (un)binding extracts all components
    const auto &[vx, vy, vz] = twist.twist.linear;
    const auto &[ox, oy, oz] = twist.twist.linear;

    Eigen::Vector3d linear{vx, vy, vz};
    Eigen::Vector3d angular{ox, oy, oz};

    // Rotate Linear Velocity into local robot frame
    Eigen::Vector3d linear_local = pose_.rotation() * linear;
    // And add a small step into this direction onto
    //  the current pose's selection
    pose_.translation() += dt * linear_local;

    // Add a small step of rotation onth the current pose's rotation
    Eigen::Quaterniond angular_quat{
        Eigen::AngleAxisd(dt * angular.x(), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(dt * angular.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(dt * angular.z(), Eigen::Vector3d::UnitZ())};
    // TODO: figure out if this does what it should :)
    // pose_ = pose_.rotation() * angular_quat * pose_.translation();

    // It should implement this, but in 3d :)
    // // derive new position based on current velocities and old position
    // pose.x += (pose.vx * cos(pose.theta) - pose.vy * sin(pose.theta)) * dt;
    // pose.y += (pose.vx * sin(pose.theta) + pose.vy * cos(pose.theta)) * dt;
    // pose.theta += pose.omega * dt;

    // TODO: Update Covariance Matrix
    // ...
  }

  geometry_msgs::TransformStamped createTransform(const std_msgs::Header &hdr) {
    geometry_msgs::TransformStamped tf = tf2::eigenToTransform(pose_);

    tf.header = hdr;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;

    return tf;
  }

  nav_msgs::Odometry createOdometry(const std_msgs::Header &hdr) {
    nav_msgs::Odometry odom;
    odom.header = hdr;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // TODO:
    // odom.pose.pose.position.x = pose.x; // set the position
    // odom.pose.pose.position.y = pose.y;
    // odom.pose.pose.position.z = 0.0; // 'coz just movement in a x-y-plane
    // odom.pose.pose.orientation = createQuaternionMsgFromYaw(pose.theta);

    // odom.twist.twist.linear.x = pose.vx; // set the velocities
    // odom.twist.twist.linear.y = pose.vy;
    // odom.twist.twist.angular.z = pose.omega;

    // double tvar = 0.02, avar = 0.07;
    // odom.twist.covariance = {
    //     tvar, 0, 0, 0,    0, 0, 0, tvar, 0, 0, 0,    0, 0, 0, tvar, 0, 0, 0,
    //     0,    0, 0, avar, 0, 0, 0, 0,    0, 0, avar, 0, 0, 0, 0,    0, 0,
    //     avar,
    // };

    // double tvar = 0.02, avar = 0.07;
    // odom.twist.covariance = {
    //     tvar, 0, 0, 0,    0, 0, 0, tvar, 0, 0, 0,    0, 0, 0, tvar, 0, 0, 0,
    //     0,    0, 0, avar, 0, 0, 0, 0,    0, 0, avar, 0, 0, 0, 0,    0, 0,
    //     avar,
    // };

    return odom;
  }

private:
  ros::Publisher odom_pub_;
  ros::Subscriber twist_sub_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  // Accumulated position = velocity * dtime
  ros::Time last_time_;
  Eigen::Isometry3d pose_;

  // ROS Parameters
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  bool publish_odom_tf_{true};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_from_twistnode");

  OdometryFromTwist node;
  ros::spin();
}
