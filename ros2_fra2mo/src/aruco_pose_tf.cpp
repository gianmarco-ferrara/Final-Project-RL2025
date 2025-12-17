#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/exceptions.h>
#include <cmath>

using namespace std::chrono_literals;

class ArucoPoseTF : public rclcpp::Node
{
public:
    ArucoPoseTF()
        : Node("aruco_pose_tf"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
          tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
          pose_stabilized_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("camera_link_optical_frame", "camera_link_optical");
        this->declare_parameter<std::string>("camera_link_frame", "camera_link");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame", "fra2mo/odom");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("world_frame", "world");

        // Get parameters
        this->get_parameter("camera_link_optical_frame", camera_link_optical_frame_);
        this->get_parameter("camera_link_frame", camera_link_frame_);
        this->get_parameter("base_link_frame", base_link_frame_);
        this->get_parameter("base_footprint_frame", base_footprint_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("world_frame", world_frame_);

        // Subscriber to ArUco pose
        aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&ArucoPoseTF::handleArucoPose, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Aruco TF Publisher node started.");
    }

private:
    void handleArucoPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // If pose has already been stabilized, don't process further
        if (pose_stabilized_)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped transformed_pose;
        try
        {
            // Lookup required transforms
            auto transform_camera_optical_to_camera = 
                tf_buffer_->lookupTransform(camera_link_frame_, camera_link_optical_frame_, tf2::TimePointZero);
            auto transform_camera_to_base_link = 
                tf_buffer_->lookupTransform(base_link_frame_, camera_link_frame_, tf2::TimePointZero);
            auto transform_base_link_to_base_footprint = 
                tf_buffer_->lookupTransform(base_footprint_frame_, base_link_frame_, tf2::TimePointZero);
            auto transform_base_footprint_to_odom = 
                tf_buffer_->lookupTransform(odom_frame_, base_footprint_frame_, tf2::TimePointZero);
            auto transform_odom_to_map = 
                tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);

            // Transform the pose through all frames
            geometry_msgs::msg::PoseStamped pose_in_camera_link;
            tf2::doTransform(*msg, pose_in_camera_link, transform_camera_optical_to_camera);

            geometry_msgs::msg::PoseStamped pose_in_base_link;
            tf2::doTransform(pose_in_camera_link, pose_in_base_link, transform_camera_to_base_link);

            geometry_msgs::msg::PoseStamped pose_in_base_footprint;
            tf2::doTransform(pose_in_base_link, pose_in_base_footprint, transform_base_link_to_base_footprint);

            geometry_msgs::msg::PoseStamped pose_in_odom;
            tf2::doTransform(pose_in_base_footprint, pose_in_odom, transform_base_footprint_to_odom);

            tf2::doTransform(pose_in_odom, transformed_pose, transform_odom_to_map);

            // Stabilize pose by waiting a few cycles (simulate stabilization)
            rclcpp::Rate stabilization_rate(2);  // Wait 2 seconds for stabilization
            stabilization_rate.sleep();

            // Apply map-to-world transform
            geometry_msgs::msg::PoseStamped pose_in_world;
            applyMapToWorldTransform(transformed_pose, pose_in_world);

            // Save the final transformed pose in world frame
            saved_pose_ = pose_in_world;
            pose_stabilized_ = true;

            // Log the transformed pose in the world frame
            RCLCPP_INFO(this->get_logger(), "Aruco pose transformed to world frame:");
            RCLCPP_INFO(this->get_logger(), "Position (x, y, z): (%f, %f, %f)",
                        saved_pose_.pose.position.x, saved_pose_.pose.position.y, saved_pose_.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation (x, y, z, w): (%f, %f, %f, %f)",
                        saved_pose_.pose.orientation.x, saved_pose_.pose.orientation.y,
                        saved_pose_.pose.orientation.z, saved_pose_.pose.orientation.w);

            // Broadcast the static transform (tf_static) for ArUco marker in world frame
            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header.stamp = this->now();
            tf_stamped.header.frame_id = world_frame_;
            tf_stamped.child_frame_id = "aruco_marker";

            tf_stamped.transform.translation.x = saved_pose_.pose.position.x;
            tf_stamped.transform.translation.y = saved_pose_.pose.position.y;
            tf_stamped.transform.translation.z = saved_pose_.pose.position.z;

            tf_stamped.transform.rotation.x = saved_pose_.pose.orientation.x;
            tf_stamped.transform.rotation.y = saved_pose_.pose.orientation.y;
            tf_stamped.transform.rotation.z = saved_pose_.pose.orientation.z;
            tf_stamped.transform.rotation.w = saved_pose_.pose.orientation.w;

            // Publish static transform
            tf_broadcaster_->sendTransform(tf_stamped);

            // Log debug information
            RCLCPP_INFO(this->get_logger(),
                        "Published static transform: [frame: %s -> aruco_marker]", world_frame_.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "Translation:\n"
                        "    x : %.2f\n"
                        "    y : %.2f\n"
                        "    z : %.2f\n"
                        "Orientation:\n"
                        "    x : %.2f\n"
                        "    y : %.2f\n"
                        "    z : %.2f\n"
                        "    w : %.2f",
                        saved_pose_.pose.position.x, saved_pose_.pose.position.y, saved_pose_.pose.position.z,
                        saved_pose_.pose.orientation.x, saved_pose_.pose.orientation.y,
                        saved_pose_.pose.orientation.z, saved_pose_.pose.orientation.w);

        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        }
    }

    void applyMapToWorldTransform(const geometry_msgs::msg::PoseStamped &pose_in_map,
                                  geometry_msgs::msg::PoseStamped &pose_in_world)
    {
        // Fixed transformation: map to world
        double x_map = pose_in_map.pose.position.x;
        double y_map = pose_in_map.pose.position.y;
        double yaw_map = -M_PI_2;  // -pi/2 rad (90 degrees clockwise)

        // Translation offset
        double x_offset = -3.0;
        double y_offset = 3.5;

        // Apply rotation (yaw) from map frame to world frame
        pose_in_world.pose.position.x = x_offset + (x_map * cos(yaw_map) - y_map * sin(yaw_map));
        pose_in_world.pose.position.y = y_offset + (x_map * sin(yaw_map) + y_map * cos(yaw_map));

        // Keep the z value the same as in the map frame
        pose_in_world.pose.position.z = pose_in_map.pose.position.z;

        // Orientation: No change in orientation since it's the same for both frames
        pose_in_world.pose.orientation = pose_in_map.pose.orientation;
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    bool pose_stabilized_;
    geometry_msgs::msg::PoseStamped saved_pose_;
    std::string camera_link_optical_frame_;
    std::string camera_link_frame_;
    std::string base_link_frame_;
    std::string base_footprint_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string world_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoPoseTF>());
    rclcpp::shutdown();
    return 0;
}
