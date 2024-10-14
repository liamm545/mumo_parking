#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

class PathVisualizer {
public:
    PathVisualizer() {
        // 변환된 경로를 발행할 퍼블리셔
        path_pub_ = nh_.advertise<nav_msgs::Path>("transformed_path", 1);
        // 기존 경로를 구독하는 서브스크라이버
        path_sub_ = nh_.subscribe("/se2_planner_node/ompl_rs_planner_ros/nav_msgs_path", 10, &PathVisualizer::pathCallback, this);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 새로운 경로 메시지 생성
        nav_msgs::Path transformed_path;
        transformed_path.header.frame_id = "velodyne";  // 변환할 타겟 프레임
        transformed_path.header.stamp = ros::Time::now();

        // tf 리스너 및 변환 객체
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            // map에서 velodyne 프레임으로 변환
            listener.waitForTransform("velodyne", msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("velodyne", msg->header.frame_id, ros::Time(0), transform);

            // 기존 경로의 각 포즈를 변환
            for (const auto& pose : msg->poses) {
                geometry_msgs::PoseStamped transformed_pose;
                transformed_pose.header.frame_id = "velodyne";
                transformed_pose.header.stamp = ros::Time::now();

                tf::Vector3 point_in_target_frame = transform * tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                transformed_pose.pose.position.x = point_in_target_frame.x();
                transformed_pose.pose.position.y = point_in_target_frame.y();
                transformed_pose.pose.position.z = point_in_target_frame.z();
                
                // 회전 정보도 변환
                tf::Quaternion quat_orig(
                    pose.pose.orientation.x, 
                    pose.pose.orientation.y, 
                    pose.pose.orientation.z, 
                    pose.pose.orientation.w
                );
                tf::Quaternion quat_transformed = transform.getRotation() * quat_orig;
                transformed_pose.pose.orientation.x = quat_transformed.x();
                transformed_pose.pose.orientation.y = quat_transformed.y();
                transformed_pose.pose.orientation.z = quat_transformed.z();
                transformed_pose.pose.orientation.w = quat_transformed.w();

                transformed_path.poses.push_back(transformed_pose);
            }

            // 변환된 경로를 발행
            path_pub_.publish(transformed_path);

        } catch (tf::TransformException& ex) {
            ROS_WARN("Transform warning: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_visualizer");
    PathVisualizer pv;
    ros::spin();
    return 0;
}
