#include <rclcpp/rclcpp.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

class FlatAreaDetector : public rclcpp::Node {
public:
    FlatAreaDetector() : Node("landing_spot_detector") {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rtabmap/cloud_ground", 10,
            std::bind(&FlatAreaDetector::pointCloudCallback, this, std::placeholders::_1)
        );

        landing_spot_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/landing_spots", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Landing Spot Detector Node Initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr landing_spot_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::vector<pcl::PointXYZ> landing_points_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
            return;
        }

        // Get min and max Z values before filtering
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();
        for (const auto& point : cloud->points) {
            if (!std::isfinite(point.z)) continue;
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_z = std::max(max_z, static_cast<double>(point.z));
        }
        RCLCPP_INFO(this->get_logger(), "Z range before filtering: min=%.2f, max=%.2f", min_z, max_z);

        // Adjusted Pass-through Filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z + 0.1, max_z - 0.1); // Dynamic range
        pass.filter(*cloud);

        if (cloud->size() < 100) {
            RCLCPP_WARN(this->get_logger(), "Too few points after filtering!");
            return;
        }

        // Normal Estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        ne.setRadiusSearch(0.1);  // Increased for better stability
        ne.compute(*cloud_normals);

        // Plane Segmentation
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(500);  // Increased iterations
        seg.setDistanceThreshold(0.1);  // More tolerance
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 50) {
            RCLCPP_WARN(this->get_logger(), "No suitable plane detected (inliers: %zu)", inliers->indices.size());
            return;
        }

        // Extract Planar Points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*plane_cloud);

        // Flatness Check
        Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f centroid;
        pcl::computeMeanAndCovarianceMatrix(*plane_cloud, covariance_matrix, centroid);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
        Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();

        double flatness_ratio = eigen_values[0] / eigen_values[2];  // Smallest / Largest eigenvalue
        if (flatness_ratio > 0.05) {  // More strict flatness threshold
            RCLCPP_WARN(this->get_logger(), "Surface is not flat enough!");
            return;
        }

        pcl::PointXYZ best_point(centroid[0], centroid[1], centroid[2]);
        landing_points_.push_back(best_point);

        // Publish landing spot
        geometry_msgs::msg::Point landing_spot;
        landing_spot.x = best_point.x;
        landing_spot.y = best_point.y;
        landing_spot.z = best_point.z;
        landing_spot_pub_->publish(landing_spot);

        // Publish visualization marker
        // Publish visualization marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "landing_spot";
        marker.id = static_cast<int>(landing_points_.size()); // Unique ID for each marker
        marker.type = visualization_msgs::msg::Marker::CYLINDER;  // Flat plate
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = landing_spot;

        // --- 🔧 FIX: Ensure proper orientation using Eigen Quaternion ---
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f z_axis(0, 0, 1);

        // Compute rotation quaternion
        Eigen::Quaternionf q;
        Eigen::Vector3f rotation_axis = z_axis.cross(plane_normal);
        double rotation_angle = acos(z_axis.dot(plane_normal));

        if (rotation_axis.norm() > 1e-6) {
            q = Eigen::AngleAxisf(rotation_angle, rotation_axis.normalized());
        } else {
            q = Eigen::Quaternionf::Identity();  // No rotation needed
        }

        // Apply quaternion to marker
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Set marker scale (cylinder plate dimensions)
        marker.scale.x = 0.3;  // Diameter
        marker.scale.y = 0.3;  // Diameter
        marker.scale.z = 0.02; // Small height to look like a plate

        // Set marker color to green
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Publish marker
        marker_pub_->publish(marker);



    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlatAreaDetector>());
    rclcpp::shutdown();
    return 0;
}
