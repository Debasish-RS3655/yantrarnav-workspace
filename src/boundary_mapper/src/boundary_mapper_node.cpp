#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/msg/marker_array.hpp>

class BoundaryMapper : public rclcpp::Node
{
public:
  BoundaryMapper() : Node("boundary_mapper")
  {
    RCLCPP_INFO(this->get_logger(), "Boundary Mapper Node has started.");

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rtabmap/cloud_ground", 10,
      std::bind(&BoundaryMapper::pointCloudCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/boundary_markers", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointIndices::Ptr yellow_indices(new pcl::PointIndices());
    int yellow_count = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      const auto& point = cloud->points[i];
      float total = point.r + point.g + point.b;
      if (total > 0) 
      {
        float r_norm = point.r / total;
        float g_norm = point.g / total;
        float b_norm = point.b / total;

        if (r_norm > 0.35 && g_norm > 0.35 && b_norm < 0.25) // Any shade of yellow
        {
          yellow_indices->indices.push_back(i);
          yellow_count++;
        }
      }
    }

    if (yellow_count > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Detected %d yellow points.", yellow_count);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No yellow points detected. Adjust lighting or position.");
      return;
    }

    // Extract yellow points into a separate cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr yellow_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(yellow_indices);
    extract.setNegative(false);
    extract.filter(*yellow_cloud);

    // Remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(yellow_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*yellow_cloud);

    // Publish markers
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "boundary_yellow";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& p : yellow_cloud->points)
    {
      geometry_msgs::msg::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryMapper>());
  rclcpp::shutdown();
  return 0;
}
