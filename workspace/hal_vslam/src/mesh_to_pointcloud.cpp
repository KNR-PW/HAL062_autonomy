#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nvblox_msgs/msg/mesh.hpp>


namespace hal_vslam {

  class MeshToPointCloudComponent : public rclcpp::Node {
  public:
    explicit MeshToPointCloudComponent(const rclcpp::NodeOptions & options)
    : Node("mesh_to_pointcloud", options)
    {
      mesh_sub_ = this->create_subscription<nvblox_msgs::msg::Mesh>(
        "/nvblox_node/mesh", rclcpp::SensorDataQoS(),
        std::bind(&MeshToPointCloudComponent::meshCallback, this, std::placeholders::_1));

      cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/nvblox/mesh_pointcloud", rclcpp::SensorDataQoS());
    }

  private:
    void meshCallback(const nvblox_msgs::msg::Mesh::SharedPtr msg) {
      size_t num_points = 0;
      for (const auto& block : msg->blocks) {
        num_points += block.vertices.size();
      }

      if (num_points > 0) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = msg->header.frame_id;
        cloud.header.stamp = msg->header.stamp;
        // cloud.header.stamp = this->now();

        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(num_points);
        cloud.is_bigendian = false;
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto & block : msg->blocks) {
          for (const auto & v : block.vertices) {
            *iter_x = v.x;
            *iter_y = v.y;
            *iter_z = v.z;
            ++iter_x; ++iter_y; ++iter_z;
          }
        }

        if (cloud.point_step != 12) {
          RCLCPP_WARN(this->get_logger(), "point_step is %u, expected 12; repacking manually to avoid padding.", cloud.point_step);
        }

        cloud_pub_->publish(cloud);
      }
    }

    rclcpp::Subscription<nvblox_msgs::msg::Mesh>::SharedPtr mesh_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  };

} // namespace hal_vslam

// Register component with class loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hal_vslam::MeshToPointCloudComponent)