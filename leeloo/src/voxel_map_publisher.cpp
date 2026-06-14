#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <curobo_msgs/srv/get_voxel_grid.hpp>
#include <curobo_msgs/msg/sparse_voxel_grid.hpp>

#include <chrono>
#include <vector>

using GetVoxelGridClient = rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>;
using Clock = std::chrono::steady_clock;

class VoxelMapPublisher : public rclcpp::Node
{
public:
  VoxelMapPublisher() : Node("voxel_map_publisher")
  {
    declare_parameter("frame_id", "base_0");
    declare_parameter("rate_hz", 5.0);
    declare_parameter("use_service", false);  // If true, also poll the legacy service.

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("voxel_map", qos);

    // Sparse topic subscriber — primary benchmarking path (Tier 2).
    sparse_sub_ = create_subscription<curobo_msgs::msg::SparseVoxelGrid>(
      "/unified_planner/voxel_grid_sparse",
      rclcpp::QoS(10),
      [this](curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg) {
        on_sparse_message(msg);
      });
    RCLCPP_INFO(get_logger(), "Subscribed to /unified_planner/voxel_grid_sparse");

    // Legacy service path — optional, for comparison against Tier 1 numbers.
    if (get_parameter("use_service").as_bool()) {
      client_ = create_client<curobo_msgs::srv::GetVoxelGrid>("/unified_planner/get_voxel_grid");
      RCLCPP_INFO(get_logger(), "Waiting for /unified_planner/get_voxel_grid ...");
      client_->wait_for_service();
      RCLCPP_INFO(get_logger(), "Service ready — also polling at %.1f Hz.",
                  get_parameter("rate_hz").as_double());

      double period_s = 1.0 / get_parameter("rate_hz").as_double();
      timer_ = create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this]() { send_service_request(); });
    }
  }

private:
  void on_sparse_message(curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg)
  {
    auto t_recv = Clock::now();

    double inter_arrival_ms = 0.0;
    if (sparse_count_ > 0) {
      inter_arrival_ms = std::chrono::duration<double, std::milli>(
        t_recv - t_last_sparse_).count();
    }
    t_last_sparse_ = t_recv;
    ++sparse_count_;

    // Reconstruct dense grid from sparse indices and time it.
    const auto sx = msg->size_x;
    const auto sy = msg->size_y;
    const auto sz = msg->size_z;
    const double vs = msg->resolution;
    const double ox = msg->origin.x;
    const double oy = msg->origin.y;
    const double oz = msg->origin.z;
    const double half = vs / 2.0;

    auto t_recon_start = Clock::now();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = get_parameter("frame_id").as_string();
    marker.header.stamp    = get_clock()->now();
    marker.ns              = "voxel_grid";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = vs;
    marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
    marker.points.reserve(msg->occupied_indices.size());

    const uint32_t syz = sy * sz;
    for (int32_t linear : msg->occupied_indices) {
      uint32_t u = static_cast<uint32_t>(linear);
      uint32_t gx = u / syz;
      uint32_t rem = u % syz;
      uint32_t gy = rem / sz;
      uint32_t gz = rem % sz;

      geometry_msgs::msg::Point p;
      p.x = ox + gx * vs + half;
      p.y = oy + gy * vs + half;
      p.z = oz + gz * vs + half;
      marker.points.push_back(p);
    }

    auto t_recon_end = Clock::now();
    double recon_ms = std::chrono::duration<double, std::milli>(t_recon_end - t_recon_start).count();

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    pub_->publish(array);

    RCLCPP_INFO(get_logger(),
      "[sparse #%u] occupied=%zu | grid=%u×%u×%u | inter-arrival=%.1f ms | reconstruct=%.2f ms | vs=%.3f m",
      sparse_count_, msg->occupied_indices.size(), sx, sy, sz,
      inter_arrival_ms, recon_ms, vs);
  }

  // ── Legacy service path (kept for direct RTT comparison) ─────────────────

  void send_service_request()
  {
    if (svc_pending_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Previous service call still pending — service too slow for requested rate.");
      return;
    }

    auto req = std::make_shared<curobo_msgs::srv::GetVoxelGrid::Request>();
    req->bbox_min_x = -1.28; req->bbox_min_y = -1.28; req->bbox_min_z = -1.28;
    req->bbox_max_x =  1.28; req->bbox_max_y =  1.28; req->bbox_max_z =  1.28;

    t_send_ = Clock::now();
    svc_pending_ = true;
    ++svc_call_count_;

    client_->async_send_request(
      req,
      [this](GetVoxelGridClient::SharedFuture future) { on_service_response(future); });
  }

  void on_service_response(GetVoxelGridClient::SharedFuture future)
  {
    auto t_recv = Clock::now();
    double rtt_ms = std::chrono::duration<double, std::milli>(t_recv - t_send_).count();
    svc_pending_ = false;

    auto resp = future.get();
    const auto & grid = resp->voxel_grid;

    size_t occupied = 0;
    for (auto v : grid.data) {
      if (v != 0u) ++occupied;
    }

    RCLCPP_INFO(get_logger(),
      "[service #%u] RTT=%.1f ms | occupied=%zu | grid=%u×%u×%u | vs=%.3f m",
      svc_call_count_, rtt_ms, occupied, grid.size_x, grid.size_y, grid.size_z,
      grid.resolutions.x);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<curobo_msgs::msg::SparseVoxelGrid>::SharedPtr sparse_sub_;
  rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>::SharedPtr          client_;
  rclcpp::TimerBase::SharedPtr                                       timer_;

  // Sparse path state
  Clock::time_point t_last_sparse_;
  uint32_t sparse_count_ {0};

  // Service path state
  Clock::time_point t_send_;
  bool     svc_pending_    {false};
  uint32_t svc_call_count_ {0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
