/**
 * pointcloud_voxelizer/src/voxelizer_node.cpp
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * C++ port of the Python voxelizer_node.  Uses PCL VoxelGrid for downsampling
 * and pcl::transformPointCloud for the sensor→map transform.  Map subtraction
 * uses a std::unordered_set<VoxelKey> with Chebyshev-neighbourhood lookup —
 * there is no PCL primitive for this operation.
 *
 * Pipeline per scan:
 *   raw LiDAR (sensor frame)
 *     → range filter (sensor frame, radial)
 *     → transform to map frame via pcl_pose  (pcl::transformPointCloud)
 *     → voxel downsample                     (pcl::VoxelGrid)
 *     → snap every point to its voxel centre
 *     → subtract voxels within subtraction_radius Chebyshev distance of map
 *     → publish /voxelized_cloud  +  /voxel_markers
 *     → publish /foreground_cloud +  /foreground_markers
 *
 * All numeric parameters are live-tunable via `ros2 param set`.
 * Topic parameters require a node restart.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

// ---------------------------------------------------------------------------
// VoxelKey  –  integer (ix, iy, iz) triplet with a good hash
// ---------------------------------------------------------------------------

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey & o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey & k) const
    {
        // Boost-style hash combining for three 32-bit integers
        size_t h = 0;
        auto combine = [&h](int32_t v) {
            h ^= static_cast<size_t>(std::hash<int32_t>{}(v))
                 + 0x9e3779b9u + (h << 6) + (h >> 2);
        };
        combine(k.x);
        combine(k.y);
        combine(k.z);
        return h;
    }
};

using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

// ---------------------------------------------------------------------------
// Height-based colour map  (blue → green → red, same as Python version)
// ---------------------------------------------------------------------------

static void heightColormap(
    const std::vector<float> & z_vals,
    std::vector<float> & r_out,
    std::vector<float> & g_out,
    std::vector<float> & b_out)
{
    const size_t n = z_vals.size();
    if (n == 0) return;

    float z_min = *std::min_element(z_vals.begin(), z_vals.end());
    float z_max = *std::max_element(z_vals.begin(), z_vals.end());
    float range = (z_max - z_min > 1e-6f) ? (z_max - z_min) : 1e-6f;

    r_out.resize(n);
    g_out.resize(n);
    b_out.resize(n);

    for (size_t i = 0; i < n; ++i) {
        float t = std::clamp((z_vals[i] - z_min) / range, 0.0f, 1.0f);
        r_out[i] = std::clamp(2.0f * t - 1.0f,          0.0f, 1.0f);
        g_out[i] = 1.0f - 2.0f * std::abs(t - 0.5f);
        b_out[i] = std::clamp(1.0f - 2.0f * t,          0.0f, 1.0f);
    }
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class VoxelizerNode : public rclcpp::Node
{
public:
    explicit VoxelizerNode()
    : Node("voxelizer_node")
    {
        // ---- declare parameters -----------------------------------------
        auto mkFloatDesc = [](const std::string & desc_str,
                              double from, double to)
        {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.description = desc_str;
            rcl_interfaces::msg::FloatingPointRange r;
            r.from_value = from;
            r.to_value   = to;
            r.step       = 0.0;
            d.floating_point_range.push_back(r);
            return d;
        };

        auto mkDesc = [](const std::string & desc_str)
        {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.description = desc_str;
            return d;
        };

        this->declare_parameter("input_topic",             "/hesai/pandar");
        this->declare_parameter("output_topic",            "/voxelized_cloud");
        this->declare_parameter("marker_topic",            "/voxel_markers");
        this->declare_parameter("global_frame_id",         "map");
        this->declare_parameter("pose_topic",              "/pcl_pose");
        this->declare_parameter("pose_type",               "PoseWithCovarianceStamped");
        this->declare_parameter("map_topic",               "/initial_map");
        this->declare_parameter("initial_map_file",        "");
        this->declare_parameter("initial_map_leaf_size",   0.0,
            mkFloatDesc("Downsample leaf size for optional .pcd map file (m). 0.0 disables downsampling.", 0.0, 5.0));
        this->declare_parameter("foreground_topic",        "/foreground_cloud");
        this->declare_parameter("foreground_marker_topic", "/foreground_markers");

        this->declare_parameter("subtraction_radius", 2,
            mkDesc("Chebyshev radius (in voxels) around each map voxel that is "
                   "considered static and removed from the foreground output. "
                   "1=3^3=27 neighbours, 2=5^3=125, 3=7^3=343.  Tunable live."));

        this->declare_parameter("leaf_size", 0.0,
            mkFloatDesc("Voxel downsampling edge length in metres (0.0 to disable)", 0.0, 5.0));

        this->declare_parameter("subtraction_leaf_size", 0.2,
            mkFloatDesc("Map subtraction voxel grid size in metres", 0.01, 5.0));

        this->declare_parameter("min_range", 0.5,
            mkFloatDesc("Discard points closer than this (metres); "
                        "applied in sensor frame before transform", 0.0, 10.0));

        this->declare_parameter("max_range", 100.0,
            mkFloatDesc("Discard points farther than this (metres); "
                        "applied in sensor frame before transform", 1.0, 500.0));

        this->declare_parameter("marker_alpha", 0.85,
            mkFloatDesc("Cube opacity (0 = transparent, 1 = opaque)", 0.0, 1.0));

        this->declare_parameter("tf_timeout",     0.05,
            mkFloatDesc("Seconds to wait for TF lookup before falling back to pose", 0.0, 1.0));
        this->declare_parameter("lidar_x_offset", 0.0,
            mkFloatDesc("Fallback static lidar→base_link offset X (m)", -10.0, 10.0));
        this->declare_parameter("lidar_y_offset", 0.0,
            mkFloatDesc("Fallback static lidar→base_link offset Y (m)", -10.0, 10.0));
        this->declare_parameter("lidar_z_offset", 0.0,
            mkFloatDesc("Fallback static lidar→base_link offset Z (m)", -10.0, 10.0));

        // ---- read initial values ----------------------------------------
        inputTopic_    = this->get_parameter("input_topic").as_string();
        outputTopic_   = this->get_parameter("output_topic").as_string();
        markerTopic_   = this->get_parameter("marker_topic").as_string();
        globalFrameId_ = this->get_parameter("global_frame_id").as_string();
        poseTopic_     = this->get_parameter("pose_topic").as_string();
        poseType_      = this->get_parameter("pose_type").as_string();
        mapTopic_      = this->get_parameter("map_topic").as_string();
        initialMapFile_= this->get_parameter("initial_map_file").as_string();
        initialMapLeafSize_ = static_cast<float>(this->get_parameter("initial_map_leaf_size").as_double());
        fgTopic_       = this->get_parameter("foreground_topic").as_string();
        fgMarkerTopic_ = this->get_parameter("foreground_marker_topic").as_string();

        subRadius_   = static_cast<int>(this->get_parameter("subtraction_radius").as_int());
        leafSize_    = static_cast<float>(this->get_parameter("leaf_size").as_double());
        subLeafSize_ = static_cast<float>(this->get_parameter("subtraction_leaf_size").as_double());
        minRange_    = static_cast<float>(this->get_parameter("min_range").as_double());
        maxRange_    = static_cast<float>(this->get_parameter("max_range").as_double());
        markerAlpha_ = static_cast<float>(this->get_parameter("marker_alpha").as_double());

        tfTimeout_   = this->get_parameter("tf_timeout").as_double();
        lidarXOff_   = static_cast<float>(this->get_parameter("lidar_x_offset").as_double());
        lidarYOff_   = static_cast<float>(this->get_parameter("lidar_y_offset").as_double());
        lidarZOff_   = static_cast<float>(this->get_parameter("lidar_z_offset").as_double());

        rebuildNeighborOffsets();

        // ---- TF2 listener -----------------------------------------------
        tfBuffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        // ---- live param callback ----------------------------------------
        paramCallbackHandle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                return onSetParameters(params);
            });

        // ---- QoS profiles -----------------------------------------------
        auto sensorQos    = rclcpp::SensorDataQoS();                  // BEST_EFFORT
        auto reliableQos  = rclcpp::QoS(10).reliable();
        auto transientQos = rclcpp::QoS(1).reliable().transient_local();

        // ---- publishers -------------------------------------------------
        pcPub_    = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        outputTopic_, reliableQos);
        mkPub_    = this->create_publisher<visualization_msgs::msg::Marker>(
                        markerTopic_, reliableQos);
        fgPcPub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        fgTopic_, reliableQos);
        fgMkPub_  = this->create_publisher<visualization_msgs::msg::Marker>(
                        fgMarkerTopic_, reliableQos);

        if (!initialMapFile_.empty()) {
            mapPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        mapTopic_, transientQos);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(initialMapFile_, *cloud) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", initialMapFile_.c_str());
            } else {
                // Populate the local map representation BEFORE downsampling
                rawMapCloud_ = cloud;
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*rawMapCloud_, *rawMapCloud_, indices);
                rebuildMapVoxels();

                if (initialMapLeafSize_ > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Downsampling initial map with leaf size %.2f", initialMapLeafSize_);
                    pcl::VoxelGrid<pcl::PointXYZ> grid;
                    grid.setLeafSize(initialMapLeafSize_, initialMapLeafSize_, initialMapLeafSize_);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
                    grid.setInputCloud(cloud);
                    grid.filter(*cloud_filtered);
                    cloud = cloud_filtered;
                }

                sensor_msgs::msg::PointCloud2 mapMsg;
                pcl::toROSMsg(*cloud, mapMsg);
                mapMsg.header.frame_id = globalFrameId_;
                mapMsg.header.stamp = this->get_clock()->now();
                mapPub_->publish(mapMsg);
                RCLCPP_INFO(this->get_logger(), "Published initial map from %s", initialMapFile_.c_str());
            }
        }

        // ---- subscribers ------------------------------------------------
        cloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            inputTopic_, sensorQos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                cloudCallback(msg);
            });

        if (poseType_ == "Odometry") {
            poseSubOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                poseTopic_, reliableQos,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    auto poseMsg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                    poseMsg->header = msg->header;
                    poseMsg->pose = msg->pose.pose;
                    latestPose_ = poseMsg;
                });
        } else if (poseType_ == "PoseStamped") {
            poseSubPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                poseTopic_, reliableQos,
                [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                    latestPose_ = msg;
                });
        } else {
            poseSubCov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                poseTopic_, reliableQos,
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
                    auto poseMsg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                    poseMsg->header = msg->header;
                    poseMsg->pose = msg->pose.pose;
                    latestPose_ = poseMsg;
                });
        }

        rclcpp::SubscriptionOptions subOptions;
        subOptions.ignore_local_publications = true;

        mapSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            mapTopic_, transientQos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                mapCallback(msg);
            }, subOptions);

        // ---- stats timer ------------------------------------------------
        statsTimer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { logStats(); });

        RCLCPP_INFO(this->get_logger(),
            "VoxelizerNode started\n"
            "  input              : %s\n"
            "  pose               : %s\n"
            "  map                : %s\n"
            "  voxel out          : %s\n"
            "  foreground         : %s\n"
            "  leaf               : %.3f m\n"
            "  sub leaf           : %.3f m\n"
            "  subtraction_radius : %d voxels (%zu offsets)\n"
            "  range              : [%.2f, %.2f] m\n"
            "  Waiting for map on \"%s\" and pose on \"%s\"...",
            inputTopic_.c_str(), poseTopic_.c_str(),
            mapTopic_.c_str(), outputTopic_.c_str(), fgTopic_.c_str(),
            leafSize_, subLeafSize_, subRadius_, neighborOffsets_.size(),
            minRange_, maxRange_,
            mapTopic_.c_str(), poseTopic_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    // Parameter callback
    // -----------------------------------------------------------------------

    rcl_interfaces::msg::SetParametersResult
    onSetParameters(const std::vector<rclcpp::Parameter> & params)
    {
        static const std::unordered_set<std::string> topicParams = {
            "input_topic", "output_topic", "marker_topic",
            "pose_topic", "pose_type", "map_topic", "initial_map_file", "initial_map_leaf_size", "foreground_topic", "foreground_marker_topic"
        };

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & p : params) {
            if (topicParams.count(p.get_name())) {
                result.successful = false;
                result.reason = "\"" + p.get_name() +
                                "\" cannot be changed at runtime — restart the node";
                return result;
            }

            if (p.get_name() == "subtraction_radius") {
                int val = static_cast<int>(p.as_int());
                if (val < 0) {
                    result.successful = false;
                    result.reason = "subtraction_radius must be >= 0";
                    return result;
                }
                subRadius_ = val;
                rebuildNeighborOffsets();
                RCLCPP_INFO(this->get_logger(),
                    "subtraction_radius -> %d (%zu offsets)",
                    subRadius_, neighborOffsets_.size());

            } else if (p.get_name() == "leaf_size") {
                if (p.as_double() < 0.0) {
                    result.successful = false;
                    result.reason = "leaf_size must be >= 0";
                    return result;
                }
                leafSize_ = static_cast<float>(p.as_double());
                RCLCPP_INFO(this->get_logger(), "leaf_size -> %.4f m", leafSize_);

            } else if (p.get_name() == "subtraction_leaf_size") {
                if (p.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason = "subtraction_leaf_size must be > 0";
                    return result;
                }
                subLeafSize_ = static_cast<float>(p.as_double());
                rebuildMapVoxels();
                publishDeleteAll();
                RCLCPP_INFO(this->get_logger(), "subtraction_leaf_size -> %.4f m", subLeafSize_);

            } else if (p.get_name() == "min_range") {
                if (p.as_double() < 0.0) {
                    result.successful = false;
                    result.reason = "min_range must be >= 0";
                    return result;
                }
                minRange_ = static_cast<float>(p.as_double());
                RCLCPP_INFO(this->get_logger(), "min_range -> %.3f m", minRange_);

            } else if (p.get_name() == "max_range") {
                if (p.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason = "max_range must be > 0";
                    return result;
                }
                maxRange_ = static_cast<float>(p.as_double());
                RCLCPP_INFO(this->get_logger(), "max_range -> %.3f m", maxRange_);

            } else if (p.get_name() == "marker_alpha") {
                markerAlpha_ = std::clamp(static_cast<float>(p.as_double()),
                                          0.0f, 1.0f);
                RCLCPP_INFO(this->get_logger(), "marker_alpha -> %.3f", markerAlpha_);
            }
        }

        return result;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Recompute the flat list of (2r+1)³ Chebyshev-neighbourhood offsets.
    void rebuildNeighborOffsets()
    {
        neighborOffsets_.clear();
        const int r = subRadius_;
        for (int dz = -r; dz <= r; ++dz)
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx)
                    neighborOffsets_.push_back({dx, dy, dz});
    }

    /// Re-index raw map points into a voxel set at current subLeafSize_.
    void rebuildMapVoxels()
    {
        if (!rawMapCloud_ || rawMapCloud_->empty()) {
            mapVoxelSet_.reset();
            return;
        }
        auto newSet = std::make_unique<VoxelSet>();
        newSet->reserve(rawMapCloud_->size());
        const float invLeaf = 1.0f / subLeafSize_;
        for (const auto & pt : *rawMapCloud_) {
            VoxelKey key{
                static_cast<int32_t>(std::floor(pt.x * invLeaf)),
                static_cast<int32_t>(std::floor(pt.y * invLeaf)),
                static_cast<int32_t>(std::floor(pt.z * invLeaf))
            };
            newSet->insert(key);
        }
        mapVoxelSet_ = std::move(newSet);
        RCLCPP_INFO(this->get_logger(),
            "Map voxel index rebuilt: %zu voxels at leaf=%.4f m",
            mapVoxelSet_->size(), subLeafSize_);
    }

    /// Send DELETEALL to both marker channels to wipe stale cubes in RViz2.
    void publishDeleteAll()
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->get_clock()->now();
        m.ns = "voxels";
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        mkPub_->publish(m);
        m.ns = "fg_voxels";
        fgMkPub_->publish(m);
    }

    // -----------------------------------------------------------------------
    // Map subtraction
    // -----------------------------------------------------------------------

    /// Returns a keep-mask (true = foreground, NOT near any map voxel).
    std::vector<bool> subtractMap(const std::vector<VoxelKey> & keys)
    {
        const size_t n = keys.size();
        std::vector<bool> keep(n, true);

        if (!mapVoxelSet_ || mapVoxelSet_->empty()) {
            return keep;  // no map → keep everything
        }

        const VoxelSet & ms = *mapVoxelSet_;

        for (size_t i = 0; i < n; ++i) {
            const auto & k = keys[i];
            for (const auto & off : neighborOffsets_) {
                VoxelKey nb{k.x + off[0], k.y + off[1], k.z + off[2]};
                if (ms.count(nb)) {
                    keep[i] = false;
                    break;  // early exit per-point
                }
            }
        }

        return keep;
    }

    // -----------------------------------------------------------------------
    // Incoming map callback
    // -----------------------------------------------------------------------

    void mapCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        rawMapCloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *rawMapCloud_);
        // Remove NaN/Inf points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*rawMapCloud_, *rawMapCloud_, indices);
        rebuildMapVoxels();
    }

    // -----------------------------------------------------------------------
    // Cloud callback — main pipeline
    // -----------------------------------------------------------------------

    void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        try {
            // ---- 1. Parse PointCloud2 into PCL cloud --------------------
            pcl::PointCloud<pcl::PointXYZI>::Ptr sensorCloud(
                new pcl::PointCloud<pcl::PointXYZI>);
            
            bool has_intensity = false;
            for (const auto & field : msg->fields) {
                if (field.name == "intensity") {
                    has_intensity = true;
                    break;
                }
            }

            if (has_intensity) {
                pcl::fromROSMsg(*msg, *sensorCloud);
            } else {
                pcl::PointCloud<pcl::PointXYZ> tempCloud;
                pcl::fromROSMsg(*msg, tempCloud);
                sensorCloud->reserve(tempCloud.size());
                for (const auto & pt : tempCloud) {
                    pcl::PointXYZI pt_i;
                    pt_i.x = pt.x;
                    pt_i.y = pt.y;
                    pt_i.z = pt.z;
                    pt_i.intensity = 0.0f;
                    sensorCloud->push_back(pt_i);
                }
            }

            const size_t nIn = sensorCloud->size();
            if (nIn == 0) return;

            // ---- 2. Range filter in sensor frame (radial) ---------------
            pcl::PointCloud<pcl::PointXYZI>::Ptr rangeFiltered(
                new pcl::PointCloud<pcl::PointXYZI>);
            rangeFiltered->reserve(nIn);

            const float minR2 = minRange_ * minRange_;
            const float maxR2 = maxRange_ * maxRange_;

            for (const auto & pt : *sensorCloud) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
                    !std::isfinite(pt.z))
                    continue;
                float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
                if (r2 >= minR2 && r2 <= maxR2)
                    rangeFiltered->push_back(pt);
            }

            if (rangeFiltered->empty()) return;

            // ---- 3. Transform to map frame ----------------------------------------
            pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud(
                new pcl::PointCloud<pcl::PointXYZI>);

            std_msgs::msg::Header outHeader = msg->header;
            outHeader.frame_id = globalFrameId_;
            outHeader.stamp    = msg->header.stamp;

            bool tfSuccess = false;

            // PRIMARY: pose (World→base_link) from /robotPose + TF for sensor offset.
            // Using TimePointZero for the sensor offset avoids timestamp sync issues
            // with Isaac Sim sim-time, while the Odometry gives accurate World position.
            if (latestPose_ && latestPose_->header.frame_id == globalFrameId_) {
                const auto & pos = latestPose_->pose.position;
                const auto & q   = latestPose_->pose.orientation;

                double qn = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
                double qx = q.x/qn, qy = q.y/qn, qz = q.z/qn, qw = q.w/qn;

                Eigen::Affine3f T_pose = Eigen::Affine3f::Identity();
                T_pose.linear() << static_cast<float>(1 - 2*(qy*qy + qz*qz)),
                                   static_cast<float>(    2*(qx*qy - qz*qw)),
                                   static_cast<float>(    2*(qx*qz + qy*qw)),
                                   static_cast<float>(    2*(qx*qy + qz*qw)),
                                   static_cast<float>(1 - 2*(qx*qx + qz*qz)),
                                   static_cast<float>(    2*(qy*qz - qx*qw)),
                                   static_cast<float>(    2*(qx*qz - qy*qw)),
                                   static_cast<float>(    2*(qy*qz + qx*qw)),
                                   static_cast<float>(1 - 2*(qx*qx + qy*qy));
                T_pose.translation() <<
                    static_cast<float>(pos.x),
                    static_cast<float>(pos.y),
                    static_cast<float>(pos.z);

                // Look up base_link → sensor_frame at latest available time (static mount).
                Eigen::Affine3f T_offset = Eigen::Affine3f::Identity();
                try {
                    auto sensorTs = tfBuffer_->lookupTransform(
                        "base_link",
                        msg->header.frame_id,
                        tf2::TimePointZero);
                    T_offset = tf2::transformToEigen(sensorTs).cast<float>();
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "base_link→%s TF unavailable (%s); using configured offsets.",
                        msg->header.frame_id.c_str(), ex.what());
                    T_offset.translation() << lidarXOff_, lidarYOff_, lidarZOff_;
                }

                pcl::transformPointCloud(*rangeFiltered, *mapCloud, T_pose * T_offset);
                tfSuccess = true;
            }

            // SECONDARY: full TF chain (used when no pose topic is available).
            if (!tfSuccess) {
                try {
                    auto ts = tfBuffer_->lookupTransform(
                        globalFrameId_,
                        msg->header.frame_id,
                        msg->header.stamp,
                        rclcpp::Duration::from_seconds(tfTimeout_));

                    Eigen::Affine3f Tf = tf2::transformToEigen(ts).cast<float>();
                    pcl::transformPointCloud(*rangeFiltered, *mapCloud, Tf);
                    tfSuccess = true;

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Full TF chain failed (%s). Publishing in sensor frame.", ex.what());
                }
            }

            // LAST RESORT: no transform available
            if (!tfSuccess) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "No pose and no TF available. Publishing in sensor frame.");
                mapCloud = rangeFiltered;
                outHeader.frame_id = msg->header.frame_id;
            }

            // ---- 4. Voxel downsample (PCL VoxelGrid) --------------------
            pcl::PointCloud<pcl::PointXYZI>::Ptr voxelised(
                new pcl::PointCloud<pcl::PointXYZI>);
            if (leafSize_ > 0.0f) {
                pcl::VoxelGrid<pcl::PointXYZI> vg;
                vg.setInputCloud(mapCloud);
                vg.setLeafSize(leafSize_, leafSize_, leafSize_);
                vg.filter(*voxelised);
            } else {
                voxelised = mapCloud;
            }

            const size_t nOut = voxelised->size();
            if (nOut == 0) return;

            // ---- 5. Snap to voxel centres + collect keys ----------------
            // VoxelGrid gives centroids; snap to grid node for gap-free CUBE_LIST.
            std::vector<VoxelKey>         keys(nOut);
            std::vector<std::array<float,3>> centers(nOut);
            const float invLeaf = 1.0f / subLeafSize_;

            for (size_t i = 0; i < nOut; ++i) {
                auto & pt = (*voxelised)[i];
                VoxelKey k{
                    static_cast<int32_t>(std::floor(pt.x * invLeaf)),
                    static_cast<int32_t>(std::floor(pt.y * invLeaf)),
                    static_cast<int32_t>(std::floor(pt.z * invLeaf))
                };
                keys[i] = k;
                float cx = (static_cast<float>(k.x) + 0.5f) * subLeafSize_;
                float cy = (static_cast<float>(k.y) + 0.5f) * subLeafSize_;
                float cz = (static_cast<float>(k.z) + 0.5f) * subLeafSize_;
                centers[i] = {cx, cy, cz};
                // Move the point to the snapped centre so the cloud matches markers
                pt.x = cx; pt.y = cy; pt.z = cz;
            }

            // ---- 6. Map subtraction -------------------------------------
            std::vector<bool> keepMask = subtractMap(keys);

            size_t nFg = 0;
            for (bool b : keepMask) nFg += b ? 1 : 0;

            // ---- 7a. Publish full voxelised cloud -----------------------
            {
                sensor_msgs::msg::PointCloud2 outMsg;
                pcl::toROSMsg(*voxelised, outMsg);
                outMsg.header = outHeader;
                pcPub_->publish(outMsg);
            }

            // ---- 7b. Publish full CUBE_LIST marker ----------------------
            publishMarker(mkPub_, outHeader, centers, "voxels");

            // ---- 7c. Foreground cloud & marker --------------------------
            if (nFg > 0) {
                pcl::PointCloud<pcl::PointXYZI> fgCloud;
                fgCloud.reserve(nFg);
                std::vector<std::array<float,3>> fgCenters;
                fgCenters.reserve(nFg);

                for (size_t i = 0; i < nOut; ++i) {
                    if (keepMask[i]) {
                        fgCloud.push_back((*voxelised)[i]);
                        fgCenters.push_back(centers[i]);
                    }
                }

                sensor_msgs::msg::PointCloud2 fgMsg;
                pcl::toROSMsg(fgCloud, fgMsg);
                fgMsg.header = outHeader;
                fgPcPub_->publish(fgMsg);
                publishMarker(fgMkPub_, outHeader, fgCenters, "fg_voxels");
            } else {
                // Publish empty cloud so downstream nodes don't stall
                pcl::PointCloud<pcl::PointXYZI> empty;
                sensor_msgs::msg::PointCloud2 emptyMsg;
                pcl::toROSMsg(empty, emptyMsg);
                emptyMsg.header = outHeader;
                fgPcPub_->publish(emptyMsg);
            }

            // ---- stats --------------------------------------------------
            ++msgCount_;
            totalIn_  += static_cast<uint64_t>(nIn);
            totalOut_ += static_cast<uint64_t>(nOut);

        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                *this->get_clock(), 2000,
                "Failed to process cloud: %s", e.what());
        }
    }

    // -----------------------------------------------------------------------
    // Marker helper
    // -----------------------------------------------------------------------

    void publishMarker(
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        const std_msgs::msg::Header & header,
        const std::vector<std::array<float,3>> & centers,
        const std::string & ns)
    {
        visualization_msgs::msg::Marker marker;
        marker.header   = header;
        marker.ns       = ns;
        marker.id       = 0;
        marker.type     = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action   = visualization_msgs::msg::Marker::ADD;
        marker.scale.x  = subLeafSize_;
        marker.scale.y  = subLeafSize_;
        marker.scale.z  = subLeafSize_;
        marker.pose.orientation.w = 1.0;
        marker.lifetime.sec     = 0;
        marker.lifetime.nanosec = 0;

        const size_t n = centers.size();

        // Build height colour map
        std::vector<float> zVals(n);
        for (size_t i = 0; i < n; ++i) zVals[i] = centers[i][2];
        std::vector<float> rArr, gArr, bArr;
        heightColormap(zVals, rArr, gArr, bArr);

        marker.points.resize(n);
        marker.colors.resize(n);
        for (size_t i = 0; i < n; ++i) {
            marker.points[i].x = centers[i][0];
            marker.points[i].y = centers[i][1];
            marker.points[i].z = centers[i][2];
            marker.colors[i].r = rArr[i];
            marker.colors[i].g = gArr[i];
            marker.colors[i].b = bArr[i];
            marker.colors[i].a = markerAlpha_;
        }

        pub->publish(marker);
    }

    // -----------------------------------------------------------------------
    // Stats timer (every 5 s)
    // -----------------------------------------------------------------------

    void logStats()
    {
        if (!latestPose_)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for pose on \"%s\" …", poseTopic_.c_str());

        if (!mapVoxelSet_)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for map on \"%s\" …", mapTopic_.c_str());

        if (msgCount_ == 0) {
            if (latestPose_ && mapVoxelSet_)
                RCLCPP_INFO(this->get_logger(),
                    "No scan messages received yet — waiting for input.");
            return;
        }

        double avgIn  = static_cast<double>(totalIn_)  / msgCount_;
        double avgOut = static_cast<double>(totalOut_) / msgCount_;
        double ratio  = (avgIn > 0.0) ? (avgOut / avgIn * 100.0) : 0.0;
        size_t mapN   = mapVoxelSet_ ? mapVoxelSet_->size() : 0;

        RCLCPP_INFO(this->get_logger(),
            "[%u msgs]  in=%.0f pts  voxels=%.0f (%.1f%%)  "
            "map_voxels=%zu  leaf=%.3f m  sub_leaf=%.3f m",
            msgCount_, avgIn, avgOut, ratio, mapN, leafSize_, subLeafSize_);

        msgCount_ = 0;
        totalIn_  = 0;
        totalOut_ = 0;
    }

    // -----------------------------------------------------------------------
    // Member variables
    // -----------------------------------------------------------------------

    // Topic names (set once at startup)
    std::string inputTopic_, outputTopic_, markerTopic_, globalFrameId_;
    std::string poseTopic_, poseType_, mapTopic_, initialMapFile_, fgTopic_, fgMarkerTopic_;
    float initialMapLeafSize_ = 0.0f;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>            tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    double tfTimeout_  = 0.05;
    float  lidarXOff_  = 0.0f;
    float  lidarYOff_  = 0.0f;
    float  lidarZOff_  = 0.0f;

    // Live-tunable parameters
    float leafSize_    = 0.3f;
    float subLeafSize_ = 0.2f;
    float minRange_    = 0.5f;
    float maxRange_    = 100.0f;
    float markerAlpha_ = 0.85f;
    int   subRadius_   = 2;

    // Precomputed (2r+1)³ neighbor offsets
    std::vector<std::array<int32_t,3>> neighborOffsets_;

    // State
    geometry_msgs::msg::PoseStamped::ConstSharedPtr latestPose_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rawMapCloud_;
    std::unique_ptr<VoxelSet> mapVoxelSet_;

    // Pub/sub handles
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pcPub_, fgPcPub_, mapPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  mkPub_, fgMkPub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_, mapSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr poseSubOdom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSubPose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseSubCov_;
    rclcpp::TimerBase::SharedPtr statsTimer_;

    // Parameter callback handle (must stay alive)
    OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;

    // Stats counters
    uint32_t msgCount_ = 0;
    uint64_t totalIn_  = 0;
    uint64_t totalOut_ = 0;
};

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelizerNode>());
    rclcpp::shutdown();
    return 0;
}
