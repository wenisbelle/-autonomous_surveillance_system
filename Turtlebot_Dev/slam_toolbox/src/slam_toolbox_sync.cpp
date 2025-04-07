/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 * Copyright Work Modifications (c) 2024, Daniel I. Meza
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_sync.hpp"

#include <memory>
namespace slam_toolbox
{

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  ssClear_ = this->create_service<slam_toolbox::srv::ClearQueue>("slam_toolbox/clear_queue",
      std::bind(&SynchronousSlamToolbox::clearQueueCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  threads_.push_back(std::make_unique<boost::thread>(
      boost::bind(&SynchronousSlamToolbox::run, this)));
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  rclcpp::Rate r(100);
  while (rclcpp::ok()) {
    if (!isPaused(PROCESSING)) {
      PosedScan scan_w_pose(nullptr, karto::Pose2()); // dummy, updated in critical section
      bool queue_empty = true;
      {
        boost::mutex::scoped_lock lock(q_mutex_);
        queue_empty = q_.empty();
        if (!queue_empty) {
          scan_w_pose = q_.front();
          q_.pop();

          if (q_.size() > 10) {
            RCLCPP_WARN(get_logger(), "Queue size has grown to: %i. "
              "Recommend stopping until message is gone if online mapping.",
              (int)q_.size());
          }
        }
      }
      if (!queue_empty) {
        addScan(getLaser(scan_w_pose.scan), scan_w_pose);
        continue;
      }
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const std::string & base_frame_id)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  
  // no odom info on any pose helper
  Pose2 pose;
  if (!pose_helpers_[base_frame_id]->getOdomPose(pose, scan->header.stamp, base_frame_id)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose for %s", base_frame_id.c_str());
    return;
  }

  // Add new sensor to laser ID map and create its laser assistant
  { // ensure mutex is released
    boost::mutex::scoped_lock l(laser_id_map_mutex_);
    if (m_laser_id_to_base_id_.find(scan->header.frame_id) == m_laser_id_to_base_id_.end()) {
      m_laser_id_to_base_id_[scan->header.frame_id] = base_frame_id;
      laser_assistants_[scan->header.frame_id] = std::make_unique<laser_utils::LaserAssistant>(shared_from_this(), tf_.get(), base_frame_id);
    }
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // if sync and valid, add to queue
  if (shouldProcessScan(scan, pose)) {
    boost::mutex::scoped_lock lock(q_mutex_);
    q_.push(PosedScan(scan, pose));
  }
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::ClearQueue::Request> req,
  std::shared_ptr<slam_toolbox::srv::ClearQueue::Response> resp)
/*****************************************************************************/
{
  RCLCPP_INFO(get_logger(), "Clearing all queued scans to add to map.");
  while (!q_.empty()) {
    q_.pop();
  }
  resp->status = true;
  return resp->status;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::SynchronousSlamToolbox)
