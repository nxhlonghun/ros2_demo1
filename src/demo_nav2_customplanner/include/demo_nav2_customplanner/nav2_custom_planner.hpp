#ifndef NAV2_CUSTOM_PLANNER_HPP
#define NAV2_CUSTOM_PLANNER_HPP
#include <memory>
#include <string>
// 消息接口
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp"

#include "rclcpp/rclcpp.hpp"

#include "nav2_core/global_planner.hpp" //基类
#include "nav2_util/lifecycle_node.hpp" //rclcpp的自类
#include "nav2_util/robot_utils.hpp"    //常用工具

namespace nav2_custom_planner
{
    class CustomPlanner : public nav2_core::GlobalPlanner
    {
    public:
        CustomPlanner() = default;
        ~CustomPlanner() = default;
        // 插件配置方法
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        // 插件清理方法
        void cleanup() override;
        // 插件激活方法
        void activate() override;
        // 插件停用方法
        void deactivate() override;
        // 为给定的起点和终点位姿创建路径的方法
        nav_msgs::msg::Path
        createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        // 坐标变换缓存指针，可用于查询坐标关系
        std::shared_ptr<tf2_ros::Buffer> tf_Buffer_;
        // 节点指针
        nav2_util::LifecycleNode::SharedPtr node_;
        // 全局代价地图
        nav2_costmap_2d::Costmap2D *costmap_;
        // 全局代价地图坐标系
        std::string global_frame_, name_;
        // 插值分辨率
        double interpolation_resolution_;
    };
} // namespace nav2_custom_planner

#endif