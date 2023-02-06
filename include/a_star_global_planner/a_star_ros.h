#ifndef ASTAR_GLOBAL_PLANNER_ROS_
#define ASTAR_GLOBAL_PLANNER_ROS_

#include <ros/ros.h>

#include "a_star.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <nav_core/base_global_planner.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


using std::string;

namespace astar_global_planner
{
    /*Astar 算法与外接接口*/
    class AstarGlobalPlannerRos : public nav_core::BaseGlobalPlanner{
        public:
        /*Default constructor for the Astar Planner Ros object*/
        AstarGlobalPlannerRos();
        /*constructor for Astar Planner Ros object*/
        AstarGlobalPlannerRos(std::string name, costmap_2d::Costmap2DROS* costmap_2d);
        /*constructor for Astar Planner Ros object*/
        AstarGlobalPlannerRos(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        /*Initialization function for the A star Planner object*/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
        /*Given a goal pose in the world, compute a plan*/
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);        
        
        private:
        /*函数*/
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        bool worldToMap(double wx, double wy, int& mx, int& my);
        void mapToWorld(double mx, double my, double& wx, double& wy);
        

        /*变量*/
        costmap_2d::Costmap2D* costmap_;

        // 创建一个路径规划的指针，用于调用路径规划的函数与变量
        boost::shared_ptr<AstarGlobalPlanner> planner_;
        
        ros::Publisher plan_pub_;
        ros::Publisher potarr_pub_;

        ros::ServiceServer make_plan_srv;

        std::string frame_id_;;

        boost::mutex mutex_;
        //需要从参数服务器上加载的，在launch文件中配置相应的yaml文件上传到参数服务器
        bool initialized_;  // 初始化的标志
        bool visualize_potential_; // 是否通过PointCloud2消息在rviz中可视化潜在的区域
        bool allow_unknown_; // 是否允许NavFn规划的路径通过未知区域，默认是true
        bool outline_map_;
        double planner_window_x_, planner_window_y_; // 规划窗口的大小，默认是0。(按:从源码中看，好像没用到)
        double default_tolerance_; //到达终点的容忍度。当规划的路径点到目标点的距离小于该参数描述的值的时候，就认为路径点基本到达了目标点
        double conver_offset;
    };
};

#endif