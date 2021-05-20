#ifndef GLOBAL_PLANNER_PLUS_GLOBAL_POTENTIALS_INTERFACE_H
#define GLOBAL_PLANNER_PLUS_GLOBAL_POTENTIALS_INTERFACE_H


#include <global_planner_plus/planner_core.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <Eigen/Core>

namespace global_planner_plus
{
  
  geometry_msgs::Pose transformPose(const tf::StampedTransform& transform, const geometry_msgs::Pose& pose_msg);
  
  //   geometry_msgs::Pose transformPose(const tf::StampedTransform& transform, const PoseSE2& pose);
  
  void transformGradient(const tf::StampedTransform& transform, const Eigen::Vector2d& grad, Eigen::Vector2d& t);
  
  class Costmap2DPotentials : public costmap_2d::Costmap2D
  {
  public:
    bool update(const global_planner_plus::PotentialGrid::ConstPtr& potentials);
    float getPointPotential(const geometry_msgs::Point& world_point) const;
    bool getGradient(const geometry_msgs::Point& world_point, Eigen::Vector2d& grad) const;
    
    bool mapToGrid(double wx, double wy, double& mx, double& my) const;
    Eigen::Vector2d gridToMap(const Eigen::Vector2d& grid_pos) const;
    
  private:
    float gradCell(unsigned int n);
    bool updateGradients();
    
    bool interpolateValue(const std::vector<float>& values, const geometry_msgs::Point& world_point, float& result) const;
    
  protected:
    std::string name_ = "costmap_2d_potentials";
    
  private:
    global_planner_plus::PotentialGrid::ConstPtr potentials_;
    std::vector<float> gradx_, grady_;
    unsigned int lethal_cost_;
    float convert_offset_;
  };
  
  class GoalFunc;
  
  class GlobalPotentialsInterface
  {
    friend class GoalFunc;
  public:
    //TODO: Pass in transform listener and costmap
    GlobalPotentialsInterface(ros::NodeHandle nh=ros::NodeHandle());
    virtual void init(std::string potentials_topic);
    //bool updatePotentials(const PoseSE2& start, const PoseSE2& goal);
    virtual bool updateTransform(const tf::StampedTransform& transform);
    virtual float getPotential(const geometry_msgs::Pose& pose) const;
    virtual bool getGradient(const geometry_msgs::Pose& pose, Eigen::Vector2d& grad) const;
    virtual bool isInited() const;
    virtual bool worldToGrid(const Eigen::Vector2d& world, Eigen::Vector2d& grid) const;
    virtual std_msgs::Header getHeader() const;
    
  protected:
    virtual void potentialsCB(const global_planner_plus::PotentialGrid::ConstPtr& potentials_msg);

    ros::NodeHandle nh_;
    //global_planner_plus::GlobalPlannerPlus planner_;
    message_filters::Subscriber<global_planner_plus::PotentialGrid> potentials_sub_;
    message_filters::Cache<global_planner_plus::PotentialGrid> potentials_cache_;
    
  protected:
    Costmap2DPotentials costmap_;
    std::string name_ = "global_potentials_interface";
    //tf::TransformListener* tf_;
    tf::StampedTransform global_to_map_transform_, map_to_global_transform_;
    global_planner_plus::PotentialGrid::ConstPtr current_potentials_msg_;
    //costmap_2d::Costmap2DROS lcr_;
    //std::string planning_frame_id_;
    bool inited_;
  };
  
  
  
} //end namespace global_planner_plus

#endif //GLOBAL_PLANNER_PLUS_GLOBAL_POTENTIALS_INTERFACE_H

