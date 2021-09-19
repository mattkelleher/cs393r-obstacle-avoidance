//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    length_(0.4), //TODO
    width_(0.2),  //TODO
    wheelbase_(0.35), //TODO
    trackbase_(0.17), //TODO
    smargin_(0.1),
    freePathLength_(0),	
    nav_complete_(true),
    nav_goal_loc_(7, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

float Navigation::_Distance(Vector2f p1, Vector2f p2) {
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

float Navigation::_EvalPath(float r) {
    float rin = 0;
    float rout = 0;
    float rmid = 0;;
    Vector2f centerOfTurning(0, r);
    Vector2f frontLeft((length_ + wheelbase_) / 2 + smargin_, width_ / 2 + smargin_);
    Vector2f frontRight((length_ + wheelbase_) / 2 + smargin_, -1 * width_ / 2 - smargin_);

    if(r == 0) { //special case, car moving forward will always hit front (+- width/2 + smargin)
      //TODO      
    }
    if(r > 0) {  //turning left, front left corner is middle radius, front right corner is outer radius
      rin = _Distance(centerOfTurning, Vector2f(0, width_ / 2 + smargin_));
      rout = _Distance(centerOfTurning, frontRight);
      rmid = _Distance(centerOfTurning, frontLeft);
  
    }
    else if(r < 0){ //turning right, front left corner is outer radius, front right corner is middle radius 
      rin = _Distance(centerOfTurning, Vector2f(0, -1 * width_ / 2 - smargin_));
      rout = _Distance(centerOfTurning, frontLeft);
      rmid = _Distance(centerOfTurning, frontRight);
    }
    
    //Check if car will collide with point, if so where?
    float rpoint;
    bool hits = false;
    float bestTheta = 360; //angle in degrees we move along this path 
    Vector2f pointOfImpact(0,0);
    for(auto point: point_cloud_) {
      hits = false;
      if(point.x() >= 0) { //we are not worried about points behind us //TODO this MAY not be true for future assignments
	rpoint = _Distance(centerOfTurning, point);
        if(rpoint >= rmid && rpoint <= rout) { // front of car will hit point
          hits = true;
	  pointOfImpact.x() = (length_ + wheelbase_) / 2 + smargin_;
          pointOfImpact.y() = sqrt(pow(rpoint, 2) - pow(pointOfImpact.x(), 2)) + centerOfTurning.y(); 
	}
        else if(rpoint >= rin && rpoint < rmid) { // side of car will hit point
          hits = true;
	  if(r > 0) {
	    pointOfImpact.y() = width_ / 2 + smargin_;
	  }
	  else if(r < 0) {
            pointOfImpact.y() = -1 * width_ / 2 - smargin_;
	  }
	  pointOfImpact.x() = sqrt(pow(rpoint, 2) - pow((pointOfImpact.y() - centerOfTurning.y()) , 2));
        }
        if(hits) {
         // Calculate theta, if theta < curr best theta
	 float theta = 0;
	 if(theta < bestTheta) {
           bestTheta = theta;
	 }
	 //freePathLength = _Distance(Vector2f(0,0), endPoint); //TODO calculate endpoint 
        }
      }
    }

    //TODO is max path length better or should we stop early at point closest to goal on arc?
  return 1; //TODO return _Score(xx)
}

int Navigation::_Score(int freePathLength, Vector2f endPoint) { //TODO re write
  // TODO potentially add hyperparameter to navigation class to weight score
  return _Distance(Vector2f(0,0), endPoint) - _Distance(endPoint, nav_goal_loc_);
  //return freePathLength - sqrt(pow(endPoint.x() - nav_goal_loc_.x(), 2)
  //	       + pow(endPoint.y() - nav_goal_loc_.y(), 2)); 
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  visualization::DrawCross(nav_goal_loc_, 0.5, 0x1644db, local_viz_msg_);
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  float bestScore = 0;
  float bestCurv = 0;
  float score;
  for(float curv = 45; curv >= -45; curv -= 5) {
    if(curv == 0) {
      score = _EvalPath(0);
    }
    else{
      float r = wheelbase_ / tan(M_PI / 180 * curv);
      score = _EvalPath(r);
    }
    if(score > bestScore){
      bestScore = score;
      bestCurv = curv;
    }
  }
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = bestCurv;
  
  //TODO Check to see if we have enough space or if we need to break
  drive_msg_.velocity = 1;
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
