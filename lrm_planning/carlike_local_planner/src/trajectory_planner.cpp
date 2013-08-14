/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner.h>

using namespace std;
using namespace costmap_2d;

namespace carlike_local_planner{
  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model, 
      const Costmap2D& costmap, 
      std::vector<geometry_msgs::Point> footprint_spec,
      double inscribed_radius, double circumscribed_radius,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      double sim_time, double sim_granularity, 
      int vx_samples, int vtheta_samples,
      double pdist_scale, double gdist_scale, double occdist_scale,
      double heading_lookahead, double oscillation_reset_dist,
      double escape_reset_dist, double escape_reset_theta,
      bool holonomic_robot,
      double max_vel_x, double min_vel_x,
      double max_vel_th, double min_vel_th, double min_in_place_vel_th,
      double backup_vel,
      bool dwa, bool heading_scoring, double heading_scoring_timestep, bool simple_attractor,
      vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
    : map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), costmap_(costmap), 
    world_model_(world_model), footprint_spec_(footprint_spec),
    inscribed_radius_(inscribed_radius), circumscribed_radius_(circumscribed_radius),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead), 
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist), 
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x), 
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
  {
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;

    last_ackerman_theta_=0.0;
    limit_delta_theta_ = 0.15;
    acerman_theta_limit_=0.79;
    vel_samples_=2;
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    MapCell cell = map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.path_dist >= map_.map_.size() || cell.goal_dist >= map_.map_.size() || occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.path_dist;
    goal_cost = cell.goal_dist;
    total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  //create and score a trajectory given the current pose of the robot and selected velocities
  void TrajectoryPlanner::generateTrajectoryCarLike(double x, double y, double theta, double goal_th, double acerman_theta_samp, double vel_samp, double impossible_cost, Trajectory& traj){
    double x_i = x;
    double y_i = y;
    double theta_i = theta;


    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    num_steps = int(sim_time_ / sim_granularity_ + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    num_steps = 20;

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = 0; 
    traj.yv_ = 0; 
    traj.thetav_ = acerman_theta_samp;
    traj.lin_vel_ = vel_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;
    double ang_diff;
    double angdist_scale = 10; // it need removing to properties
    double ang_cost;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      double cell_pdist = map_(cell_x, cell_y).path_dist;
      double cell_gdist = map_(cell_x, cell_y).goal_dist;
      //ROS_INFO("Costs cell_x=%d cell_y=%d: occ_cost=%f footprint_cost=%f cell_cost=%f cell_pdist=%f cell_gdist=%f",cell_x, cell_y,occ_cost,footprint_cost,double(costmap_.getCost(cell_x, cell_y)), cell_pdist, cell_gdist);
      //update path and goal distances
      if(!heading_scoring_){
        path_dist = cell_pdist;
        goal_dist = cell_gdist;
      }
      else if(time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt){
        heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
        //update path and goal distances
        path_dist = cell_pdist;
        goal_dist = cell_gdist;
      }

      //do we want to follow blindly
      if(simple_attractor_){
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) * 
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) + 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) * 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
        path_dist = 0.0;
      }
      else{
        //if a point on this trajectory has no clear path to goal it is invalid
        if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
          ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f", 
              goal_dist, path_dist, impossible_cost);
          traj.cost_ = -2.0;
          return;
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate new positions

      double wsl = 0.45; //wheels base
      double linear_velocity = vel_samp;

      double r = wsl * tan(3.141592654/2 - acerman_theta_samp); //radius
      double lin_distance = linear_velocity * dt;
      double dtheta_i = lin_distance/r;

      theta_i += dtheta_i;
      x_i += lin_distance*cos(theta_i);
      y_i += lin_distance*sin(theta_i);

      ang_diff = abs(angles::shortest_angular_distance(theta_i, goal_th));
      angdist_scale = 100; // it need removing to properties
      ang_cost = ang_diff*angdist_scale/(goal_dist+1);

      //break from cycle if point is goal
      if (goal_dist>=0 && goal_dist<5){
    	  ROS_INFO("Break from cycle");
    	  break;
      }


      //ROS_INFO("addPoint x_i=%f y_i=%f theta_i=%f r=%f lin_dstance=%f",x_i, y_i, theta_i, r, lin_distance);
    }

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if(!heading_scoring_){
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
      ROS_DEBUG("!heading_scoring_");
    }
    else{
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    }
    double vel_cost; //penalty for driving backwards
    vel_cost = 0;
    if (vel_samp<0){
    	vel_cost = 0.2*cost;}
    ROS_INFO("path_dist = %f goal_dist = %f occ_cost = %f vel_cost=%f ang_cost=%f",path_dist,goal_dist,occ_cost, vel_cost,ang_cost);
    traj.cost_ = cost+vel_cost+ang_cost;
  }


  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    //find a clear line of sight from the robot's cell to a point on the path
    for(int i = global_plan_.size() - 1; i >=0; --i){
      if(costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)){
        if(lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0){
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;
          double v2_x = cos(heading);
          double v2_y = sin(heading);

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;

        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1, 
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      point_cost = pointCost(x, y); //Score the current point

      if(point_cost < 0)
        return -1;

      if(line_cost < point_cost)
        line_cost = point_cost;

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if(compute_dists){
      //reset the map for new operations
      map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      map_.setPathCells(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double acerman_theta_samp, double lin_vel){
    Trajectory t; 

    double cost = scoreTrajectory(x, y, theta, acerman_theta_samp, lin_vel);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0)
      return true;

    //otherwise the check fails
    return false;
  }

  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double  acerman_theta_samp, double lin_vel){
    Trajectory t; 
    double impossible_cost = map_.map_.size();
    generateTrajectoryCarLike(x, y, theta, 0, acerman_theta_samp, lin_vel, impossible_cost, t);

    // return the cost.
    return double( t.cost_ );
  }



  //create the trajectories we wish to score
  Trajectory TrajectoryPlanner::createTrajectoriesCarLike(double x, double y, double theta,  double goal_th){
    
    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

    Trajectory* swap = NULL;

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = map_.map_.size();
    
    double dacer_theta = 2*limit_delta_theta_/vtheta_samples_;
    double vel_sample = -0.1;
    for(int j=0; j<vel_samples_; j++){
		double acerman_theta_samp = (last_ackerman_theta_-limit_delta_theta_ < -acerman_theta_limit_) ? -acerman_theta_limit_:last_ackerman_theta_-limit_delta_theta_;
		for(int i = 0; i < vtheta_samples_; ++i){
			if (acerman_theta_samp>acerman_theta_limit_)
				break;
			generateTrajectoryCarLike(x, y, theta, goal_th, acerman_theta_samp, vel_sample, impossible_cost, *comp_traj);
			//ROS_INFO("Trajectory n=%d acerman_theta_samp=%f cost=%f",i,acerman_theta_samp,comp_traj->cost_);
			//if the new trajectory is better... let's take it
			if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
				ROS_INFO("Found better trajectory cost=%f", comp_traj->cost_);
			  swap = best_traj;
			  best_traj = comp_traj;
			  comp_traj = swap;
			}

			acerman_theta_samp += dacer_theta;
		  }
		vel_sample+=0.2;
    }
    return *best_traj;
  }
  
  
  //given the current state of the robot, find a good trajectory
  Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, double goal_th,
      double& accerman_theta, double& lin_vel){

    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(global_vel.getRotation());

    double x = global_pose.getOrigin().getX();
    double y = global_pose.getOrigin().getY();
    double theta = yaw;

    double vx = global_vel.getOrigin().getX();
    double vy = global_vel.getOrigin().getY();
    double vtheta = vel_yaw;

    //reset the map for new operations
    map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    vector<carlike_local_planner::Position2DInt> footprint_list = getFootprintCells(x, y, theta, true);

    //mark cells within the initial footprint of the robot
    for(unsigned int i = 0; i < footprint_list.size(); ++i){
      map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    map_.setPathCells(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectoriesCarLike(x, y, theta, goal_th);

    ROS_INFO("Trajectories created Vel=%f Theta=%f cost=%f",best.lin_vel_,best.thetav_,best.cost_);

    if(best.cost_ < 0){
    	accerman_theta = 0;
    	lin_vel = 0;
    }
    else{
    	accerman_theta = best.thetav_;
    	lin_vel = best.lin_vel_;
    }
    last_ackerman_theta_ = accerman_theta; //remember ackerman theta
    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //build the oriented footprint
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = x_i;
    robot_position.y = y_i;

    //check if the footprint is legal
    double footprint_cost = world_model_.footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);

    return footprint_cost;
  }

  void TrajectoryPlanner::getLineCells(int x0, int x1, int y0, int y1, vector<carlike_local_planner::Position2DInt>& pts){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    carlike_local_planner::Position2DInt pt;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      pt.x = x;      //Draw the current pixel
      pt.y = y;
      pts.push_back(pt);

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }
  }

  //get the cellsof a footprint at a given position
  vector<carlike_local_planner::Position2DInt> TrajectoryPlanner::getFootprintCells(double x_i, double y_i, double theta_i, bool fill){
    vector<carlike_local_planner::Position2DInt> footprint_cells;

    //if we have no footprint... do nothing
    if(footprint_spec_.size() <= 1){
      unsigned int mx, my;
      if(costmap_.worldToMap(x_i, y_i, mx, my)){
        Position2DInt center;
        center.x = mx;
        center.y = my;
        footprint_cells.push_back(center);
      }
      return footprint_cells;
    }

    //pre-compute cos and sin values
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    double new_x, new_y;
    unsigned int x0, y0, x1, y1;
    unsigned int last_index = footprint_spec_.size() - 1;

    for(unsigned int i = 0; i < last_index; ++i){
      //find the cell coordinates of the first segment point
      new_x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x0, y0))
        return footprint_cells;

      //find the cell coordinates of the second segment point
      new_x = x_i + (footprint_spec_[i + 1].x * cos_th - footprint_spec_[i + 1].y * sin_th);
      new_y = y_i + (footprint_spec_[i + 1].x * sin_th + footprint_spec_[i + 1].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x1, y1))
        return footprint_cells;

      getLineCells(x0, x1, y0, y1, footprint_cells);
    }

    //we need to close the loop, so we also have to raytrace from the last pt to first pt
    new_x = x_i + (footprint_spec_[last_index].x * cos_th - footprint_spec_[last_index].y * sin_th);
    new_y = y_i + (footprint_spec_[last_index].x * sin_th + footprint_spec_[last_index].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x0, y0))
      return footprint_cells;

    new_x = x_i + (footprint_spec_[0].x * cos_th - footprint_spec_[0].y * sin_th);
    new_y = y_i + (footprint_spec_[0].x * sin_th + footprint_spec_[0].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x1, y1))
      return footprint_cells;

    getLineCells(x0, x1, y0, y1, footprint_cells);

    if(fill)
      getFillCells(footprint_cells);

    return footprint_cells;
  }

  void TrajectoryPlanner::getFillCells(vector<carlike_local_planner::Position2DInt>& footprint){
    //quick bubble sort to sort pts by x
    carlike_local_planner::Position2DInt swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1){
      if(footprint[i].x > footprint[i + 1].x){
        swap = footprint[i];
        footprint[i] = footprint[i + 1];
        footprint[i + 1] = swap;
        if(i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    carlike_local_planner::Position2DInt min_pt;
    carlike_local_planner::Position2DInt max_pt;
    unsigned int min_x = footprint[0].x;
    unsigned int max_x = footprint[footprint.size() -1].x;
    //walk through each column and mark cells inside the footprint
    for(unsigned int x = min_x; x <= max_x; ++x){
      if(i >= footprint.size() - 1)
        break;

      if(footprint[i].y < footprint[i + 1].y){
        min_pt = footprint[i];
        max_pt = footprint[i + 1];
      }
      else{
        min_pt = footprint[i + 1];
        max_pt = footprint[i];
      }

      i += 2;
      while(i < footprint.size() && footprint[i].x == x){
        if(footprint[i].y < min_pt.y)
          min_pt = footprint[i];
        else if(footprint[i].y > max_pt.y)
          max_pt = footprint[i];
        ++i;
      }

      //loop though cells in the column
      for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        footprint.push_back(pt);
      }
    }
  }

  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = map_.goal_x_;
    y = map_.goal_y_;
  }

};


