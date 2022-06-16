#include<radiation_layer/rad_layer.h>
#include <pluginlib/class_list_macros.h>

#include <costmap_2d/footprint.h> // Added for footprint functions - inflation of radiation sensor
#include <costmap_2d/costmap_math.h>  // Added for updateWithMax
#include <cmath> // Required for basic maths operators

PLUGINLIB_EXPORT_CLASS(radiation_layer_namespace::RadLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;  // 255
using costmap_2d::LETHAL_OBSTACLE;  // 254
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // 253
using costmap_2d::FREE_SPACE;  // 0

namespace radiation_layer_namespace
{

RadLayer::RadLayer() {}

// How to Initialize the layer
void RadLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  node_name_ = ros::this_node::getName() + "/" + name_; // Useful for logging to user console when many layers are running
  current_ = true;
  default_value_ = NO_INFORMATION;  // What should the layer be filled with by default
  rolling_window_ = layered_costmap_->isRolling();

  // Setup up clean averages storage
  averages_ = NULL;
  weight_obs_= NULL;
  averages_size_ = 0;

  update_full_layer_ = false;


  matchSize();  // Resize layer to match the layered costmap (usually is same as Static layer) OBSTACLE LAYER USES OBSTACLELAYER::MATCHSIZE???
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Get radiation topic name
  std::string radiation_topic;
  nh.param("radiation_topic", radiation_topic, std::string("radiation_topic"));

  inflate_radiation_ = false;
  sensor_footprint_ = makeSensorFootprintFromParams(nh);
  nh.param("averaging_scale_length", averaging_scale_length_, 0.0);
  nh.param("combination_method", combination_method_, 0);
  nh.param("minimum_weight", minimum_weight_, 0.0);


  // Setup subscriber to topic, callback function = radiationCB
  radiation_sub_ = g_nh.subscribe(radiation_topic, 1, &RadLayer::radiationCB, this);

  // Initialise threshold values - WILL REPLACE WITH RECONFIGURE PARAMETERS LATER
  upper_threshold_ = 0.2;  // At what averaged float value will = 254 (LETHAL_OBSTACLE) or 252 when not using lethal cost
  upper_threshold_scale_ = 3;
  ut_ = upper_threshold_ * pow(10.0, upper_threshold_scale_);
  lower_threshold_ = 1; // Cut off value where averaged float values are igored
  lower_threshold_scale_ = 0;
  lt_ = lower_threshold_ * pow(10.0, lower_threshold_scale_);
  use_lethal_ = false;
  max_cost_ = INSCRIBED_INFLATED_OBSTACLE - 1;

  


  // Add dynamic reconfigure to layer
  dsrv_ = new dynamic_reconfigure::Server<radiation_layer::RadiationLayerConfig>(nh);
  dynamic_reconfigure::Server<radiation_layer::RadiationLayerConfig>::CallbackType cb = boost::bind(
      &RadLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void RadLayer::matchSize()
{
  // Cache current observation data
  // first pair = x & y coords, second pair = weight_obs and average
  std::list<std::pair<std::pair<double,double>, std::pair<float,float> > > observation_cache;
  getCache(observation_cache);

  // Ensure local costmap layer size and resolution matches master
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

  // Resize arrays for data averages and weight_obs per cell
  unsigned int size_x = master->getSizeInCellsX();
  unsigned int size_y = master->getSizeInCellsY();
  // Set up correct size for averages_
  if (averages_ == NULL) {
    //ROS_WARN("RadLayer::updateCosts(): averages_ array is NULL");
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    weight_obs_ = new float[averages_size_];
  }
  else if (averages_size_ != size_x * size_y)
  {
    //ROS_WARN("RadLayer::updateCosts(): averages_ array size is wrong");
    delete[] averages_;
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    weight_obs_ = new float[averages_size_];
  }
  std::fill_n(weight_obs_, averages_size_, 0.0);
  std::fill_n(averages_, averages_size_, std::numeric_limits<float>::quiet_NaN()); // Fill averages_ with nan

  // Repopulate averages_ and weight_obs with any cached data

  for (std::list<std::pair<std::pair<double,double>, std::pair<float,float> > >::iterator cache_item = observation_cache.begin(); cache_item != observation_cache.end(); cache_item++) {
    unsigned int idx_x, idx_y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices
    if (!worldToMap(cache_item -> first.first, cache_item -> first.second, idx_x, idx_y)) {
      continue;
    }

    unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later

    weight_obs_[linear_idx] = cache_item -> second.first;
    averages_[linear_idx] = cache_item -> second.second;
  } //end for
  update_full_layer_ = true;
}


void RadLayer::reconfigureCB(radiation_layer::RadiationLayerConfig &config, uint32_t level)
{
  // If threshold values change, recompute entire costmap
  if (config.lower_threshold != lower_threshold_ || config.lower_threshold_scale != lower_threshold_scale_ || config.upper_threshold != upper_threshold_ || config.upper_threshold_scale != upper_threshold_scale_ || config.use_lethal != use_lethal_) {
    update_full_layer_ = true;  // Flag that all cell costs should be recalculated
  }
  
    bool previous_enabled = enabled_;
    enabled_ = config.enabled;

    double previous_lt = lt_;
    lower_threshold_ = config.lower_threshold;
    lower_threshold_scale_ = config.lower_threshold_scale;
    lt_ = lower_threshold_ * pow(10.0, lower_threshold_scale_);

    double previous_ut = ut_;
    upper_threshold_ = config.upper_threshold;
    upper_threshold_scale_ = config.upper_threshold_scale;
    ut_ = upper_threshold_ * pow(10.0, upper_threshold_scale_);

    // Check lower threshold < upper threshold
    // Equals check also prevents a divide by zero condition for cost scaling
    if (lt_ >= ut_)
    {
      // Swaps the highest and lowest values around into the right variables
      double temp_lt = lt_;
      lt_ = ut_;
      ut_ = temp_lt+0.1;  // Add small amount to avoid divide by zero   
      ROS_WARN("%s: Thresholds invalid - please check",node_name_.c_str());
    }

    bool previous_use_lethal = use_lethal_;
    use_lethal_ = config.use_lethal;
    max_cost_ = use_lethal_ ? LETHAL_OBSTACLE : INSCRIBED_INFLATED_OBSTACLE - 1;  // If true, set max_cost_ lethal_obstacle, else set to 252 (use of ? is a stand in for a if-else statement)
    

    // Used for logging changes in layer parameters //
    // Enabled
    if (enabled_ != previous_enabled)
    {
      //ROS_INFO(config.enabled ? ("Layer Enabled") : ("Layer Disabled"));
      ROS_INFO("%s: %s", node_name_.c_str(), enabled_ ? "Enabled" : "Disabled");
    }  

    // Thresholds
    if (lt_ != previous_lt || ut_ != previous_ut)
    {
      ROS_INFO("%s: Thresholds: Lower= %.2f, Upper= %.2f", node_name_.c_str(), lt_, ut_);
    }

    // Use lethal
    if (use_lethal_ != previous_use_lethal)
    {
      ROS_INFO("%s: %s", node_name_.c_str(), use_lethal_ ? "Lethal Cost Enabled" : "Lethal Cost Disabled");
    }
       
    
}

void RadLayer::radiationCB(const radiation_msgs::DoseRate& rad_msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);  // Ensure only this functon can interact with radiation radiation_msg_buffer_
  radiation_msg_buffer_.push_back(rad_msg);  // Place new message onto buffer
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release control of the buffer
}

void RadLayer::updateObservations(std::list<std::pair<unsigned int, float> > &updates)
{
  std::list<radiation_msgs::DoseRate> radiation_msg_buffer_copy_;  // Create blank buffer for observations

  boost::recursive_mutex::scoped_lock lock(lock_);  // Restrict access to buffer to this function only (i.e. can't be updated via radiationCB())
  radiation_msg_buffer_copy_ = std::list<radiation_msgs::DoseRate>(radiation_msg_buffer_);  // Make copy of current observation buffer
  radiation_msg_buffer_.clear();  // Clear current observation buffer for new messages
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release access to observation buffer


  // Perform required processing on last set of observations //
  if (radiation_msg_buffer_copy_.size() == 0) {   //  If no observations are available for processing then return
    return;
  }

  // Loop over all observations
  for (std::list<radiation_msgs::DoseRate>::iterator obs = radiation_msg_buffer_copy_.begin(); obs != radiation_msg_buffer_copy_.end(); obs++) {

    // Convert observation frame to costmap frame at message timestamp
    geometry_msgs::TransformStamped transformStamped;  // Store transform between frames

    try
    {
      // Get transform between observation frame and costmap frame.  tf_ is somehow passed to the layer (hence it is not declared in this layer), but I don't where or how
      transformStamped = tf_ -> lookupTransform(global_frame_, obs -> header.frame_id, obs -> header.stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;  // If cannot get transform return
    }

    // Extract radiation data value from DoseRate message
    float value = obs -> rate; 

    // Vector of x and y cell indices
    std::vector<costmap_2d::MapLocation> cell_locations;

    if (inflate_radiation_) {
      // Vector of x and y cell indices
      std::vector<costmap_2d::MapLocation> footprint_locations;
      // Output footprint
      std::vector<geometry_msgs::Point> transformed_footprint;
      
      // Find yaw of radiation footprint
      double yaw_sub = tf2::getYaw(transformStamped.transform.rotation);
      // Transform footprint to be centred about the sensor, transformFootprint(x,y,yaw,footprint,footprint_out)
      costmap_2d::transformFootprint(transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw_sub, sensor_footprint_, transformed_footprint);


      // Loop over footprint points
      for (unsigned int i = 0; i < transformed_footprint.size(); ++i)
      {
        costmap_2d::MapLocation loc;
        if (worldToMap(transformed_footprint[i].x, transformed_footprint[i].y, loc.x, loc.y))
        {
          footprint_locations.push_back(loc);  // If footprint inbounds, push back
        }
      }
      convexFillCells(footprint_locations, cell_locations);
    } else {  // end if inflate_radiation_
      costmap_2d::MapLocation loc;
      if (!worldToMap(transformStamped.transform.translation.x, transformStamped.transform.translation.y, loc.x, loc.y)) {
      ROS_WARN("Observation out of bounds");
      continue;
      }
      cell_locations.push_back(loc);
    }  // end if inflate_radiation
    
    for (size_t i = 0; i < cell_locations.size(); i++)
    {
      unsigned int idx_x = cell_locations[i].x, idx_y = cell_locations[i].y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices

      // GET POLYGON AREA BASED ON MEASUREMENT LOCATION
      // PRODUCE LIST OF CELL INDICES TO BE UPDATED INSIDE THE POLYGON
      // FOR ELEMENTS IN LIST{UPDATE weight_obs_, calculate averages_}
      unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later


      

      double cell_pos_x, cell_pos_y;
      mapToWorld(idx_x, idx_y, cell_pos_x, cell_pos_y);
      double dist = distance(transformStamped.transform.translation.x, transformStamped.transform.translation.y, cell_pos_x, cell_pos_y);
      double dist_weighting = 1.0;
      if (averaging_scale_length_ > 0.0)
      {
        dist_weighting = exp(-0.5 * pow((dist/(averaging_scale_length_)), 2.0));
      }
     

      // Catch first observation, where value will be nan and can't be averaged
      if (weight_obs_[linear_idx] == 0.0) {
        averages_[linear_idx] = value;
      }
      else
      {
        // Perform weighted average for cell
        // BASED ON CENTRE OF MASS EQUATION
        averages_[linear_idx] = ( (dist_weighting * value)+ (weight_obs_[linear_idx]*averages_[linear_idx]) ) / (dist_weighting + weight_obs_[linear_idx]);
        
      }
      weight_obs_[linear_idx] = weight_obs_[linear_idx] + dist_weighting;  // Update cell weight with total weight including last measurement
      // Update list of updated cells for costmap
      // NEED TO CHECK IF CELL INDEX IN PAIR LIST, THEREFORE UPDATE THAT INDEX, DON'T PUT THE INDEX IN TWICE, SAVE ON COMPUTATION
      if (weight_obs_[linear_idx] >= minimum_weight_)  // Only update costmap cells if cell weight is above minimum
      {
        std::pair <unsigned int, float> data_pair (linear_idx, averages_[linear_idx]);
        updates.push_back(data_pair);
      }
      
      
    }// end for loop over footprint cells

  } // end for loop over observations


}

void RadLayer::getCache(std::list<std::pair<std::pair<double,double>, std::pair<float,float> > > &observation_cache)
{
  float* averages_copy;
  float* weight_obs_copy;
  unsigned int averages_size_copy;

  // Ensure observation data is copied before being erased or modified by another function
  boost::recursive_mutex::scoped_lock lock(cache_);
  averages_copy = averages_;
  weight_obs_copy = weight_obs_;
  averages_size_copy = averages_size_;
  boost::recursive_mutex::scoped_lock unlock(cache_);

  for (size_t i = 0; i < averages_size_copy; i++) {
    if (weight_obs_copy[i] > 0.0) {
      unsigned int mx, my;
      double wx, wy;
      indexToCells(i, mx, my);  // Convert cell index to map cell (x and y index)
      mapToWorld(mx, my, wx, wy);  // Convert cell x and y to world coords

      // Pair up
      std::pair<double, double> cell_location (wx, wy);
      std::pair<float, float> observation_data (weight_obs_copy[i], averages_copy[i]);

      observation_cache.push_back(std::make_pair(cell_location, observation_data));  //Push a pair of pairs to list
    }
  } // end for
  return;
}

unsigned int RadLayer::scaledValue(float value)
{
  int scaled_value = int(max_cost_ * ((value - lt_) / (ut_ - lt_) ) );
  if (scaled_value > max_cost_) { // If cost value > LETHAL_OBSTACLE, then limit to LETHAL_OBSTACLE
    scaled_value = max_cost_;
  }
  if (scaled_value < FREE_SPACE) {
    scaled_value = FREE_SPACE;  // If cost value < 0, then limit to FREE_SPACE
  }

  return (unsigned int)scaled_value;
}

void RadLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  // If layer not enabled, do not record data to local costmap - not sure why you would _want_ to do this for radiation
  // if (!enabled_)
  //   return;

  if (rolling_window_){
    matchSize(); // Will cache all current radiation data, plus set the origin etc of the map
  }
  std::list<std::pair<unsigned int, float> > updates;  // Store all updates for processing

  if (update_full_layer_) {
    for (size_t i = 0; i < averages_size_; i++) {
      if (weight_obs_[i] > 0.0) {  // If observations have been entered for that cell
        std::pair <unsigned int, float> data_pair (i, averages_[i]);  // Create index and average value pair
        updates.push_back(data_pair);  // Push back pair to updates
      }
    } // end for
    update_full_layer_ = false;
  }
    
  updateObservations(updates);// Include new observations
  

  // Take all observation updates and mark costmap accordingly
  for (std::list<std::pair<unsigned int, float> >::iterator updt = updates.begin(); updt != updates.end(); updt++) {
    unsigned int temp_idx = updt -> first;  // first of pair = index
    float temp_value = updt -> second;  // Second of pair = averaged float value
    costmap_[temp_idx] = scaledValue(temp_value);  // Set scaled (0-254) value based on thresholds in costmap
  }

  // THIS WORKED - TAKEN FROM INFLATION LAYER
  // Allows for total enable/disable of layer, removing all cost when !enabled
  // I don't understand why this works rather than updating the min to max of only all observed data
  *min_x = -std::numeric_limits<float>::max();
  *min_y = -std::numeric_limits<float>::max();
  *max_x = std::numeric_limits<float>::max();
  *max_y = std::numeric_limits<float>::max();
}

void RadLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_){
    return;
  }

switch (combination_method_)
  {
    case 0:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 2:  // Addition
      updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 3:  // Maximum, preserve No Info
      updatePreserveNoInfo(master_grid, min_i, min_j, max_i, max_j);
      break;  
    default:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
  }
  // UPDATE WITH MAX //
  // Nice behaviour because it maintains lethal obstacles from other layers
  // if (master_cost ==  NO_INFORMATION || master_cost < local_cost): update
  // updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH MAX PRESERVE NO INFO
  // Same as updateWithMax, however, it does not overwrite NO_INFO (255) cells
  // This is useful for only computing cost for cells observed by a LiDAR or an existing map
  // updatePreserveNoInfo(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH OVERWRITE //
  // Regardless of other layers, overwrite
  // if (local != NO_INFORMATION): update
  // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH ADDITION //
  // update with master_cost + local_cost, up to max_val = 253 (i.e. LETHAL_OBSTACLE - 1)
  // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);

}

void RadLayer::updatePreserveNoInfo(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == NO_INFORMATION){
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost < costmap_[it])
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

std::vector<geometry_msgs::Point> RadLayer::makeSensorFootprintFromParams(ros::NodeHandle& nh)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh.searchParam("radiation_footprint", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);

    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
    {
      if (costmap_2d::makeFootprintFromString(std::string(footprint_xmlrpc), points))
      {
        costmap_2d::writeFootprintToParam(nh, points);
        inflate_radiation_ = true;
        return points;
      }
    }
    else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray  && footprint_xmlrpc.size() > 2)
    {
      points = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      costmap_2d::writeFootprintToParam(nh, points);
      inflate_radiation_ = true;
      return points;
    }
  }

  // If radius is given, use this instead of footprint
  if (nh.searchParam("radiation_radius", full_radius_param_name))
  {
    double radiation_radius;
    nh.param(full_radius_param_name, radiation_radius, 0.0);
    points = costmap_2d::makeFootprintFromRadius(radiation_radius);
    nh.setParam("radiation_radius", radiation_radius);
    inflate_radiation_ = true;
  }
  return points;
}

} // end namespace
