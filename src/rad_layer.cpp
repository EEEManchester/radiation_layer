#include<radiation_layer/rad_layer.h>
#include <pluginlib/class_list_macros.h>

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
  current_ = true;
  default_value_ = NO_INFORMATION;  // What should the layer be filled with by default

  // Setup up clean averages storage
  averages_ = NULL;
  n_obs_ = NULL;
  averages_size_ = 0;

  update_full_layer_ = false;


  matchSize();  // Resize layer to match the layered costmap (usually is same as Static layer) OBSTACLE LAYER USES OBSTACLELAYER::MATCHSIZE???
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Get radiation topic name
  std::string radiation_topic;
  nh.param("radiation_topic", radiation_topic, std::string("radiation_topic"));
  // Setup subscriber to topic, callback function = radiationCB
  radiation_sub_ = g_nh.subscribe(radiation_topic, 1, &RadLayer::radiationCB, this);

  // Initialise threshold values - WILL REPLACE WITH PARAMETERS LATER
  upper_threshold_ = 0.2;  // At what averaged float value will = 254 (LETHAL_OBSTACLE)
  upper_threshold_scale_ = 3;
  ut_ = upper_threshold_ * pow(10.0, upper_threshold_scale_);
  lower_threshold_ = 1; // Cut off value where averaged float values are igored
  lower_threshold_scale_ = 0;
  lt_ = lower_threshold_ * pow(10.0, lower_threshold_scale_);


  // Add dynamic reconfigure to layer
  // dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  // dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
  //     &RadLayer::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);
  dsrv_ = new dynamic_reconfigure::Server<radiation_layer::RadiationLayerConfig>(nh);
  dynamic_reconfigure::Server<radiation_layer::RadiationLayerConfig>::CallbackType cb = boost::bind(
      &RadLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void RadLayer::matchSize()
{
  // Cache current observation data
  // first pair = x & y coords, second pair = n_obs and average
  std::list<std::pair<std::pair<double,double>, std::pair<unsigned int,float> > > observation_cache;
  getCache(observation_cache);

  // Ensure local costmap layer size and resolution matches master
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

  // Resize arrays for data averages and n_obs per cell
  unsigned int size_x = master->getSizeInCellsX();
  unsigned int size_y = master->getSizeInCellsY();
  // Set up correct size for averages_
  if (averages_ == NULL) {
    //ROS_WARN("RadLayer::updateCosts(): averages_ array is NULL");
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    n_obs_ = new unsigned int[averages_size_];
  }
  else if (averages_size_ != size_x * size_y)
  {
    //ROS_WARN("RadLayer::updateCosts(): averages_ array size is wrong");
    delete[] averages_;
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    n_obs_ = new unsigned int[averages_size_];
  }
  memset(n_obs_, 0, size_x * size_y * sizeof(unsigned int));  // Fill n_observations with 0
  std::fill_n(averages_, averages_size_, std::numeric_limits<float>::quiet_NaN()); // Fill averages_ with nan

  // Repopulate averages_ and n_obs_ with any cached data

  for (std::list<std::pair<std::pair<double,double>, std::pair<unsigned int,float> > >::iterator cache_item = observation_cache.begin(); cache_item != observation_cache.end(); cache_item++) {
    unsigned int idx_x, idx_y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices
    if (!worldToMap(cache_item -> first.first, cache_item -> first.second, idx_x, idx_y)) {
      return;
    }

    unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later

    n_obs_[linear_idx] = cache_item -> second.first;
    averages_[linear_idx] = cache_item -> second.second;
  } //end for
  update_full_layer_ = true;
}


// void RadLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
// {
//   enabled_ = config.enabled;
// }

void RadLayer::reconfigureCB(radiation_layer::RadiationLayerConfig &config, uint32_t level)
{
  // If threshold values change, recompute entire costmap
  if (config.lower_threshold != lower_threshold_ || config.lower_threshold_scale != lower_threshold_scale_ || config.upper_threshold != upper_threshold_ || config.upper_threshold_scale != upper_threshold_scale_) {
    update_full_layer_ = true;  // Flag that all cell costs should be recalculated
  }
    enabled_ = config.enabled;

    lower_threshold_ = config.lower_threshold;
    lower_threshold_scale_ = config.lower_threshold_scale;
    lt_ = lower_threshold_ * pow(10.0, lower_threshold_scale_);

    upper_threshold_ = config.upper_threshold;
    upper_threshold_scale_ = config.upper_threshold_scale;
    ut_ = upper_threshold_ * pow(10.0, upper_threshold_scale_);

    ROS_INFO("Updated thresholds: Lower= %.2f, Upper= %.2f", lt_, ut_);
}

void RadLayer::radiationCB(const sensor_msgs::Image& rad_msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);  // Ensure only this functon can interact with radiation radiation_msg_buffer_
  radiation_msg_buffer_.push_back(rad_msg);  // Place new message onto buffer
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release control of the buffer
}

void RadLayer::updateObservations(std::list<std::pair<unsigned int, float> > &updates)
{
  std::list<sensor_msgs::Image> radiation_msg_buffer_copy_;  // Create blank buffer for observations

  boost::recursive_mutex::scoped_lock lock(lock_);  // Restrict access to buffer to this function only (i.e. can't be updated via radiationCB())
  radiation_msg_buffer_copy_ = std::list<sensor_msgs::Image>(radiation_msg_buffer_);  // Make copy of current observation buffer
  radiation_msg_buffer_.clear();  // Clear current observation buffer for new messages
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release access to observation buffer


  // Perform required processing on last set of observations //
  if (radiation_msg_buffer_copy_.size() == 0) {   //  If no observations are available for processing then return
    return;
  }

  // Loop over all observations
  for (std::list<sensor_msgs::Image>::iterator obs = radiation_msg_buffer_copy_.begin(); obs != radiation_msg_buffer_copy_.end(); obs++) {

    // Convert observation frame to costmap frame at message timestamp
    tf::StampedTransform transformStamped;  // Store transform between frames

    try
    {
      // Get transform between observation frame and costmap frame.  tf_ is somehow passed to the layer (hence it is not declared in this layer), but I don't where or how
      tf_ -> lookupTransform(global_frame_, obs -> header.frame_id, obs -> header.stamp, transformStamped);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;  // If cannot get transform return
    }


    unsigned int idx_x, idx_y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices
    if (!worldToMap(transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), idx_x, idx_y)) {
      ROS_WARN("Observation out of bounds");
      return;
    }

    unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later

    // Extract radiation data value from image message - associated with camera green channel
    float value = obs -> data[1];  // Retrive 2nd channel (index 1) from data field in message
    // SHOULD PROBABLY DO SOME CHECKS HERE - index errors, no data errors, wrong format etc

    n_obs_[linear_idx] = n_obs_[linear_idx] + 1; // Increase number of observation in cell by 1

    // Catch first observation, where value will be nan and can't be averaged
    if (n_obs_[linear_idx] == 1) {
      averages_[linear_idx] = value;
    }
    else
    {
      // Perform weighted average for cell
      averages_[linear_idx] = ((1.0/n_obs_[linear_idx])*value) + ((1.0 - 1.0/n_obs_[linear_idx])*averages_[linear_idx]);
    }

    // Update list of updated cells for costmap
    std::pair <unsigned int, float> data_pair (linear_idx, averages_[linear_idx]);
    updates.push_back(data_pair);

  } // end for loop over observations


}

void RadLayer::getCache(std::list<std::pair<std::pair<double,double>, std::pair<unsigned int,float> > > &observation_cache)
{
  float* averages_copy;
  unsigned int* n_obs_copy;
  unsigned int averages_size_copy;

  // Ensure observation data is copied before being erased or modified by another function
  boost::recursive_mutex::scoped_lock lock(cache_);
  averages_copy = averages_;
  n_obs_copy = n_obs_;
  averages_size_copy = averages_size_;
  boost::recursive_mutex::scoped_lock unlock(cache_);

  for (size_t i = 0; i < averages_size_copy; i++) {
    if (n_obs_copy[i] > 0) {
      unsigned int mx, my;
      double wx, wy;
      indexToCells(i, mx, my);  // Convert cell index to map cell (x and y index)
      mapToWorld(mx, my, wx, wy);  // Convert cell x and y to world coords

      // Pair up
      std::pair<double, double> cell_location (wx, wy);
      std::pair<unsigned int, float> observation_data (n_obs_copy[i], averages_copy[i]);

      observation_cache.push_back(std::make_pair(cell_location, observation_data));  //Push a pair of pairs to list
    }
  } // end for
  return;
}

unsigned int RadLayer::scaledValue(float value)
{
  int scaled_value = int(LETHAL_OBSTACLE * ((value - lt_) / (ut_ - lt_) ) );
  if (scaled_value > LETHAL_OBSTACLE) { // If cost value > LETHAL_OBSTACLE, then limit to LETHAL_OBSTACLE
    scaled_value = LETHAL_OBSTACLE;
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

  std::list<std::pair<unsigned int, float> > updates;  // Store all updates for processing

  if (update_full_layer_) {
    for (size_t i = 0; i < averages_size_; i++) {
      if (n_obs_[i] > 0) {  // If observations have been observed for that cell
        std::pair <unsigned int, float> data_pair (i, averages_[i]);  // Create index and average value pair
        updates.push_back(data_pair);  // Push back pair to updates
      }
    } // end for
    update_full_layer_ = false;
  }
  else
  {
    updateObservations(updates);
  }

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


  // UPDATE WITH MAX //
  // Nice behaviour because it maintains lethal obstacles from other layers
  // if (master_cost ==  NO_INFORMATION || master_cost < local_cost): update
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH OVERWRITE //
  // Regardless of other layers, overwrite
  // if (local != NO_INFORMATION): update
  // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH ADDITION //
  // update with master_cost + local_cost, up to max_val = 253 (i.e. LETHAL_OBSTACLE - 1)
  // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);

}

} // end namespace
