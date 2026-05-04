#include <semantic_segmentation2point_cloud.h>

SemanticSegmentation2PointCloud::SemanticSegmentation2PointCloud(std::string name):Node(name){
  
  has_info_ = false;
  width_start_ = 0;
  width_end_ = 848;
  height_start_ = 28;
  height_end_ = 452;

  this->declare_parameter("max_distance", rclcpp::ParameterValue(4.0));
  this->get_parameter("max_distance", max_distance_);
  RCLCPP_INFO(this->get_logger(), "max_distance: %.2f" , max_distance_);

  this->declare_parameter("sample_step", rclcpp::ParameterValue(2));
  this->get_parameter("sample_step", sample_step_);
  RCLCPP_INFO(this->get_logger(), "sample_step: %d" , sample_step_);

  this->declare_parameter("voxel_size", rclcpp::ParameterValue(0.05));
  this->get_parameter("voxel_size", leaf_size_);
  RCLCPP_INFO(this->get_logger(), "voxel_size: %.2f" , leaf_size_);

  downSizeFilter_intensity_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  downSizeFilter_rgb_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  
  this->declare_parameter("exclude_class", rclcpp::PARAMETER_INTEGER_ARRAY);
  rclcpp::Parameter exclude_class= this->get_parameter("exclude_class");
  exclude_class_vector_ = exclude_class.as_integer_array();

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sematic_segmentation_point_cloud", 2);

  cbs_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options2;
  sub_options2.callback_group = cbs_group2_;
  depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&SemanticSegmentation2PointCloud::cbCameraInfo, this, std::placeholders::_1), sub_options2);

  mask_sub_.subscribe(this, "ddrnet_inferenced_mask");
  depth_img_sub_.subscribe(this, "image_rect_raw");
  syncApproximate_ = std::make_shared<message_filters::Synchronizer<MaskDepthSyncPolicy>>(MaskDepthSyncPolicy(10), mask_sub_, depth_img_sub_);
  syncApproximate_->registerCallback(&SemanticSegmentation2PointCloud::cbMaskDepthImg, this);  

}

void SemanticSegmentation2PointCloud::cbMaskDepthImg(const sensor_msgs::msg::Image::SharedPtr mask,
                    const sensor_msgs::msg::Image::SharedPtr depth){

  RCLCPP_DEBUG(this->get_logger(), "Sync");
  RCLCPP_INFO_ONCE(this->get_logger(), "Semantic segmentation mask encoding is %s", mask->encoding.c_str());
  if(!has_info_){
    return;
  }

  try
  {
    cv_image_ = cv_bridge::toCvCopy(depth, depth->encoding);
    cv_mask_ = cv_bridge::toCvCopy(mask, mask->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if(mask->encoding=="mono8"){

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    float cx, cy, fx, fy;//principal point and focal lengths

    cx = camera_info_.k[2]; //(cloud->width >> 1) - 0.5f;
    cy = camera_info_.k[5]; //(cloud->height >> 1) - 0.5f;
    fx = 1.0f / camera_info_.k[0]; 
    fy = 1.0f / camera_info_.k[4]; 

    for (unsigned int v = 0; v < depth->height; v+=sample_step_)
    {
      for (unsigned int u = 0; u < depth->width; u+=sample_step_)
      {

        pcl::PointXYZI pt;
        float z = cv_image_->image.at<unsigned short>(v, u) * 0.001;
        char semantic_segmentation_class = 0;
        // Check is in mask
        if(v>=height_start_ && v<height_end_ && u>=width_start_ && u<width_end_){
          semantic_segmentation_class = cv_mask_->image.at<char>(v-height_start_, u-width_start_);
          pcl::PointCloud<pcl::PointXYZI> pc_i;
          xyzi_map_.insert(std::make_pair(semantic_segmentation_class, pc_i)); //it does not matter that insertion succeed or fail
        }
        else{
          continue;
        }
        // Check for invalid measurements
        if (std::isnan(z) || z>max_distance_)
        {
          continue;
          //pt.x = pt.y = pt.z = Z;
        }
        else // Fill in XYZ
        {
          auto it = std::find(exclude_class_vector_.begin(), exclude_class_vector_.end(), int(semantic_segmentation_class));
          if ( it!= exclude_class_vector_.end()) {
            continue;
          }
          pt.x = (u - cx) * z * fx;
          pt.y = (v - cy) * z * fy;
          pt.z = z;
          pt.intensity = int(semantic_segmentation_class);
        }
        cloud->push_back(pt);
        xyzi_map_[semantic_segmentation_class].push_back(pt);
      }
    }
    downSizeFilter_intensity_.setInputCloud(cloud);
    downSizeFilter_intensity_.filter(*cloud);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = depth->header;
    cloud_pub_->publish(output);
  }
  else if(mask->encoding=="8UC3"){
    
    xyzrgb_map_.clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    float cx, cy, fx, fy;//principal point and focal lengths

    cx = camera_info_.k[2]; //(cloud->width >> 1) - 0.5f;
    cy = camera_info_.k[5]; //(cloud->height >> 1) - 0.5f;
    fx = 1.0f / camera_info_.k[0]; 
    fy = 1.0f / camera_info_.k[4]; 

    for (unsigned int v = 0; v < depth->height; v+=sample_step_)
    {
      for (unsigned int u = 0; u < depth->width; u+=sample_step_)
      {
        pcl::PointXYZRGB pt;
        float z = cv_image_->image.at<unsigned short>(v, u) * 0.001;
        cv::Vec3b segmentation_color;
        unsigned int hex_value = 0;
        // Check is in mask
        if(v>=height_start_ && v<height_end_ && u>=width_start_ && u<width_end_){
          //@ depth v=480 --> mask v=452
          segmentation_color = cv_mask_->image.at<cv::Vec3b>(v-height_start_, u-width_start_);
          //@ Although color in rviz is RGB, but after cv convert, it is back to bgr again!!!
          pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
          unsigned int hex_value = (static_cast<unsigned int>(segmentation_color[2]) << 16) |
                             (static_cast<unsigned int>(segmentation_color[1]) << 8) |
                             static_cast<unsigned int>(segmentation_color[0]);
          xyzrgb_map_.insert(std::make_pair(hex_value, pc_rgb)); //it does not matter that insertion succeed or fail
        }
        else{
          continue;
        }
        // Check for invalid measurements
        if (std::isnan(z) || z>max_distance_)
        {
          continue;
          //pt.x = pt.y = pt.z = Z;
        }
        else // Fill in XYZ
        {
          pt.x = (u - cx) * z * fx;
          pt.y = (v - cy) * z * fy;
          pt.z = z;
          pt.r = segmentation_color[2];
          pt.g = segmentation_color[1];
          pt.b = segmentation_color[0];
        }
        cloud->push_back(pt);
        xyzrgb_map_[hex_value].push_back(pt);
      }
    }
    RCLCPP_DEBUG(this->get_logger(), "%lu classes have been detected", xyzrgb_map_.size());
    downSizeFilter_rgb_.setInputCloud(cloud);
    downSizeFilter_rgb_.filter(*cloud);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = depth->header;
    cloud_pub_->publish(output);
  }


}

void SemanticSegmentation2PointCloud::cbCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
  
  //RCLCPP_INFO_ONCE(this->get_logger(), "Got camera info.");
  if(!has_info_){
    has_info_ = true;
    camera_info_ = *msg;
  }

}

// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  SemanticSegmentation2PointCloud SS2C = SemanticSegmentation2PointCloud("depthimg2pointcloud_right");

  rclcpp::executors::MultiThreadedExecutor::SharedPtr mulexecutor_;

  mulexecutor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  mulexecutor_->add_node(SS2C.get_node_base_interface());
   
  mulexecutor_->spin();

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;

}