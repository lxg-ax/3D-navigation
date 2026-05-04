#include "map_editor_panel/map_editor_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace map_editor_panel
{

MapEditorPanel::MapEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{  
  sub_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  //@---------- operation button
  QGroupBox *operation_groupBox = new QGroupBox(tr("Map Editor Panel"));
  save_dir_btn_ = new QPushButton(this);
  save_dir_btn_->setText("Save Selected PointCloud");
  connect(save_dir_btn_, SIGNAL(clicked()), this, SLOT(saveSelectedPC()));

  clear_selection_btn_ = new QPushButton(this);
  clear_selection_btn_->setText("Clear Selection");
  connect(clear_selection_btn_, SIGNAL(clicked()), this, SLOT(clearSelection()));

  last_step_btn_ = new QPushButton(this);
  last_step_btn_->setText("Last Step");
  connect(last_step_btn_, SIGNAL(clicked()), this, SLOT(lastStep()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->addWidget(save_dir_btn_, 0, 0, 1, 1);
  grid_layout->addWidget(clear_selection_btn_, 0, 1, 1, 1);
  grid_layout->addWidget(last_step_btn_, 0, 2, 1, 1);
  operation_groupBox->setLayout(grid_layout);

  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  layout->addWidget(operation_groupBox);
  setLayout(layout);
  
}

void MapEditorPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  

  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("/point_cloud_selection/panel_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_selected_pc_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud_selection/selected_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&MapEditorPanel::selectedPCCb, this, std::placeholders::_1));

  clock_ = raw_node->get_clock();

}

void MapEditorPanel::selectedPCCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  pcl::fromROSMsg(*msg, *sub_pc_);
}

void MapEditorPanel::clearSelection(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "clear";
  operation_command_pub_->publish(tmp_str);
}

void MapEditorPanel::lastStep(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "last_step";
  operation_command_pub_->publish(tmp_str);
}


void MapEditorPanel::saveSelectedPC()
{
  
  QFileDialog FileDialog;
  QString path=FileDialog.getExistingDirectory(this, tr("Select Dir"),  "/", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);

  std::string dir = path.toStdString();
  
  if(dir == ""){
    RCLCPP_INFO(rclcpp::get_logger("map_editor_panel"), "Dir is empty.");
    return;
  }
  else{

    if(sub_pc_->points.empty()){
      RCLCPP_INFO(rclcpp::get_logger("map_editor_panel"), "Selected points is empty.");
      return;
    }
    
    std::string export_dir_string = dir + "/" + currentDateTime()+ "_map.pcd";
    RCLCPP_INFO(rclcpp::get_logger("map_editor_panel"), "Save selected point cloud to: %s", export_dir_string.c_str());
    pcl::PCDWriter w;
    w.writeASCII (export_dir_string, *sub_pc_, 3);
    
    std_msgs::msg::String tmp_str;
    tmp_str.data = "clear";
    operation_command_pub_->publish(tmp_str);
  }
  

}

}  // namespace map_editor_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_editor_panel::MapEditorPanel, rviz_common::Panel)
