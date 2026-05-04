#include "pose_graph_editor_panel/pose_graph_editor_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace pose_graph_editor_panel
{

PoseGraphEditorPanel::PoseGraphEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
  //@---------- Read file button
  QGroupBox *readfile_groupBox = new QGroupBox(tr("Open file"));
  pose_graph_dir_btn_ = new QPushButton(this);
  pose_graph_dir_btn_->setText("Open File");
  connect(pose_graph_dir_btn_, SIGNAL(clicked()), this, SLOT(openPoseGraph()));
  QVBoxLayout *readfile_vbox = new QVBoxLayout;
  readfile_vbox->addWidget(pose_graph_dir_btn_);
  readfile_vbox->addStretch(1);
  readfile_groupBox->setLayout(readfile_vbox);

  //@---------- Radion group box
  QGroupBox *radio_groupBox = new QGroupBox(tr("Please Select Current Pose Graph You Want To Work On"));

  pg_radio_0_ = new QRadioButton(tr("Pose Graph 0"));
  pg_radio_1_ = new QRadioButton(tr("Pose Graph 1"));
  pg_radio_2_ = new QRadioButton(tr("Pose Graph 2"));
  radio_buttons_.push_back(pg_radio_0_);
  radio_buttons_.push_back(pg_radio_1_);
  radio_buttons_.push_back(pg_radio_2_);

  pg_radio_0_->setChecked(true);
  QVBoxLayout *radio_vbox = new QVBoxLayout;
  radio_vbox->addWidget(pg_radio_0_);
  radio_vbox->addWidget(pg_radio_1_);
  radio_vbox->addWidget(pg_radio_2_);
  connect(pg_radio_0_, SIGNAL(clicked()), this, SLOT(pg_radio_0_clicked()));
  connect(pg_radio_1_, SIGNAL(clicked()), this, SLOT(pg_radio_1_clicked()));
  connect(pg_radio_2_, SIGNAL(clicked()), this, SLOT(pg_radio_2_clicked()));
  radio_vbox->addStretch(1);
  radio_groupBox->setLayout(radio_vbox);
  
  //@---------- Operation group box
  QGroupBox *icp_groupBox = new QGroupBox(tr("ICP tools"));

  do_icp_ = new QPushButton(tr("ICP"));
  connect(do_icp_, SIGNAL(clicked()), this, SLOT(icp_clicked ()));
  accept_result_ = new QPushButton(tr("Accept"));
  connect(accept_result_, SIGNAL(clicked()), this, SLOT(accept_clicked ()));
  last_step_ = new QPushButton(tr("Last Step"));
  connect(last_step_, SIGNAL(clicked()), this, SLOT(last_step_clicked ()));
  export_map_ = new QPushButton(tr("Export Result"));
  connect(export_map_, SIGNAL(clicked()), this, SLOT(export_map_clicked ()));

  px_increase_ = new QPushButton(tr("px+"));
  px_decrease_ = new QPushButton(tr("px-"));
  py_increase_ = new QPushButton(tr("py+"));
  py_decrease_ = new QPushButton(tr("py-"));
  pz_increase_ = new QPushButton(tr("pz+"));
  pz_decrease_ = new QPushButton(tr("pz-"));
  connect(px_increase_, SIGNAL(clicked()), this, SLOT(pxIncrease ()));
  connect(px_decrease_, SIGNAL(clicked()), this, SLOT(pxDecrease ()));
  connect(py_increase_, SIGNAL(clicked()), this, SLOT(pyIncrease ()));
  connect(py_decrease_, SIGNAL(clicked()), this, SLOT(pyDecrease ()));
  connect(pz_increase_, SIGNAL(clicked()), this, SLOT(pzIncrease ()));
  connect(pz_decrease_, SIGNAL(clicked()), this, SLOT(pzDecrease ()));

  roll_increase_ = new QPushButton(tr("roll+"));
  roll_decrease_ = new QPushButton(tr("roll-"));
  pitch_increase_ = new QPushButton(tr("pitch+"));
  pitch_decrease_ = new QPushButton(tr("pitch-"));
  yaw_increase_ = new QPushButton(tr("yaw+"));
  yaw_decrease_ = new QPushButton(tr("yaw-"));
  connect(roll_increase_, SIGNAL(clicked()), this, SLOT(rollIncrease ()));
  connect(roll_decrease_, SIGNAL(clicked()), this, SLOT(rollDecrease ()));
  connect(pitch_increase_, SIGNAL(clicked()), this, SLOT(pitchIncrease ()));
  connect(pitch_decrease_, SIGNAL(clicked()), this, SLOT(pitchDecrease ()));
  connect(yaw_increase_, SIGNAL(clicked()), this, SLOT(yawIncrease ()));
  connect(yaw_decrease_, SIGNAL(clicked()), this, SLOT(yawDecrease ()));
  
  QGridLayout *grid_layout = new QGridLayout;
  
  //addWidget(*Widget, row, column, rowspan, colspan)
  grid_layout->addWidget(export_map_, 0, 0, 1, 1);
  grid_layout->addWidget(last_step_, 0, 1, 1, 1);
  grid_layout->addWidget(do_icp_, 0, 2, 1, 1);
  grid_layout->addWidget(accept_result_, 0, 3, 1, 1);

  grid_layout->addWidget(px_increase_, 1, 0, 1, 1);
  grid_layout->addWidget(py_increase_, 1, 1, 1, 1);
  grid_layout->addWidget(pz_increase_, 1, 2, 1, 1);
  grid_layout->addWidget(roll_increase_, 1, 3, 1, 1);
  grid_layout->addWidget(pitch_increase_, 1, 4, 1, 1);
  grid_layout->addWidget(yaw_increase_, 1, 5, 1, 1);

  grid_layout->addWidget(px_decrease_, 2, 0, 1, 1);
  grid_layout->addWidget(py_decrease_, 2, 1, 1, 1);
  grid_layout->addWidget(pz_decrease_, 2, 2, 1, 1);
  grid_layout->addWidget(roll_decrease_, 2, 3, 1, 1);
  grid_layout->addWidget(pitch_decrease_, 2, 4, 1, 1);
  grid_layout->addWidget(yaw_decrease_, 2, 5, 1, 1);
  icp_groupBox->setLayout(grid_layout);

  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  
  layout->addWidget(readfile_groupBox);
  layout->addWidget(radio_groupBox);
  layout->addWidget(icp_groupBox);
  setLayout(layout);

  pose_graph_dir_btn_->setEnabled(true);
  
}

void PoseGraphEditorPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pose_graph_dir_pub_ = raw_node->create_publisher<std_msgs::msg::String>("pose_graph_dir", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  current_pose_graphs_sub_ = raw_node->create_subscription<std_msgs::msg::String>("current_pose_graphs", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&PoseGraphEditorPanel::cpgHandler, this, std::placeholders::_1));
  
  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  current_focus_pg_pub_  = raw_node->create_publisher<std_msgs::msg::String>("current_focus_pg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  clock_ = raw_node->get_clock();

  pg_radio_0_clicked();
}

void PoseGraphEditorPanel::pg_radio_0_clicked(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "pg_0";
  current_focus_pg_pub_->publish(tmp_str);
}

void PoseGraphEditorPanel::pg_radio_1_clicked(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "pg_1";
  current_focus_pg_pub_->publish(tmp_str);
}

void PoseGraphEditorPanel::pg_radio_2_clicked(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "pg_2";
  current_focus_pg_pub_->publish(tmp_str);
}

void PoseGraphEditorPanel::openPoseGraph()
{
  QFileDialog FileDialog;
  QString path=FileDialog.getExistingDirectory(this, tr("Open Directory"),  "/", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);

  std::string dir = path.toStdString();

  RCLCPP_INFO(rclcpp::get_logger("pose_graph_editor_panel"), "%s", dir.c_str());

  std_msgs::msg::String tmp_str;
  tmp_str.data = dir;
  pose_graph_dir_pub_->publish(tmp_str);
}

void PoseGraphEditorPanel::cpgHandler(const std_msgs::msg::String::SharedPtr msg){

  all_pose_graphs_.clear();
  std::stringstream pg_full_name(msg->data);
  std::string one_pg_full_name;
  while(std::getline(pg_full_name, one_pg_full_name, ';'))
  {
    all_pose_graphs_.push_back(one_pg_full_name);
  }
  
  for(size_t i=0;i<all_pose_graphs_.size();i++){
    radio_buttons_[i]->setText(tr(all_pose_graphs_[i].c_str()));
  }
  RCLCPP_DEBUG(rclcpp::get_logger("pose_graph_editor_panel"), "%s", msg->data.c_str());

  if(all_pose_graphs_.size()>1){
    pose_graph_dir_btn_->setVisible(false);
  }

}

void PoseGraphEditorPanel::pubOperationCommand(std::string m_string){
  std_msgs::msg::String tmp_str;
  tmp_str.data = m_string;
  operation_command_pub_->publish(tmp_str);
}

void PoseGraphEditorPanel::icp_clicked(){ pubOperationCommand(std::string("icp")); }
void PoseGraphEditorPanel::accept_clicked(){ pubOperationCommand(std::string("accept")); }
void PoseGraphEditorPanel::last_step_clicked(){pubOperationCommand(std::string("last")); }

void PoseGraphEditorPanel::export_map_clicked(){
  pubOperationCommand(std::string("export_map")); 
  QMessageBox::information(this, "Message", "Maps are saved.", QMessageBox::Ok);
}

void PoseGraphEditorPanel::pxIncrease(){ pubOperationCommand(std::string("px+")); }
void PoseGraphEditorPanel::pxDecrease(){ pubOperationCommand(std::string("px-")); }
void PoseGraphEditorPanel::pyIncrease(){ pubOperationCommand(std::string("py+")); }
void PoseGraphEditorPanel::pyDecrease(){ pubOperationCommand(std::string("py-")); }
void PoseGraphEditorPanel::pzIncrease(){ pubOperationCommand(std::string("pz+")); }
void PoseGraphEditorPanel::pzDecrease(){ pubOperationCommand(std::string("pz-")); }

void PoseGraphEditorPanel::rollIncrease(){ pubOperationCommand(std::string("roll+")); }
void PoseGraphEditorPanel::rollDecrease(){ pubOperationCommand(std::string("roll-")); }
void PoseGraphEditorPanel::pitchIncrease(){ pubOperationCommand(std::string("pitch+")); }
void PoseGraphEditorPanel::pitchDecrease(){ pubOperationCommand(std::string("pitch-")); }
void PoseGraphEditorPanel::yawIncrease(){ pubOperationCommand(std::string("yaw+")); }
void PoseGraphEditorPanel::yawDecrease(){ pubOperationCommand(std::string("yaw-")); }

}  // namespace pose_graph_editor_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pose_graph_editor_panel::PoseGraphEditorPanel, rviz_common::Panel)
