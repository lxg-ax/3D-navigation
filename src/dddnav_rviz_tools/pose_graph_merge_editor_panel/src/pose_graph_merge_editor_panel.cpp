#include "pose_graph_merge_editor_panel/pose_graph_merge_editor_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace pose_graph_merge_editor_panel
{

PoseGraphMergeEditorPanel::PoseGraphMergeEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
  
  //@---------- Operation group box
  QGroupBox *icp_groupBox = new QGroupBox(tr("Merge ICP tools"));

  do_icp_ = new QPushButton(tr("Merge ICP"));
  connect(do_icp_, SIGNAL(clicked()), this, SLOT(icp_clicked ()));
  accept_result_ = new QPushButton(tr("Merge Accept"));
  connect(accept_result_, SIGNAL(clicked()), this, SLOT(accept_clicked ()));

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
  grid_layout->addWidget(do_icp_, 0, 0, 1, 1);
  grid_layout->addWidget(accept_result_, 0, 2, 1, 1);

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
  layout->addWidget(icp_groupBox);
  setLayout(layout);
  
}

void PoseGraphMergeEditorPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  clock_ = raw_node->get_clock();

}


void PoseGraphMergeEditorPanel::pubOperationCommand(std::string m_string){
  std_msgs::msg::String tmp_str;
  tmp_str.data = m_string;
  operation_command_pub_->publish(tmp_str);
}

void PoseGraphMergeEditorPanel::icp_clicked(){ pubOperationCommand(std::string("merge_icp")); }
void PoseGraphMergeEditorPanel::accept_clicked(){

  QFileDialog FileDialog;
  QString path=FileDialog.getExistingDirectory(this, tr("Select location to save merged files"),  "/", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);

  std::string dir = path.toStdString();

  RCLCPP_INFO(rclcpp::get_logger("pose_graph_merge_editor_panel"), "Select location: %s", dir.c_str());

  std::string complete_cmd = "merge_accept:" + dir;

  pubOperationCommand(complete_cmd);

}

void PoseGraphMergeEditorPanel::export_map_clicked(){
  pubOperationCommand(std::string("export_map")); 
  QMessageBox::information(this, "Message", "Maps are saved.", QMessageBox::Ok);
}

void PoseGraphMergeEditorPanel::pxIncrease(){ pubOperationCommand(std::string("merge_px+")); }
void PoseGraphMergeEditorPanel::pxDecrease(){ pubOperationCommand(std::string("merge_px-")); }
void PoseGraphMergeEditorPanel::pyIncrease(){ pubOperationCommand(std::string("merge_py+")); }
void PoseGraphMergeEditorPanel::pyDecrease(){ pubOperationCommand(std::string("merge_py-")); }
void PoseGraphMergeEditorPanel::pzIncrease(){ pubOperationCommand(std::string("merge_pz+")); }
void PoseGraphMergeEditorPanel::pzDecrease(){ pubOperationCommand(std::string("merge_pz-")); }

void PoseGraphMergeEditorPanel::rollIncrease(){ pubOperationCommand(std::string("merge_roll+")); }
void PoseGraphMergeEditorPanel::rollDecrease(){ pubOperationCommand(std::string("merge_roll-")); }
void PoseGraphMergeEditorPanel::pitchIncrease(){ pubOperationCommand(std::string("merge_pitch+")); }
void PoseGraphMergeEditorPanel::pitchDecrease(){ pubOperationCommand(std::string("merge_pitch-")); }
void PoseGraphMergeEditorPanel::yawIncrease(){ pubOperationCommand(std::string("merge_yaw+")); }
void PoseGraphMergeEditorPanel::yawDecrease(){ pubOperationCommand(std::string("merge_yaw-")); }

}  // namespace pose_graph_merge_editor_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pose_graph_merge_editor_panel::PoseGraphMergeEditorPanel, rviz_common::Panel)
