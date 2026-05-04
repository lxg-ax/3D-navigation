#include <QMainWindow>
#include "rviz_common/panel.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QRadioButton>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>

#include "rclcpp/rclcpp.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

namespace pose_graph_merge_editor_panel
{
  class PoseGraphMergeEditorPanel : public rviz_common::Panel // QMainWindow
{
  Q_OBJECT
  
  public:

    explicit PoseGraphMergeEditorPanel(QWidget *parent = nullptr);

    void onInitialize() override;

  private:
    
    QPushButton *px_increase_;
    QPushButton *px_decrease_;
    QPushButton *py_increase_;
    QPushButton *py_decrease_;
    QPushButton *pz_increase_;
    QPushButton *pz_decrease_;
    QPushButton *roll_increase_;
    QPushButton *roll_decrease_;
    QPushButton *pitch_increase_;
    QPushButton *pitch_decrease_;
    QPushButton *yaw_increase_;
    QPushButton *yaw_decrease_;
    
    QPushButton *do_icp_;
    QPushButton *accept_result_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_pose_graphs_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_focus_pg_pub_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<std::string> all_pose_graphs_;
    std::vector<QRadioButton*> radio_buttons_;

    void cpgHandler(const std_msgs::msg::String::SharedPtr msg);
    void pubOperationCommand(std::string m_string);
  // Here we declare some internal slots.
  protected Q_SLOTS:

    void mergePoseGraph();

    void pxIncrease();
    void pxDecrease();
    void pyIncrease();
    void pyDecrease();
    void pzIncrease();
    void pzDecrease();

    void rollIncrease();
    void rollDecrease();
    void pitchIncrease();
    void pitchDecrease();
    void yawIncrease();
    void yawDecrease();


    void icp_clicked();
    void accept_clicked();
    void last_step_clicked();
    void export_map_clicked();

};

} // namespace pose_graph_editor_panel
