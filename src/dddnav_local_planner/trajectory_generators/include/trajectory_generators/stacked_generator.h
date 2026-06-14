// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/trajectory_generator_theory.h>
/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins*/
#include <pluginlib/class_loader.hpp>

namespace trajectory_generators
{

class StackedGenerator{

  public:

    StackedGenerator(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                    std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer);
    ~StackedGenerator();

    /*Plugin operations*/
    void addPlugin(std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory> theory);

    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> getSharedDataPtr(){return shared_data_;}

    void initializeTheories_wi_Shared_data();
    bool hasMoreTrajectories(std::string pname);
    bool nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj);
    bool combineByScores(std::string pname,
                         std::vector<base_trajectory::Trajectory>& scored,
                         base_trajectory::Trajectory& combined);
 
    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex theory_mutex_t;
    theory_mutex_t* getMutex()
    {
      return access_;
    }

  private:

    std::map<std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory>> theories_;
    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data_;
    /*mutex*/
    theory_mutex_t* access_;
    
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

}//end of name space