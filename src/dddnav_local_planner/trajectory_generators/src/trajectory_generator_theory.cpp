/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <trajectory_generators/trajectory_generator_theory.h>

#include <limits>

namespace trajectory_generators
{

TrajectoryGeneratorTheory::TrajectoryGeneratorTheory(){

}

void TrajectoryGeneratorTheory::initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node){
  name_ = name;
  node_ = weak_node.lock();
  onInitialize();
}

void TrajectoryGeneratorTheory::setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data){
  shared_data_ = shared_data;
}

bool TrajectoryGeneratorTheory::combineByScores(
    std::vector<base_trajectory::Trajectory>& scored,
    base_trajectory::Trajectory& combined){
  combined.cost_ = -1.0;
  double min_cost = std::numeric_limits<double>::max();
  bool found = false;
  for (auto& t : scored){
    if (t.cost_ >= 0.0 && t.cost_ <= min_cost){
      combined = t;
      min_cost = t.cost_;
      found = true;
    }
  }
  return found;
}


}//end of name space