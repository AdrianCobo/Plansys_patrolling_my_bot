// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "cv_bridge/cv_bridge.h"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING), new_state_(PATROL_WP1)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
    cv::namedWindow("control panel", cv::WINDOW_AUTOSIZE);
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));
  }

  int read_key_pressed()
  {
    int key_pressed;
    StateType new_state;
    // wait new image format during 100ms
    //cv::imshow("R2D2 control panel", null);
    key_pressed = cv::waitKey(100);

    if(key_pressed == -1){
      return -1;
    }

    // change the image format if it is necessary
    
    switch (key_pressed) {
      case '0':
        new_state = STARTING;
        break;
      case '1':
        new_state = PATROL_WP1;
        break;
      case '2':
        new_state = PATROL_WP2;
        break;
      case '3':
        new_state = PATROL_WP3;
        break;
      case '4':
        new_state = PATROL_WP4;
        break;
    }
    std::cout << new_state << std::endl;
    return new_state;
  }

  void step()
  {
    int key = read_key_pressed();
    if(key!=-1){
      new_state_ = (StateType)key;
    }
    std::cout << "state: " << state_ << std::endl;
    std::cout << "state: " << new_state_ << std::endl;

    switch (state_) {
      case STARTING:
        {
          // Set the goal for next state
          goal_wp_ = "(patrolled wp1)";
          std::string goal = "(and" + goal_wp_ + ")";
          problem_expert_->setGoal(plansys2::Goal(goal));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = PATROL_WP1;
          }
        }
        break;    

      case PATROL_WP1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate(goal_wp_));

              // Set the goal for next state
              goal_wp_ = "(patrolled wp1)";
              std::string goal = "(and" + goal_wp_ + ")";
              problem_expert_->setGoal(plansys2::Goal(goal));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = new_state_;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_WP2:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate(goal_wp_));

              // Set the goal for next state
              goal_wp_ = "(patrolled wp2)";
              std::string goal = "(and" + goal_wp_ + ")";
              problem_expert_->setGoal(plansys2::Goal(goal));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = new_state_;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_WP3:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate(goal_wp_));

              // Set the goal for next state
              goal_wp_ = "(patrolled wp3)";
              std::string goal = "(and" + goal_wp_ + ")";
              problem_expert_->setGoal(plansys2::Goal(goal));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = new_state_;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case PATROL_WP4:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate(goal_wp_));

              // Set the goal for next state
              goal_wp_ = "(patrolled wp4)";
              std::string goal = "(and" + goal_wp_ + ")";
              problem_expert_->setGoal(plansys2::Goal(goal));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = new_state_;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, PATROL_WP1, PATROL_WP2, PATROL_WP3, PATROL_WP4} StateType;
  StateType state_, new_state_;
  std::string goal_wp_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(20);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
