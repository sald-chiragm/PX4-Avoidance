#include "safe_landing_planner/waypoint_generator.hpp"

#include "avoidance/common.h"

#include <ros/console.h>

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

using avoidance::SLPState;
std::string toString(SLPState state) {
  std::string state_str = "unknown";
  switch (state) {
    // case SLPState::GOTO:
    //   state_str = "GOTO";
    //   break;
    // case SLPState::ALTITUDE_CHANGE:
    //   state_str = "ALTITUDE CHANGE";
    //   break;
    // case SLPState::LOITER:
    //   state_str = "LOITER";
    //   break;
    // case SLPState::LAND:
    //   state_str = "LAND";
    //   break;
    case SLPState::EVALUATE_GRID:
      state_str = "EVALUATE_GRID";
      break;
    case SLPState::GOTO_LAND:
      state_str = "GOTO_LAND";
      break;
  }
  return state_str;
}

WaypointGenerator::WaypointGenerator(): usm::StateMachine<SLPState>(SLPState::EVALUATE_GRID), publishTrajectorySetpoints_([](const Eigen::Vector3f &, const Eigen::Vector3f &, float, float) 
  {
    ROS_ERROR("publishTrajectorySetpoints_ not set in WaypointGenerator");
  }) {
  initializeMask();
}

void WaypointGenerator::initializeMask() {
  for (int i = 0; i < mask_.rows(); i++) {
    for (int j = 0; j < mask_.cols(); j++) {
      mask_(i, j) = std::hypot(i - smoothing_land_cell_, j - smoothing_land_cell_) < (smoothing_land_cell_ + 0.5f);
    }
  }
}

void WaypointGenerator::calculateWaypoint() {
  updateSLPState();
  iterateOnce();

  if (getState() != prev_slp_state_) {
    std::string state_str = toString(getState());
    ROS_INFO("\033[1;36m [WGN] Update to %s state \033[0m", state_str.c_str());
  }
}

void WaypointGenerator::updateSLPState() {
  if (update_smoothing_size_ || mask_.rows() != (smoothing_land_cell_ * 2 + 1)) {
    mask_.resize((smoothing_land_cell_ * 2 + 1), (smoothing_land_cell_ * 2 + 1));

    initializeMask();
    update_smoothing_size_ = false;
  }

  if (grid_slp_.land_.rows() != can_land_hysteresis_matrix_.rows()) {
    can_land_hysteresis_matrix_.resize(grid_slp_.land_.rows(), grid_slp_.land_.cols());
    can_land_hysteresis_matrix_.fill(0.0f);
    can_land_hysteresis_result_.resize(grid_slp_.land_.rows(), grid_slp_.land_.cols());
    can_land_hysteresis_result_.fill(0);
  }

  if (!is_land_waypoint_) {
    decision_taken_ = false;
    can_land_ = true;
    can_land_hysteresis_matrix_.fill(0.0f);
    explorarion_is_active_ = false;
    n_explored_pattern_ = -1;
    factor_exploration_ = 1.f;
    landing_radius_ = 2.0f;
    ROS_INFO("[WGN] Not a land waypoint");
  }

  return;
}

SLPState WaypointGenerator::chooseNextState(SLPState currentState, usm::Transition transition) {
  prev_slp_state_ = currentState;
  state_changed_ = true;
  // clang-format off
  USM_TABLE(
      currentState, SLPState::EVALUATE_GRID,
      USM_STATE(transition, SLPState::EVALUATE_GRID, USM_MAP(usm::Transition::NEXT1, SLPState::GOTO_LAND));
      USM_STATE(transition, SLPState::GOTO_LAND, USM_MAP(usm::Transition::NEXT1, SLPState::EVALUATE_GRID));
      );
  // clang-format on
}

usm::Transition WaypointGenerator::runCurrentState() {
  if (trigger_reset_) {
    trigger_reset_ = false;
    return usm::Transition::ERROR;
    ROS_INFO("[ERROR] usm::Transition::ERROR;");
  }

  usm::Transition t;
  switch (getState()) {
    case SLPState::EVALUATE_GRID:
      ROS_INFO("[STATE   ]   EVALUATE_GRID");
      t = runEvaluateGrid();
      break;

    case SLPState::GOTO_LAND:
      ROS_INFO("[STATE   ]   GOTO_LAND");
      t = runGoToLand();
      break;
  }
  state_changed_ = false;
  return t;
}


usm::Transition WaypointGenerator::runGoToLand() {
  if (withinLandingRadius()) {
    within_landing_radius_ = true;
    usm::Transition::REPEAT;
  } else{
    usm::Transition::NEXT1;
  }

  return usm::Transition::NEXT1;
}

usm::Transition WaypointGenerator::runEvaluateGrid() {
  ROS_INFO("\033[1;31m [WGN] runEvaluateGrid \033[0m\n");
  publishTrajectorySetpoints_(loiter_position_, nan_setpoint, loiter_yaw_, NAN);
  ROS_INFO("\033[1;31m [WGN] runEvaluateGrid %f %f %f - nan nan nan yaw %f \033[0m\n", loiter_position_.x(),
           loiter_position_.y(), loiter_position_.z(), loiter_yaw_);

  landing_radius_ = 0.5f;
  Eigen::Vector2i center = Eigen::Vector2i(grid_slp_.land_.rows() / 2, grid_slp_.land_.cols() / 2);
  Eigen::Vector2i offset = Eigen::Vector2i(grid_slp_.land_.rows() / 2, grid_slp_.land_.cols() / 2);

  Eigen::Vector2i left_upper_corner =
      Eigen::Vector2i(center.x() - smoothing_land_cell_, center.y() - smoothing_land_cell_);
  
  std::cout << "left_upper_corner   " << left_upper_corner << std::endl;
  
  can_land_ = evaluatePatch(left_upper_corner);
  if (can_land_) {
    decision_taken_ = true;
    return usm::Transition::NEXT1;  // GOTO_LAND
    std::cout << "decision_taken_   " << decision_taken_ << std::endl;
  }

  int n_iterations = (1 + (grid_slp_.land_.rows() - (2 * smoothing_land_cell_ + 1)) / stride_) / 2;
  for (int i = 1; i < n_iterations; i++) {
    for (int j = 0; j < exploration_pattern.size(); j++) {
      offset.x() = center.x() + exploration_pattern[j].x() * i * stride_ - smoothing_land_cell_;
      offset.y() = center.y() + exploration_pattern[j].y() * i * stride_ - smoothing_land_cell_;
      can_land_ = evaluatePatch(offset);

      if (can_land_) {
        decision_taken_ = true;
        Eigen::Vector2f min, max;

        grid_slp_.getGridLimits(min, max);
        goal_ = Eigen::Vector3f(
            position_.x() + (offset.x() + smoothing_land_cell_ - grid_slp_.land_.rows() / 2) * grid_slp_.getCellSize(),
            position_.y() + (offset.y() + smoothing_land_cell_ - grid_slp_.land_.cols() / 2) * grid_slp_.getCellSize(),
            position_.z());

        velocity_setpoint_.z() = NAN;
        ROS_INFO("\033[1;31m [WGN] Found landing area in grid at %f %f %f \033[0m", goal_.x(), goal_.y(), goal_.z());
        return usm::Transition::NEXT1;  // GOTO_LAND
      }
    }
  }
  decision_taken_ = true;

  return usm::Transition::REPEAT;  // GOTO
}

bool WaypointGenerator::evaluatePatch(Eigen::Vector2i &left_upper_corner) {
  return can_land_hysteresis_result_.block(left_upper_corner.x(), left_upper_corner.y(), mask_.rows(), mask_.cols())
             .cwiseProduct(mask_)
             .sum() == mask_.sum();
}

bool WaypointGenerator::withinLandingRadius() {
  return (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
}

bool WaypointGenerator::inVerticalRange() {
  return std::abs(std::abs(position_.z() - altitude_landing_area_percentile_) - loiter_height_) < vertical_range_error_;
}


float WaypointGenerator::landingAreaHeightPercentile(float percentile) {
  std::vector<float> altitude_landing_area((smoothing_land_cell_ * 2 + 1) * (smoothing_land_cell_ * 2 + 1));

  int offset_center = grid_slp_.land_.rows() / 2;
  for (int i = offset_center - smoothing_land_cell_; i <= offset_center + smoothing_land_cell_; i++) {
    for (int j = offset_center - smoothing_land_cell_; j <= offset_center + smoothing_land_cell_; j++) {
      int index = (smoothing_land_cell_ * 2 + 1) * (i - offset_center + smoothing_land_cell_) +
                  (j - offset_center + smoothing_land_cell_);
      altitude_landing_area[index] = grid_slp_.mean_(i, j);
    }
  }

  std::sort(altitude_landing_area.begin(), altitude_landing_area.end());
  int index = static_cast<int>(std::round(percentile / 100.f * altitude_landing_area.size()));
  return altitude_landing_area[index];
}
}
