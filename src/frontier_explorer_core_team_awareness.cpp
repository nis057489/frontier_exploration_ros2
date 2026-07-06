/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::teamMapCallback(const OccupancyGrid2d & map_msg)
{
  // No preemption/cost-status interplay is needed here (unlike costmapCallback): this is a
  // plain data refresh consumed the next time frontiers are filtered, which already happens
  // on this robot's own map cadence.
  team_map = map_msg;
}

bool FrontierExplorerCore::goal_point_known_to_team(
  const std::pair<double, double> & goal_point) const
{
  if (!team_map.has_value()) {
    return false;
  }

  const double radius = std::max(0.0, params.team_known_check_radius_m);
  const double resolution = team_map->map().info.resolution;
  const int cell_radius = resolution > 0.0 ?
    static_cast<int>(std::ceil(radius / resolution)) : 0;

  int center_mx = 0;
  int center_my = 0;
  if (!team_map->worldToMapNoThrow(goal_point.first, goal_point.second, center_mx, center_my)) {
    return false;
  }

  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      const int mx = center_mx + dx;
      const int my = center_my + dy;
      if (mx < 0 || my < 0 ||
        static_cast<unsigned int>(mx) >= team_map->map().info.width ||
        static_cast<unsigned int>(my) >= team_map->map().info.height)
      {
        continue;
      }
      if (team_map->getCost(mx, my) != static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
        // The team already resolved this cell (free or occupied) -- no need to send a robot
        // here just to re-derive information the team already shared.
        return true;
      }
    }
  }
  return false;
}

bool FrontierExplorerCore::goal_point_near_any_peer(
  const std::pair<double, double> & goal_point) const
{
  if (params.peer_avoidance_radius_m <= 0.0 || !callbacks.get_peer_positions) {
    return false;
  }

  const double radius_sq = params.peer_avoidance_radius_m * params.peer_avoidance_radius_m;
  for (const auto & peer_position : callbacks.get_peer_positions()) {
    const double dx = goal_point.first - peer_position.first;
    const double dy = goal_point.second - peer_position.second;
    if ((dx * dx) + (dy * dy) <= radius_sq) {
      return true;
    }
  }
  return false;
}

FrontierSequence FrontierExplorerCore::filter_frontiers_for_team_awareness(
  const FrontierSequence & frontiers) const
{
  if (!params.team_awareness_enabled) {
    return frontiers;
  }
  if (!team_map.has_value() && (!callbacks.get_peer_positions || params.peer_avoidance_radius_m <= 0.0)) {
    // Fast path: nothing to check against yet (no team map received, no peer data wired up).
    return frontiers;
  }

  FrontierSequence filtered;
  filtered.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    const auto goal_point = frontier_position(frontier);
    if (goal_point_known_to_team(goal_point) || goal_point_near_any_peer(goal_point)) {
      continue;
    }
    filtered.push_back(frontier);
  }
  return filtered;
}

}  // namespace frontier_exploration_ros2
