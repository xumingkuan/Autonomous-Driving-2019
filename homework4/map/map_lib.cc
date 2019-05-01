// By xmk 2019.5.1

#include <boost/functional/hash.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include "homework4/map/map_lib.h"

namespace homework4 {
namespace map {

namespace {

class HashForPoint3D {
 public:
  std::size_t operator()(const interface::geometry::Point3D& point) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, point.x());
    boost::hash_combine(seed, point.y());
    boost::hash_combine(seed, point.z());
    return seed;
  }
};

class EqualToForPoint3D {
 public:
  bool operator()(const interface::geometry::Point3D& a,
                  const interface::geometry::Point3D& b) const {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }
};

}  // namespace

interface::geometry::Point2D MapLib::lane_point_2D(std::pair<int, int> id) const {
  const auto& point3D = map_data_.lane(id.first).central_line().point(id.second);
  interface::geometry::Point2D point2D;
  point2D.set_x(point3D.x());
  point2D.set_y(point3D.y());
  return point2D;
}

std::pair<int, int> MapLib::nearest_lane_point(const math::Vec2d& point) const {
  std::pair<int, int> nearest(0, 0);
  double nearest_distance_sqr = INFINITY;
  const int num_lane = map_data_.lane_size();
  for (int i = 0; i < num_lane; i++) {
    const auto& now_lane = map_data_.lane(i).central_line();
    const int num_point = now_lane.point_size();
    for (int j = 0; j < num_point; j++) {
      math::Vec2d now_point(now_lane.point(j));
      double now_distance_sqr = point.DistanceSqrToPoint(now_point);
      if(now_distance_sqr < nearest_distance_sqr) {
        nearest = std::make_pair(i, j);
        nearest_distance_sqr = now_distance_sqr;
      }
    }
  }
  return nearest;
}

int MapLib::initiate_successors() {
  const int num_lane = map_data_.lane_size();
  successors_.assign(num_lane, std::vector<int>());

  std::unordered_multimap<interface::geometry::Point3D, int, HashForPoint3D, EqualToForPoint3D>
      lane_start_point;

  for (int i = 0; i < num_lane; i++) {
    lane_start_point.insert(std::make_pair(map_data_.lane(i).central_line().point(0), i));
  }
  int num_successor_pairs = 0;
  for (int i = 0; i < num_lane; i++) {
    auto next_lane =
        lane_start_point.equal_range(*map_data_.lane(i).central_line().point().rbegin());
    for (auto it = next_lane.first; it != next_lane.second; it++) {
      map_data_.mutable_lane(i)->add_successor()->set_id(map_data_.lane(it->second).id().id());
      map_data_.mutable_lane(it->second)->add_predecessor()->set_id(map_data_.lane(i).id().id());
      successors_[i].emplace_back(it->second);
      num_successor_pairs++;
    }
  }
  return num_successor_pairs;
}

bool MapLib::find_route_points(const std::pair<int, int>& start_lane_point,
                               const std::pair<int, int>& end_lane_point,
                               interface::route::Route& route) {
  if (start_lane_point.first == end_lane_point.first &&
      start_lane_point.second <= end_lane_point.second) {
    // We can reach the destination without changing lanes.
    add_route_points(route, start_lane_point.first,
        start_lane_point.second, end_lane_point.second);
    return true;
  }

  if (successors_.empty()) {
    initiate_successors();
  }
  const int num_lane = map_data_.lane_size();
  std::vector<int> min_distance(num_lane, INT32_MAX);
  std::vector<int> prev_lane(num_lane);
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int> >,
      std::greater<std::pair<int, int> > > update_lane;
  for (int lane_id : successors_[start_lane_point.first]) {
    min_distance[lane_id] = map_data_.lane(start_lane_point.first).central_line().point_size()
        - start_lane_point.second - 1;
    prev_lane[lane_id] = -1;
    update_lane.push(std::make_pair(min_distance[lane_id], lane_id));
  }
  while (!update_lane.empty()) {
    // Dijkstra's Algorithm.
    int now_lane = update_lane.top().second;
    if (min_distance[now_lane] != update_lane.top().first) {
      update_lane.pop();
      continue;
    }
    update_lane.pop();
    if (min_distance[now_lane] > min_distance[end_lane_point.first]) {
      // An optimization for single-sink shortest path.
      continue;
    }
    int now_distance = min_distance[now_lane] +
        map_data_.lane(now_lane).central_line().point_size() - 1;
    for (int lane_id : successors_[now_lane]) {
      if (now_distance < min_distance[lane_id]) {
        min_distance[lane_id] = now_distance;
        prev_lane[lane_id] = now_lane;
        update_lane.push(std::make_pair(min_distance[lane_id], lane_id));
      }
    }
  }
  if (min_distance[end_lane_point.first] == INT32_MAX) {
    // Failed to find a route.
    return false;
  }

  // Get the route found by Dijkstra's Algorithm.
  std::vector<int> route_lane_list;
  route_lane_list.emplace_back(end_lane_point.first);
  do {
    route_lane_list.emplace_back(prev_lane[route_lane_list.back()]);
  } while (route_lane_list.back() != -1);

  add_route_points(route, start_lane_point.first, start_lane_point.second);
  for (int i = route_lane_list.size() - 2; i > 0; i--) {
    add_route_points(route, route_lane_list[i]);
  }
  add_route_points(route, end_lane_point.first, 1, end_lane_point.second);
  return true;
}

void MapLib::add_route_points(interface::route::Route& route,
                              int lane_id, int start_id, int end_id) const {
  if (end_id == -1) {
    end_id = map_data_.lane(lane_id).central_line().point_size() - 1;
  }
  for (int position = start_id; position <= end_id; position++) {
    route.add_route_point()->CopyFrom(lane_point_2D(std::make_pair(lane_id, position)));
  }
}

}  // namespace map
}  // namespace homework4
