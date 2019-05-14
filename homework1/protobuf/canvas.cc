// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <iostream>
#include <glog/logging.h>

namespace homework1 {

using homework1::geometry::Point3D;

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialization;
  CHECK(polygon_.SerializeToString(&serialization)) << "Canvas serialization failed.";
  return serialization;
}

const geometry::Polyline Canvas::BuildPolyline() const {
  geometry::Polyline polyline;
  for (const auto& p : polygon_.point()) {
    auto* point = polyline.add_point();
    point->CopyFrom(p);
  }
  return polyline;
}

namespace {
double Sqr(double x) {
  return x * x;
}
}

double GetDistance(const Point3D& p1, const Point3D& p2) {
  return std::sqrt(Sqr(p1.x() - p2.x()) + Sqr(p1.y() - p2.y()) + Sqr(p1.z() - p2.z()));
}

double GetLength(const geometry::Polyline& polyline) {
  double len = 0;
  for (int i = 1; i < polyline.point_size(); i++) {
    len += GetDistance(polyline.point(i - 1), polyline.point(i));
  }
  return len;
}

}  // namespace homework1
