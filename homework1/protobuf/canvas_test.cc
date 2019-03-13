// Copyright @2018 Pony AI Inc. All rights reserved.
//
#include <gtest/gtest.h>
#include "homework1/protobuf/canvas.h"

namespace homework1 {

TEST(Canvas, Serialization) {
  Canvas canvas;
  canvas.AddPoint(1.0, 2.0, 3.0);

  const std::string serialization = canvas.SerializeToString();

  Canvas other_canvas;
  other_canvas.ParseFromString(serialization);
  ASSERT_EQ(1, other_canvas.point_size());
  EXPECT_NEAR(1.0, other_canvas.GetPoint(0).x(), 1e-6);
  EXPECT_NEAR(2.0, other_canvas.GetPoint(0).y(), 1e-6);
  EXPECT_NEAR(3.0, other_canvas.GetPoint(0).z(), 1e-6);
}

TEST(Canvas, PolylineLength) {
  Canvas canvas;
  geometry::Polyline polyline;

  canvas.AddPoint(1.0, 2.0, 3.0);
  polyline = canvas.BuildPolyline();
  const double polyline_length1 = GetLength(polyline);
  EXPECT_NEAR(0, polyline_length1, 1e-6);

  canvas.AddPoint(1.0, 2.0, 3.0);
  polyline = canvas.BuildPolyline();
  const double polyline_length2 = GetLength(polyline);
  EXPECT_NEAR(0, polyline_length2, 1e-6);

  canvas.AddPoint(2.0, 3.0, 3.0);
  polyline = canvas.BuildPolyline();
  const double polyline_length3 = GetLength(polyline);
  EXPECT_NEAR(1.414213562, polyline_length3, 1e-6);

  canvas.AddPoint(3.0, 0.0, 1.0);
  polyline = canvas.BuildPolyline();
  const double polyline_length4 = GetLength(polyline);
  EXPECT_NEAR(5.155870949, polyline_length4, 1e-6);
}
}  // namespace homework1
