// Copyright @2018 Pony AI Inc. All rights reserved.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

Mat distort(const Mat& image, double k1, double k2) {
  Mat ret(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));
  double cx = (image.rows - 1) / 2.0;
  double cy = (image.cols - 1) / 2.0;
  double fx = 800;// focus length undefined, choose arbitrarily
  double fy = 800;
  double newfx = 600;
  double newfy = 600;
  for (int i = 0; i < image.rows; i++) {
    for (int j = 0; j < image.cols; j++) {
      double x = (i - cx) / fx;
      double y = (j - cy) / fy;
      double r2 = x * x + y * y;
      double newx = newfx * x * (1 + k1 * r2 + k2 * r2 * r2) + cx;
      double newy = newfy * y * (1 + k1 * r2 + k2 * r2 * r2) + cy;
      int newi = static_cast<int>(lround(newx));
      int newj = static_cast<int>(lround(newy));
      if (newi >= 0 && newi < ret.rows && newj >= 0 && newj < ret.cols) {
        ret.at<Vec3b>(newi, newj) = image.at<Vec3b>(i, j);
      }
    }
  }
  return ret;
}

int main() {
  cv::Mat image, result;
  // ATTENTION!!! : please use absolute path for reading the data file.
  image = imread("/home/xmk/ponyai/PublicCourse/homework2/chessboard/chessboard_undistorted.png",
                 CV_LOAD_IMAGE_COLOR);
  result = distort(image, 0.1, 0.1);
  namedWindow("chessboard_undistorted");
  imshow("chessboard_undistorted", image);
  namedWindow("chessboard_distorted");
  imshow("chessboard_distorted", result);
  imwrite("/home/xmk/ponyai/PublicCourse/homework2/chessboard/chessboard_distorted.png", result);
  waitKey(0);
  return 0;
}
