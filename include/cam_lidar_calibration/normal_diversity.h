#pragma once
#include <cmath>
#include <opencv2/core.hpp>

namespace cam_lidar_calibration
{
// ACFR's normal-based initializer requires three independent normal directions.
inline bool hasNormalDiversity(const cv::Mat& normals, double minimum_ratio = 0.05)
{
  if (normals.rows < 3 || normals.cols != 3 || !cv::checkRange(normals))
    return false;
  cv::SVD decomposition(normals);
  const double ratio = decomposition.w.at<double>(2) / decomposition.w.at<double>(0);
  return std::isfinite(ratio) && ratio >= minimum_ratio;
}
}  // namespace cam_lidar_calibration
