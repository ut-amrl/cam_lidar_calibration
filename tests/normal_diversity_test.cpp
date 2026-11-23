#include "cam_lidar_calibration/normal_diversity.h"
#include <iostream>
#include <limits>

int main()
{
  using cam_lidar_calibration::hasNormalDiversity;
  cv::Mat parallel = (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, 0, 1, 0, 0, 1);
  cv::Mat yaw_only = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0.866, 0.5, 0, 0.866, -0.5, 0);
  cv::Mat noise_only = (cv::Mat_<double>(3, 3) << 1, 0, 0.001, 0.866, 0.5, -0.001, 0.866, -0.5, 0.002);
  cv::Mat diverse = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0.866, 0.5, 0, 0.866, 0, 0.5);
  if (hasNormalDiversity(parallel) || hasNormalDiversity(yaw_only) || hasNormalDiversity(noise_only))
    return 1;
  if (!hasNormalDiversity(diverse) || hasNormalDiversity(cv::Mat::zeros(3, 3, CV_64F)))
    return 2;
  diverse.at<double>(0, 0) = std::numeric_limits<double>::quiet_NaN();
  if (hasNormalDiversity(diverse))
    return 3;
  std::cout << "Normal-diversity checks passed\n";
  return 0;
}
