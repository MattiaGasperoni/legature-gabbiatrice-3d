#pragma once

#include "pointcloud.h"  // per PointCloud
#include <Eigen/Dense>   // per Vector3d


double orthogonal_LSQ(const PointCloud& pc, Vector3d* a, Vector3d* b);

PointCloud getLineSteelBarsPointCloud(const PointCloud& cloud);

PointCloud getArcSteelBarsPointCloud(const PointCloud& cloud);
