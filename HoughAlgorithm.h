#pragma once
#include "open3d/Open3D.h"

#include "pointcloud.h"  // per PointCloud
#include <Eigen/Dense>   // per Vector3d
#include <algorithm>

// Definizioni 
#define M_PI 3.141592653589793238462643383279502884L

struct Line3D
{
    Vector3d point;      // punto sulla retta
    Vector3d direction;  // direzione della retta
};

struct ArcPlane
{
	int cluster_id;
	Eigen::Vector3d center3D; // circle center in 3D
	Eigen::Vector3d normal;   // plane normal (unit)
	double radius;
	double coverage_rad;      // radians of arc covered by samples
	double residual_px;       // mean abs radial residual in the plane
};

struct CircleFit2D
{
	double cx{ 0 }, cy{ 0 }, r{ 0 }, mean_abs_res{ 0 }; bool ok{ false };
};

struct PlaneFrame
{
	// ---------- Plane fit (PCA) & projection 3D -> 2D ----------

	Eigen::Vector3d origin; // centroid
	Eigen::Vector3d ex, ey; // orthonormal in-plane basis
	Eigen::Vector3d n;      // unit normal (ex x ey)
};

std::vector<Line3D> getLineSteelBars(const PointCloud& cloud, PointCloud& remainingPoints);

std::vector<ArcPlane> getArcSteelBars(const PointCloud& cloud);

std::vector<Vector3d> getIntersectionPoints(const std::vector<Line3D>lines, const std::vector<ArcPlane> arcs);