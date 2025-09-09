#pragma once

// =======================================================
// Librerie standard C++
// =======================================================
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <cassert>
#include <algorithm>    
#include <limits>       

// =======================================================
// SDK / Librerie hardware specifiche
// =======================================================
#include <PointXYZ.h>

// =======================================================
// Header locali del progetto
// =======================================================
#include "BlobServerConfig.h"
#include "UdpParsing.h"
#include "exitcodes.h"
#include "Config.h"
#include "pointcloud.h"


// =======================================================
// Framework esterni (Open3D, OpenCV)
// =======================================================
// Disabilita warning di troncamento e conversione solo per Open3D
#pragma warning(push)
#pragma warning(disable : 4267) // size_t -> unsigned int
#pragma warning(disable : 4305) // double -> float
#include "open3d/Open3D.h"
#pragma warning(pop)
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Definizioni 
#define M_PI 3.141592653589793238462643383279502884L

inline double deg2rad(double d);

PointCloud cloudPlaneCut(
    PointCloud      cloud,
    const Vector3d& p0,
    const Vector3d& p1,
    const Vector3d& p2,
    bool            positive   = true,
    bool            getRemoved = false
);

inline Vector3d rotateVector(
    const Vector3d& v,
    double          roll_deg,
    double          pitch_deg,
    double          yaw_deg
);

inline std::array<Vector3d, 3> plane_points_from_anchor_and_euler(
    const Vector3d& anchor,
    double          roll_deg,
    double          pitch_deg,
    double          yaw_deg
);

PointCloud applyCutPlane(
    PointCloud      inputCloud,
    const Vector3d& origin,
    double          pitch,
    double          yaw,
    double          roll
);

// Converte la Point Cloud vector<PointXYZ> in PointCloud e rimuove i punti nan
PointCloud convertPointCloud(const std::vector<PointXYZ>& oldPointCloud);

// Proietta una nuvola di punti 3D su un piano definito da origine e normale,
// disegnando ogni punto proiettato come cerchio sull'immagine OpenCV.
// Calcola automaticamente i bounds dalla nuvola stessa.
void projectAndDrawPointCloud(
    const cv::Mat&         image, 
    const PointCloud&      cloud, 
    const Eigen::Vector3d& origin, 
    const Eigen::Vector3d& normal, 
    double                 scale, 
    int                    img_width, 
    int                    img_height,
    cv::Scalar             color = cv::Scalar(255, 255, 255)
);

// Funzione per processare point cloud e convertirla in immagine
cv::Mat processPointCloud(
    const std::vector<PointXYZ>& cloud,
    int img_width,
    int img_height,
    Eigen::Vector3d& origin,
    Eigen::Vector3d& normal,
    double scale,
    std::vector<Vector3d> originCutPlanes,
    std::vector<Vector3d> inclinationCutPlanes
);