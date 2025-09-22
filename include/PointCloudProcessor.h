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
#include <include/PointXYZ.h>

// =======================================================
// Header locali del progetto
// =======================================================
#include "include/BlobServerConfig.h"
#include "include/UdpParsing.h"
#include "include/exitcodes.h"
#include "include/Config.h"
#include "include/pointcloud.h"


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


// Funzione per convertire gli angoli in gradi in radianti
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

// Utilizza una seconda nuvola per calcolare i bounds,
// mantenendo proporzioni coerenti tra nuvole diverse.
// Permette di impostare raggio e spessore del punto.
void projectPointCloudBasedAnotherCloud(
    cv::Mat& image,
    const PointCloud& cloudToDraw,
    const PointCloud& cloudForBounds,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& normal,
    double scale,
    int img_width,
    int img_height,
    cv::Scalar color = cv::Scalar(255, 255, 255),
    int radiusPoint = 1,
    int thicknessPoint = -1
);


// Proietta una nuvola di punti 3D su un piano definito da origine e normale,
// disegnando ogni punto proiettato come cerchio sull'immagine OpenCV.
// Calcola automaticamente i bounds dalla nuvola stessa.
void projectPointCloud(
    const cv::Mat&         image, 
    const PointCloud&      cloud, 
    const Eigen::Vector3d& origin, 
    const Eigen::Vector3d& normal, 
    double                 scale, 
    int                    img_width, 
    int                    img_height,
    cv::Scalar             color = cv::Scalar(255, 255, 255)
);

// Filtra la Point Cloud rimuovendo i punti che non hanno un numero minimo di vicini 
// e che sono oltre una certa distanza dal centro della nuvola
// funzione poco efficiente O(n^2), valutere se usare KD-Tree o toglierla se non fondamentale
void filterPointCloud(
    PointCloud& pointCloud, 
    double      neighbor_radius, 
    int         min_neighbors, 
    double      max_distance
);

template <typename T>

void checkPointCloud(
    const std::vector<T>& cloud,
    const std::string&    message = "[Debug] Point Cloud points: "
);


// Funzione che restituisce la point cloud della testa legatrice e la salva su file
void show3dBinderPointCloud(
    PointCloud pointCloud,
    std::vector<Vector3d> originCutPlanes,
    std::vector<Vector3d> inclinationCutPlanes
);

// Funzione che fa avviare il taglio della pointcloud interattivo
cv::Mat start3dPointCloudCut(const std::vector<PointXYZ>& cloud, int img_width, int img_height, Eigen::Vector3d& origin, Eigen::Vector3d& normal, double scale);

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

