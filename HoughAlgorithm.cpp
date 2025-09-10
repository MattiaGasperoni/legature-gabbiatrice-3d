#include "HoughAlgorithm.h"
#include "pointcloud.h"
#include <Eigen/Dense>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include "hough.h"
#include "Matrix.h"

//
// Funzioni helper
//

double orthogonal_LSQ(const PointCloud& pc, Vector3d* a, Vector3d* b)
{
    double rc = 0.0;

    // anchor point is mean value
    *a = pc.meanValue();

    // copy points to libeigen matrix
    Eigen::MatrixXf points = Eigen::MatrixXf::Constant(pc.points.size(), 3, 0);
    for (int i = 0; i < points.rows(); i++) {
        points(i, 0) = pc.points.at(i).x;
        points(i, 1) = pc.points.at(i).y;
        points(i, 2) = pc.points.at(i).z;
    }

    // compute scatter matrix ...
    Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
    Eigen::MatrixXf scatter = (centered.adjoint() * centered);

    // ... and its eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatter);
    Eigen::MatrixXf eigvecs = eig.eigenvectors();

    // we need eigenvector to largest eigenvalue
    // libeigen yields it as LAST column
    b->x = eigvecs(0, 2); b->y = eigvecs(1, 2); b->z = eigvecs(2, 2);
    rc = eig.eigenvalues()(2);

    return (rc);
}

//
// Funzioni Principali
//

PointCloud getLineSteelBarsPointCloud(PointCloud cloud)
{
    PointCloud linesPointCloud;

    // Copia della point cloud originale
    PointCloud X = cloud;

    // Parametri
    double opt_minvotes = 0.1 * X.points.size();
    double opt_dx       = 0.0;                     
    int opt_verbose     = 1;
    int opt_nlines      = 0;                     
    int granularity     = 100;   
    int opt_outformat   = 0;  
    Vector3d minP, maxP, minPshifted, maxPshifted;

    // Calcolo estensioni point cloud
    X.getMinMax3D(&minP, &maxP);
    double d = (maxP - minP).norm();
    if (d == 0.0)
    {
        fprintf(stderr, "Error: all points in point cloud identical\n");
        return linesPointCloud;
    }
    X.getMinMax3D(&minPshifted, &maxPshifted);

    // dimensione Hough space
    if (opt_dx == 0.0)
        opt_dx = d / 64.0;

    // allocazione Hough
    Hough* hough = nullptr;
    try 
    {
        hough = new Hough(minPshifted, maxPshifted, opt_dx, granularity);
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Error: cannot allocate memory for Hough cells: %s\n", e.what());
        return linesPointCloud;
    }

    hough->add(X);

    // ciclo principale
    PointCloud Y;
    unsigned int nvotes;
    int nlines = 0;

    do
    {
        Vector3d a, b;  // punto ancoraggio e direzione
        hough->subtract(Y);
        nvotes = hough->getLine(&a, &b);

        if (nvotes < (unsigned int)opt_minvotes) break;

        X.pointsCloseToLine(a, b, opt_dx, &Y);

        double rc = orthogonal_LSQ(Y, &a, &b);
        if (rc == 0.0) break;

        nlines++;

        // accumula punti linee
        for (const auto& pt : Y.points)
            linesPointCloud.points.push_back(pt);

        // rimuovi punti trovati
        X.removePoints(Y);

    } while ((X.points.size() > 1) && ((opt_nlines == 0) || (opt_nlines > nlines)));

    delete hough;
    return linesPointCloud;
}


PointCloud getArcSteelBarsPointCloud(const PointCloud cloud)
{
	PointCloud arcPointCloud;



	return arcPointCloud;
}
