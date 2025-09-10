#include "open3d/Open3D.h"
#include <algorithm>
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
#include <memory>


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

static CircleFit2D FitCircle2D(const std::vector<Eigen::Vector2d>& P) {

    CircleFit2D out;

    if (P.size() < 3) return out;

    Eigen::Vector2d c(0, 0);

    for (auto& p : P) c += p;

    c /= double(P.size());

    double Suu = 0, Svv = 0, Suv = 0, Suuu = 0, Svvv = 0, Suvv = 0, Svuu = 0;

    for (auto& p : P) {

        double u = p.x() - c.x();

        double v = p.y() - c.y();

        double uu = u * u, vv = v * v;

        Suu += uu; Svv += vv; Suv += u * v;

        Suuu += uu * u; Svvv += vv * v; Suvv += u * vv; Svuu += v * uu;

    }

    Eigen::Matrix2d M; M << Suu, Suv, Suv, Svv;

    Eigen::Vector2d b(0.5 * (Suuu + Suvv), 0.5 * (Svvv + Svuu));

    if (std::abs(M.determinant()) < 1e-12) return out;

    Eigen::Vector2d uv = M.ldlt().solve(b);

    out.cx = uv.x() + c.x();

    out.cy = uv.y() + c.y();

    double r = 0;

    for (auto& p : P) r += (p - Eigen::Vector2d(out.cx, out.cy)).norm();

    out.r = r / double(P.size());

    double mres = 0;

    for (auto& p : P) mres += std::abs((p - Eigen::Vector2d(out.cx, out.cy)).norm() - out.r);

    out.mean_abs_res = mres / double(P.size());

    out.ok = std::isfinite(out.cx) && std::isfinite(out.cy) && std::isfinite(out.r);

    return out;

}

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
static double LargestArcCoverage(const std::vector<Eigen::Vector2d>& P, double cx, double cy) {

    if (P.size() < 2) return 0.0;

    std::vector<double> ang; ang.reserve(P.size());

    for (auto& p : P) ang.push_back(std::atan2(p.y() - cy, p.x() - cx));

    std::sort(ang.begin(), ang.end());

    const double TWO_PI = 2.0 * M_PI;

    std::vector<double> ext(ang);

    for (double a : ang) ext.push_back(a + TWO_PI);

    double best = 0.0; size_t n = ang.size(); size_t j = 0;

    for (size_t i = 0; i < n; ++i) {

        while (j + 1 < i + n && ext[j + 1] - ext[i] <= TWO_PI) ++j;

        best = std::max(best, ext[j] - ext[i]);
    }
    return std::min(best, TWO_PI);

}

static PlaneFrame FitPlanePCA(const std::vector<Eigen::Vector3d>& X) {
    PlaneFrame F{};

    const size_t npts = X.size();
    if (npts < 3) {
        // Fallback: identity frame at origin
        F.origin.setZero();
        F.ex = Eigen::Vector3d::UnitX();
        F.ey = Eigen::Vector3d::UnitY();
        F.n = Eigen::Vector3d::UnitZ();
        return F;
    }

    // Centroid
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& p : X) centroid += p;
    centroid /= static_cast<double>(npts);

    // 3x3 covariance (fixed-size, stable)
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto& p : X) {
        const Eigen::Vector3d d = p - centroid;
        C.noalias() += d * d.transpose();
    }
    C *= 1.0 / static_cast<double>(npts);

    // Eigen decomposition (self-adjoint); eigenvalues are ascending
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    if (es.info() != Eigen::Success) {
        // Fallback if solver fails
        F.origin = centroid;
        F.ex = Eigen::Vector3d::UnitX();
        F.ey = Eigen::Vector3d::UnitY();
        F.n = Eigen::Vector3d::UnitZ();
        return F;
    }

    // Copy eigenvectors to a concrete Matrix3d to avoid const-block issues
    const Eigen::Matrix3d V = es.eigenvectors();
    Eigen::Vector3d normal = V.col(0); // smallest eigenvalue -> plane normal
    double len = normal.norm();
    if (len > 0.0) normal /= len; else normal = Eigen::Vector3d::UnitZ();

    // Build a stable in-plane basis
    const Eigen::Vector3d axis =
        (std::abs(normal.z()) < 0.9) ? Eigen::Vector3d::UnitZ()
        : Eigen::Vector3d::UnitX();
    Eigen::Vector3d ex = (axis - axis.dot(normal) * normal).normalized();
    Eigen::Vector3d ey = normal.cross(ex).normalized();

    F.origin = centroid;
    F.ex = ex;
    F.ey = ey;
    F.n = normal;
    return F;
}

static inline Eigen::Vector2d ProjectToPlane2D(const Eigen::Vector3d& P, const PlaneFrame& F) {

    Eigen::Vector3d d = P - F.origin;

    return { d.dot(F.ex), d.dot(F.ey) };

}

static inline Eigen::Vector3d LiftFromPlane2D(const Eigen::Vector2d& p, const PlaneFrame& F) {

    return F.origin + p.x() * F.ex + p.y() * F.ey;

}

//
// Funzioni Principali
//

std::vector<Line3D> getLineSteelBars(const PointCloud& cloud, PointCloud& remainingPoints)
{
    // Array delle linee trovate
    std::vector<Line3D> detectedLines;

    // Copia della point cloud originale
    PointCloud X = cloud;

    // Parametri
    double opt_minvotes = 0.1 * X.points.size();
    double opt_dx       = 0.0;
    int opt_nlines      = 0;
    int granularity     = 4;
    Vector3d minP, maxP, minPshifted, maxPshifted;

    // Calcolo estensioni point cloud
    X.getMinMax3D(&minP, &maxP);
    double d = (maxP - minP).norm();
    if (d == 0.0)
    {
        fprintf(stderr, "Error: all points in point cloud identical\n");
        // Popola remainingPoints con tutti i punti originali
        remainingPoints = cloud;
        return detectedLines;
    }
    X.getMinMax3D(&minPshifted, &maxPshifted);

    // dimensione Hough space
    if (opt_dx == 0.0)
        opt_dx = d / 64.0;

    std::unique_ptr<Hough> hough;
    try
    {
        hough = std::make_unique<Hough>(minPshifted, maxPshifted, opt_dx, granularity);
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Error: cannot allocate memory for Hough cells: %s\n", e.what());
        // Popola remainingPoints con tutti i punti originali
        remainingPoints = cloud;
        return detectedLines;
    }

    hough->add(X);

    // ciclo principale
    PointCloud Y;
    unsigned int nvotes;
    int nlines = 0;
    do
    {
        Vector3d a, b;  // punto di ancoraggio e direzione
        hough->subtract(Y);
        nvotes = hough->getLine(&a, &b);
        if (nvotes < (unsigned int)opt_minvotes) break;

        // Estrai punti vicini alla linea candidata
        X.pointsCloseToLine(a, b, opt_dx, &Y);
        double rc = orthogonal_LSQ(Y, &a, &b);
        if (rc == 0.0) break;

        // Salva la linea rilevata
        Line3D line;
        line.point     = a;
        line.direction = b;
        detectedLines.push_back(line);

        // Rimuovi punti utilizzati per evitare ri-rilevamenti
        X.removePoints(Y);
        nlines++;
    } 
    while ((X.points.size() > 1) && ((opt_nlines == 0) || (opt_nlines > nlines)));

    // Popola remainingPoints con i punti rimasti in X
    remainingPoints = X;

    return detectedLines;
}

std::vector<ArcPlane> getArcSteelBars(const PointCloud& pointCloud)
{
	//Convertiamo PointCloud in Open3D PointCloud
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    for (const auto& point : pointCloud.points) 
    {
        cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // Array delle semicirconferenze trovate
    std::vector<ArcPlane> detectedArcs;

    // Parametri per il DBSCAN
    double eps                = 55.0;  // distanza massima per considerare vicini i punti 15
    size_t min_points         = 5;     // numero minimo di punti vicini per formare un cluster
    size_t min_cluster_size   = 280;   // numero minimo di punti che un cluster deve avere
    double residual_tol       = 18.0;  // tolleranza residua media per il fit del cerchio
    double semicircle_tol_deg = 60.0;  // tolleranza angolare intorno ai 180° per semicirchio

    if (cloud->points_.empty())
        return detectedArcs;

    // DBSCAN
    std::vector<int> labels = cloud->ClusterDBSCAN(eps, min_points, false);

    int max_label = *std::max_element(labels.begin(), labels.end());
    
    if (max_label < 0) 
        return detectedArcs;

    // Conta punti per cluster
    std::unordered_map<int, int> cluster_counts;
    for (int lb : labels)
        if (lb >= 0) cluster_counts[lb]++;

    // Palette colori
    std::vector<Eigen::Vector3d> colors_palette = 
    {
        {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {1, 0, 1}, {0, 1, 1},
        {0.5, 0.5, 0.5}, {1, 0.5, 0}, {0.5, 0, 0.5}, {0, 0.5, 0.5},
        {0.3, 0.6, 0.9}, {0.9, 0.3, 0.6}, {0.2, 0.8, 0.2}, {0.7, 0.7, 0.1},
        {0.6, 0.3, 0.9}, {0.9, 0.6, 0.1}
    };

    // Raggruppa indici dei cluster
    std::vector<std::vector<int>> clusters(max_label + 1);
    for (size_t i = 0; i < labels.size(); ++i)
    { 
        if (labels[i] >= 0) clusters[labels[i]].push_back((int)i);
    }

    // Nuova point cloud solo con cluster grandi
    std::shared_ptr<open3d::geometry::PointCloud> pcd_filtered = std::make_shared<open3d::geometry::PointCloud>();

    for (const auto& pair : cluster_counts)
    {
        int cid = pair.first;
        int cluster_size = pair.second;
        if (cluster_size < (int)min_cluster_size) continue; // skip cluster piccoli

        auto& idxs = clusters[cid];
        std::vector<Eigen::Vector3d> P3; P3.reserve(idxs.size());
        for (int i : idxs) 
        {
            P3.push_back(cloud->points_[i]);
            pcd_filtered->points_.push_back(cloud->points_[i]);
            pcd_filtered->colors_.push_back(colors_palette[cid % colors_palette.size()]);
        }

        PlaneFrame F = FitPlanePCA(P3);

        std::vector<Eigen::Vector2d> P2; P2.reserve(P3.size());
        for (auto& p : P3) P2.push_back(ProjectToPlane2D(p, F));

        CircleFit2D cf = FitCircle2D(P2);
        if (!cf.ok || cf.r <= 0) {
            continue;
        }

        double cov               = LargestArcCoverage(P2, cf.cx, cf.cy);
        const double tol         = semicircle_tol_deg * M_PI / 180.0;
        const double target_semi = M_PI;
        const double target_full = 2 * M_PI;

        //std::cout << "Cluster " << cid << ": size=" << cluster_size
        //	<< ", r=" << cf.r
        //	<< ", coverage=" << cov * 180.0 / M_PI << " deg"
        //	<< ", mean_residual=" << cf.mean_abs_res;

        bool accept = (std::abs(cov - target_semi) <= tol) || (std::abs(cov - target_full) <= tol);

        if (accept && cf.mean_abs_res <= residual_tol)
        {
            ArcPlane det;
            det.cluster_id   = cid;
            det.center3D     = LiftFromPlane2D(Eigen::Vector2d(cf.cx, cf.cy), F);
            det.normal       = F.n;
            det.radius       = cf.r;
            det.coverage_rad = cov;
            det.residual_px  = cf.mean_abs_res;
            detectedArcs.push_back(det);

            //std::cout << " -> Accepted as semicircle, color RGB=("
            //	<< colors_palette[cid % colors_palette.size()][0] << ", "
            //	<< colors_palette[cid % colors_palette.size()][1] << ", "
            //	<< colors_palette[cid % colors_palette.size()][2] << ")"
            //	<< std::endl;
        }
    }

    // Sostituisci la point cloud originale con quella filtrata
    *cloud = *pcd_filtered;
   
    return detectedArcs;
}

std::vector<Vector3d> getIntersectionPoints(const std::vector<Line3D> lines, const std::vector<ArcPlane> arcs)
{
	// Array delle intersezioni trovate
    std::vector<Vector3d> intersections;

    std::vector<Vector3d> planeNormals;
    std::vector<double>   planeDs;

    // Estrai parametri dei piani
    for (const auto& arc : arcs) 
    {
        Vector3d normal(arc.normal.x(), arc.normal.y(), arc.normal.z());

        double d = -(normal.x * arc.center3D.x() +
            normal.y * arc.center3D.y() +
            normal.z * arc.center3D.z());

        planeNormals.push_back(normal);
        planeDs.push_back(d);
    }

    // Calcola intersezioni
    for (const auto& line : lines)
    {
        for (size_t i = 0; i < planeNormals.size(); ++i) 
        {
            const Vector3d& n = planeNormals[i];
            double d = planeDs[i];

            double denom = n.x * line.direction.x + n.y * line.direction.y + n.z * line.direction.z;

            if (std::abs(denom) < 1e-8) 
            {
                continue;
            }

            double t = -(n.x * line.point.x + n.y * line.point.y + n.z * line.point.z + d) / denom;

            Vector3d intersection = line.point + line.direction * t;

            intersections.push_back(intersection);
        }
    }

    return intersections;
}