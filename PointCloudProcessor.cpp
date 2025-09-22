//
//   Inclusione di librerie
//
#include <cstdlib>
#include <cmath>
#include <array>

#include "open3d/Open3D.h"

#include "PointCloudProcessor.h"
#include "pointcloud.h"
#include "vector3d.h"
#include "HoughAlgorithm.h"


inline double deg2rad(double d)
{
	return d * M_PI / 180.0;
}

//-----------------------------------------------------------------------------------------------------------------------
//          Funzioni per tagliare i punti che non ci interessano utilizzando dei piani di taglio
//-----------------------------------------------------------------------------------------------------------------------

PointCloud cloudPlaneCut(PointCloud cloud, const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, bool positive, bool getRemoved)
{
	// Build the plane normal
	Vector3d v1 = p1 - p0;
	Vector3d v2 = p2 - p0;

	Vector3d normal = v1.cross(v2);
	double  nLen = normal.norm();

	// Degenerate triangle, just return the cloud unchanged
	if (nLen == 0.0) {
		return cloud;
	}

	// Normalise so we get a signed distance (not just projection)
	normal = normal / nLen;

	// Gather the bad points (those on the negative side)
	PointCloud badPoints;
	for (const auto& pt : cloud.points) {
		// Signed distance:  n dot (pt – p0)
		double signedDist = normal * (pt - p0);
		if (!positive) signedDist *= -1;

		// Change the comparison if you want the other side.
		if (signedDist < 0.0) {
			badPoints.points.push_back(pt);
		}
	}

	if (!getRemoved) {
		// Remove the bad points – removePoints() keeps the same order, so we inserted them in the original order.
		cloud.removePoints(badPoints);
		// Return the trimmed cloud
		return cloud;
	}
	else {
		// Return the removed cloud
		return badPoints;
	}
}

inline Vector3d rotateVector(const Vector3d& v, double roll_deg, double pitch_deg, double yaw_deg)
{
	double roll = deg2rad(roll_deg);
	double pitch = deg2rad(pitch_deg);
	double yaw = deg2rad(yaw_deg);

	double cr = std::cos(roll), sr = std::sin(roll);
	double cp = std::cos(pitch), sp = std::sin(pitch);
	double cy = std::cos(yaw), sy = std::sin(yaw);

	// R = Rz(yaw) * Ry(pitch) * Rx(roll)
	double r11 = cy * cp;
	double r12 = cy * sp * sr - sy * cr;
	double r13 = cy * sp * cr + sy * sr;

	double r21 = sy * cp;
	double r22 = sy * sp * sr + cy * cr;
	double r23 = sy * sp * cr - cy * sr;

	double r31 = -sp;
	double r32 = cp * sr;
	double r33 = cp * cr;

	return Vector3d(
		r11 * v.x + r12 * v.y + r13 * v.z,
		r21 * v.x + r22 * v.y + r23 * v.z,
		r31 * v.x + r32 * v.y + r33 * v.z
	);
}

inline std::array<Vector3d, 3> plane_points_from_anchor_and_euler(const Vector3d& anchor, double roll_deg, double pitch_deg, double yaw_deg)
{
	// normal of the plane
	Vector3d n = rotateVector(Vector3d(0, 0, 1), roll_deg, pitch_deg, yaw_deg);
	n = n / n.norm();

	// 2)  Two independent directions that lie in the plane

	// Pick a vector that is not parallel to n
	Vector3d arbitrary(1.0, 0.0, 0.0);
	if (std::abs(n.x) > 0.99)  // n close to X, use (0,1,0)
		arbitrary = Vector3d(0.0, 1.0, 0.0);

	// First direction = cross(n, arbitrary)  (still in the plane)
	Vector3d v1 = n.cross(arbitrary);
	v1 = v1 / v1.norm();           // normalise

	// Second direction = cross(n, v1)
	Vector3d v2 = n.cross(v1);
	v2 = v2 / v2.norm();

	// 3)  Produce the three points
	Vector3d p0 = anchor;
	Vector3d p1 = anchor + v1;   // shift one unit along v1
	Vector3d p2 = anchor + v2;   // shift one unit along v2

	return { p0, p1, p2 };
}

PointCloud applyCutPlane(PointCloud inputCloud, const Vector3d& origin, double pitch, double yaw, double roll)
{

	// Parametri di taglio definitivi
	std::array<Vector3d, 3>	planePoints = plane_points_from_anchor_and_euler(origin, pitch, roll, yaw);
	inputCloud = cloudPlaneCut(inputCloud, planePoints[0], planePoints[1], planePoints[2], false, false);

	return inputCloud;
}

//-----------------------------------------------------------------------------------------------------------------------

//
//   Funzioni Helper / Reindirizzanti
//

PointCloud convertPointCloud(const std::vector<PointXYZ>& oldPointCloud)
{
	PointCloud newPointCloud;
	newPointCloud.points.reserve(oldPointCloud.size());

	for (auto&& pt : oldPointCloud)
	{
		if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
		{
			newPointCloud.points.emplace_back(std::move(pt.x), std::move(pt.y), std::move(pt.z));
		}
	}

	return newPointCloud;
}

void cutPointCloud(PointCloud& pointCloud, std::vector<Vector3d> originCutPlanes, std::vector<Vector3d> inclinationCutPlanes)
{
	for (std::size_t i = 0; i < originCutPlanes.size(); ++i)
	{
		const auto& pt  = originCutPlanes[i];
		const auto& inc = inclinationCutPlanes[i];

		pointCloud = applyCutPlane(pointCloud, pt, inc.x, inc.y, inc.z);
	}
}

void projectPointCloud(const cv::Mat& image, const PointCloud& cloud, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, double scale, int img_width, int img_height, cv::Scalar color)
{
	if (cloud.points.empty())
	{
		std::cout << "[DEBUG - projectAndDrawPointCloud] No points in cloud" << std::endl;
		return;
	}

	Eigen::Vector3d n = normal.normalized();

	// Costruisci base ortonormale nel piano
	// Scegli un vettore arbitrario non parallelo alla normale
	Eigen::Vector3d arbitrary = std::abs(n.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();

	// Costruisci i due assi del piano usando Gram-Schmidt
	Eigen::Vector3d v_axis = (arbitrary - arbitrary.dot(n) * n).normalized();
	Eigen::Vector3d u_axis = n.cross(v_axis).normalized();


	// Prima passata: trova i bounding box delle coordinate proiettate
	double min_u = std::numeric_limits<double>::infinity();
	double max_u = -std::numeric_limits<double>::infinity();
	double min_v = std::numeric_limits<double>::infinity();
	double max_v = -std::numeric_limits<double>::infinity();

	std::vector<std::pair<double, double>> projected_coords;
	projected_coords.reserve(cloud.points.size());

	for (const auto& pt : cloud.points)
	{
		Eigen::Vector3d point(pt.x, pt.y, pt.z);
		Eigen::Vector3d toPoint = point - origin;

		// Proiezione ortogonale del punto sul piano
		double distance = toPoint.dot(n);
		Eigen::Vector3d proj = point - distance * n;

		// Coordinate nella base (u, v) del piano
		Eigen::Vector3d vec = proj - origin;
		double u = vec.dot(u_axis);
		double v = vec.dot(v_axis);

		projected_coords.push_back({ u, v });

		// Aggiorna bounding box  
		min_u = std::min(min_u, u);
		max_u = std::max(max_u, u);
		min_v = std::min(min_v, v);
		max_v = std::max(max_v, v);
	}

	// Evita divisione per zero
	if (max_u == min_u) max_u = min_u + 1.0;
	if (max_v == min_v) max_v = min_v + 1.0;

	// Seconda passata: disegna i punti
	for (size_t i = 0; i < projected_coords.size(); ++i)
	{
		double u = projected_coords[i].first;
		double v = projected_coords[i].second;

		// Mappa le coordinate del piano alle coordinate dell'immagine
		// Normalizza u e v nell'intervallo [0, 1] e scala per le dimensioni dell'immagine
		int x_img = static_cast<int>((1.0 - (u - min_u) / (max_u - min_u)) * (img_width - 1));
		int y_img = static_cast<int>(((v - min_v) / (max_v - min_v)) * (img_height - 1));

		if (x_img >= 0 && x_img < img_width && y_img >= 0 && y_img < img_height)
		{
			cv::circle(image, cv::Point(x_img, y_img), 1, color, -1);
		}
	}
}

void projectPointCloudBasedAnotherCloud(cv::Mat& image, const PointCloud& cloudToDraw, const PointCloud& cloudForBounds, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, double scale, int img_width, int img_height, cv::Scalar color, int radiusPoint, int thicknessPoint)
{
	if (cloudToDraw.points.empty())
	{
		std::cout << "[DEBUG - projectAndDrawPointCloud] No points to draw" << std::endl;
		return;
	}

	if (cloudForBounds.points.empty())
	{
		std::cout << "[DEBUG - projectAndDrawPointCloud] No points for bounds calculation" << std::endl;
		return;
	}

	Eigen::Vector3d n = normal.normalized();

	// Costruisci base ortonormale nel piano
	Eigen::Vector3d arbitrary = std::abs(n.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
	Eigen::Vector3d v_axis = (arbitrary - arbitrary.dot(n) * n).normalized();
	Eigen::Vector3d u_axis = n.cross(v_axis).normalized();

	// Prima passata: calcola i bounding box usando cloudForBounds
	double min_u = std::numeric_limits<double>::infinity();
	double max_u = -std::numeric_limits<double>::infinity();
	double min_v = std::numeric_limits<double>::infinity();
	double max_v = -std::numeric_limits<double>::infinity();

	for (const auto& pt : cloudForBounds.points)
	{
		Eigen::Vector3d point(pt.x, pt.y, pt.z);
		Eigen::Vector3d toPoint = point - origin;

		// Proiezione ortogonale del punto sul piano
		double distance = toPoint.dot(n);
		Eigen::Vector3d proj = point - distance * n;

		// Coordinate nella base (u, v) del piano
		Eigen::Vector3d vec = proj - origin;
		double u = vec.dot(u_axis);
		double v = vec.dot(v_axis);

		// Aggiorna bounding box
		min_u = std::min(min_u, u);
		max_u = std::max(max_u, u);
		min_v = std::min(min_v, v);
		max_v = std::max(max_v, v);
	}

	// Evita divisione per zero
	if (max_u == min_u) max_u = min_u + 1.0;
	if (max_v == min_v) max_v = min_v + 1.0;

	// Seconda passata: proietta e disegna solo i punti di cloudToDraw
	for (const auto& pt : cloudToDraw.points)
	{
		Eigen::Vector3d point(pt.x, pt.y, pt.z);
		Eigen::Vector3d toPoint = point - origin;

		// Proiezione ortogonale del punto sul piano
		double distance = toPoint.dot(n);
		Eigen::Vector3d proj = point - distance * n;

		// Coordinate nella base (u, v) del piano
		Eigen::Vector3d vec = proj - origin;
		double u = vec.dot(u_axis);
		double v = vec.dot(v_axis);

		// Mappa le coordinate del piano alle coordinate dell'immagine usando i bounds calcolati
		int x_img = static_cast<int>((1.0 - (u - min_u) / (max_u - min_u)) * (img_width - 1));
		int y_img = static_cast<int>(((v - min_v) / (max_v - min_v)) * (img_height - 1));

		if (x_img >= 0 && x_img < img_width && y_img >= 0 && y_img < img_height) 
		{
			cv::circle(image, cv::Point(x_img, y_img), radiusPoint, color, thicknessPoint);
		}
	}
}

void projectPoint(cv::Mat& image, const PointCloud& cloudForBounds, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, int img_width, int img_height, const Eigen::Vector3d& pointToHighlight, cv::Scalar color = cv::Scalar(0, 255, 255)) // Giallo di default
{
	if (cloudForBounds.points.empty())
	{
		std::cout << "[DEBUG - projectPoint] No points for bounds calculation" << std::endl;
		return;
	}

	Eigen::Vector3d n = normal.normalized();

	// Costruisci base ortonormale nel piano
	Eigen::Vector3d arbitrary = std::abs(n.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
	Eigen::Vector3d v_axis = (arbitrary - arbitrary.dot(n) * n).normalized();
	Eigen::Vector3d u_axis = n.cross(v_axis).normalized();

	// Prima passata: calcola i bounding box usando cloudForBounds
	double min_u = std::numeric_limits<double>::infinity();
	double max_u = -std::numeric_limits<double>::infinity();
	double min_v = std::numeric_limits<double>::infinity();
	double max_v = -std::numeric_limits<double>::infinity();

	for (const auto& pt : cloudForBounds.points)
	{
		Eigen::Vector3d point(pt.x, pt.y, pt.z);
		Eigen::Vector3d toPoint = point - origin;

		// Proiezione ortogonale del punto sul piano
		double distance = toPoint.dot(n);
		Eigen::Vector3d proj = point - distance * n;

		// Coordinate nella base (u, v) del piano
		Eigen::Vector3d vec = proj - origin;
		double u = vec.dot(u_axis);
		double v = vec.dot(v_axis);

		// Aggiorna bounding box
		min_u = std::min(min_u, u);
		max_u = std::max(max_u, u);
		min_v = std::min(min_v, v);
		max_v = std::max(max_v, v);
	}

	// Evita divisione per zero
	if (max_u == min_u) max_u = min_u + 1.0;
	if (max_v == min_v) max_v = min_v + 1.0;

	// Proietta il punto da evidenziare sul piano
	Eigen::Vector3d toHighlightPoint = pointToHighlight - origin;
	double highlight_distance = toHighlightPoint.dot(n);
	Eigen::Vector3d highlight_proj = pointToHighlight - highlight_distance * n;

	// Coordinate nella base (u, v) del piano
	Eigen::Vector3d highlight_vec = highlight_proj - origin;
	double highlight_u = highlight_vec.dot(u_axis);
	double highlight_v = highlight_vec.dot(v_axis);

	// Mappa le coordinate del piano alle coordinate dell'immagine usando i bounds calcolati
	int highlight_x_img = static_cast<int>((1.0 - (highlight_u - min_u) / (max_u - min_u)) * (img_width - 1));
	int highlight_y_img = static_cast<int>(((highlight_v - min_v) / (max_v - min_v)) * (img_height - 1));

	// Disegna il punto evidenziato se è dentro i limiti dell'immagine
	if (highlight_x_img >= 0 && highlight_x_img < img_width && highlight_y_img >= 0 && highlight_y_img < img_height)
	{
		cv::circle(image, cv::Point(highlight_x_img, highlight_y_img), 6, color, 2); // Cerchio principale
		cv::circle(image, cv::Point(highlight_x_img, highlight_y_img), 6, cv::Scalar(0, 0, 0), 1); // Bordo nero per contrasto
	}
}

PointCloud generateProjectionLinePointCloud(const Vector3d& a, const Vector3d& b_dir, const Vector3d& min_bound, const Vector3d& max_bound, double spacing)
{
	PointCloud pointcloud;

	// Normalizza la direzione
	double norm = b_dir.norm();
	if (norm < 1e-8) return pointcloud; // direzione nulla
	Vector3d dir = b_dir / norm;

	double t_min = -std::numeric_limits<double>::infinity();
	double t_max = std::numeric_limits<double>::infinity();

	// Per ogni asse x, y, z
	for (int i = 0; i < 3; ++i)
	{
		double a_i = (i == 0) ? a.x : (i == 1) ? a.y : a.z;
		double d_i = (i == 0) ? dir.x : (i == 1) ? dir.y : dir.z;
		double min_i = (i == 0) ? min_bound.x : (i == 1) ? min_bound.y : min_bound.z;
		double max_i = (i == 0) ? max_bound.x : (i == 1) ? max_bound.y : max_bound.z;

		if (std::abs(d_i) < 1e-8)
		{
			if (a_i < min_i || a_i > max_i) return pointcloud; // fuori dal box
		}
		else
		{
			double t1 = (min_i - a_i) / d_i;
			double t2 = (max_i - a_i) / d_i;
			if (t1 > t2) std::swap(t1, t2);
			t_min = std::max(t_min, t1);
			t_max = std::min(t_max, t2);
		}
	}

	if (t_min > t_max) return pointcloud; // la linea non interseca il box

	int num_points = static_cast<int>((t_max - t_min) / spacing) + 1;

	for (int i = 0; i <= num_points; ++i)
	{
		double t = t_min + i * spacing;
		Vector3d p = a + dir * t;
		pointcloud.points.push_back(p);
	}

	return pointcloud;
}

void filterPointCloud(PointCloud& pointCloud, double neighbor_radius, int min_neighbors, double max_distance)
{
	size_t numberOfPoints = pointCloud.points.size();
	if (numberOfPoints == 0) return;

	// Calcolo del centroide della Point Cloud
	Vector3d centroid(0, 0, 0);

	for (const auto& p : pointCloud.points)
		centroid = centroid + p;

	centroid = centroid / numberOfPoints;

	double neighbor_radius2 = neighbor_radius * neighbor_radius;
	double max_distance2    = max_distance * max_distance;

	PointCloud filtered;

	filtered.points.reserve(numberOfPoints);

	for (int i = 0; i < numberOfPoints; ++i) 
	{
		const Vector3d& pt = pointCloud.points[i];

		// Controllo distanza dal centroide
		if ((pt - centroid).norm() > max_distance2)
			continue;

		// Conta vicini
		int neighbors = 0;

		for (int j = 0; j < numberOfPoints; ++j) 
		{
			if (i == j) continue;
			if ((pt - pointCloud.points[j]).norm() <= neighbor_radius2)
				neighbors++;
		}

		if (neighbors >= min_neighbors)
			filtered.points.push_back(pt);
	}
	pointCloud.points = std::move(filtered.points);
}

template <typename T>
void checkPointCloud(const std::vector<T>& cloud, const std::string& message)
{
	if (cloud.empty())
	{
		std::cerr << "[Error] The Point Cloud is empty" << std::endl;
		std::exit(EXIT_FAILURE);
	}
	else
	{
		std::cout << message << cloud.size() << "\n" << std::endl;
	}
}

// Funzione per utilizzare Drawgeometry di Open3D con le nostre PointCloud
std::shared_ptr<open3d::geometry::PointCloud> MakePointCloud(PointCloud pcd, cv::Scalar color = cv::Scalar(255, 255, 255))
{
	auto cloud = std::make_shared<open3d::geometry::PointCloud>();
	int npts = pcd.points.size();

	cloud->points_.reserve(npts);
	for (int i = 0; i < npts; ++i)
	{
		cloud->points_.push_back({ pcd.points[i].x,pcd.points[i].y ,pcd.points[i].z });
	}


	cloud->colors_.reserve(npts);
	for (int i = 0; i < npts; ++i)
	{
		cloud->colors_.push_back({ color[2],color[1],color[0] });
	}

	return cloud;
}




void startPlaneCuttingSearch(PointCloud& cloud, Eigen::Vector3d& projectonPlaneOrigin, Eigen::Vector3d& projectonPlaneNormal, double scale, int img_width, int img_height)
{
	// Parametri iniziali
	double pitch = -30, yaw = 500, roll = 70;
	double _step = 0.1;
	cv::Mat img;
	char key = 0;

	Eigen::Vector3d cutPlaneOrigin(187.899, 86.022, 824.946);
	PointCloud cuttedPointCloud;

	do
	{
		img = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));

		projectPointCloud(img, cloud, projectonPlaneOrigin, projectonPlaneNormal, scale, img_width, img_height, cv::Scalar(255, 255, 255));
		projectPoint(img, cloud, projectonPlaneOrigin, projectonPlaneNormal, img_width, img_height, cutPlaneOrigin);

		std::array<Vector3d, 3> planePoints;
		Vector3d converted_origin(cutPlaneOrigin.x(), cutPlaneOrigin.y(), cutPlaneOrigin.z());
		planePoints = plane_points_from_anchor_and_euler(converted_origin, pitch, roll, yaw);

		cuttedPointCloud = cloudPlaneCut(cloud, planePoints[0], planePoints[1], planePoints[2], false, false);

		// Visualizza i punti che teniamo in verde
		projectPointCloudBasedAnotherCloud(img, cuttedPointCloud, cloud, projectonPlaneOrigin, projectonPlaneNormal, scale, img_width, img_height, cv::Scalar(0, 255, 0));

		cv::imshow("Plane Cutting View", img);

		key = cv::waitKey(10);

		// Controllo spostamento origine
		if (key == 'q') cutPlaneOrigin.x() += _step;
		if (key == 'w') cutPlaneOrigin.x() -= _step;
		if (key == 'a') cutPlaneOrigin.y() += _step;
		if (key == 's') cutPlaneOrigin.y() -= _step;
		if (key == 'z') cutPlaneOrigin.z() += _step;
		if (key == 'x') cutPlaneOrigin.z() -= _step;

		if (key == 'p') { _step *= 10; std::cout << "step: " << _step << std::endl; }
		if (key == 'o') { _step /= 10; std::cout << "step: " << _step << std::endl; }

		if (key == 'i')
		{
			cutPlaneOrigin = Eigen::Vector3d(187.899, 206.022, 776.046);
			pitch = yaw = roll = 0.0;
		}

		if (key == 'j') pitch += _step;
		if (key == 'k') yaw += _step;
		if (key == 'l') roll += _step;
		if (key == 'b') pitch -= _step;
		if (key == 'n') yaw -= _step;
		if (key == 'm') roll -= _step;

		std::cout << "Origin coordinates: X=" << cutPlaneOrigin.x()
			<< ", Y=" << cutPlaneOrigin.y()
			<< ", Z=" << cutPlaneOrigin.z() << std::endl;
		std::cout << "Inclinazione: pitch=" << pitch << ", yaw=" << yaw << ", roll=" << roll << std::endl;

	} while (key != 27);

	cv::destroyAllWindows();

	//
	// Visualizzazione con Open3D
	//
	//	Vector3d(187.899, 86.022, 824.946)     //Taglio punti laterali a destra
	//};


	//	Vector3d(-30,500,70),
	//};

	checkPointCloud(cuttedPointCloud.points, "\n[Debug] Point Cloud points post - plane cut: ");

	std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geoms;

	// Aggiungiamo alle nostre geometrie la PointCloud originale
	geoms.push_back(MakePointCloud(cuttedPointCloud, cv::Scalar(0, 0, 0)));

	// Visualizzatore 3D delle geometrie 
	open3d::visualization::DrawGeometries(geoms);
}


PointCloud filterIntersection(
	const PointCloud& intersectionPoints,  // punti di intersezione da filtrare
	const PointCloud& referenceCloud,      // point cloud originale
	double neighbor_radius,                 // raggio entro cui cercare punti vicini
	int min_neighbors                       // numero minimo di punti per conservare l'intersezione
)
{
	PointCloud filtered;

	for (const auto& pt : intersectionPoints.points)
	{
		int count = 0;
		for (const auto& ref_pt : referenceCloud.points)
		{
			if ((pt - ref_pt).norm() <= neighbor_radius)
				count++;
		}

		if (count >= min_neighbors)
			filtered.points.push_back(pt);
	}

	return filtered;
}


//
// Funzioni Principale
//

void show3dBinderPointCloud(PointCloud pointCloud,std::vector<Vector3d> originCutPlanes,std::vector<Vector3d> inclinationCutPlanes)
{
	cutPointCloud(pointCloud, originCutPlanes, inclinationCutPlanes);

	double neighbor_radius = 5;
	int min_neighbors = 20;
	double max_distance = 12.0;

	filterPointCloud(pointCloud, neighbor_radius, min_neighbors, max_distance);

	// Visualizzazione 3D con Open3D 
	std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geoms; 

	// Aggiungiamo alle nostre geometrie la PointCloud originale 
	geoms.push_back(MakePointCloud(pointCloud, cv::Scalar(0, 0, 0))); 

	// Visualizzatore 3D delle geometrie
	open3d::visualization::DrawGeometries(geoms);

	// SALVARE LA POINTCLOUD IN UN FILE DOPO AVERLA TAGLIATA E FILTRATA E VISUALIZZATA
	auto cloud_ptr = MakePointCloud(pointCloud, cv::Scalar(0, 0, 0));
	if (open3d::io::WritePointCloud("BordoLegatrice.pcd", *cloud_ptr))
	{
		std::cout << "PointCloud salvata con successo" << std::endl;
	}
	else
	{
		std::cerr << "Errore nel salvataggio della PointCloud!" << std::endl;
	}

}


cv::Mat start3dPointCloudCut(const std::vector<PointXYZ>& cloud, int img_width, int img_height, Eigen::Vector3d& origin, Eigen::Vector3d& normal, double scale)
{
	// Creazione immagine OpenCV nera
	cv::Mat otp_image(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));

	checkPointCloud(cloud, "[Debug] Point Cloud points pre - conversion: ");

	PointCloud pointCloud = convertPointCloud(cloud);

	checkPointCloud(pointCloud.points);

	// Proietto la PointCloud sulla immagine 2D di OpenCV
	projectPointCloud(otp_image, pointCloud, origin, normal, scale, img_width, img_height);

	startPlaneCuttingSearch(pointCloud, origin, normal, scale, img_width, img_height);

	return otp_image;

}

cv::Mat processPointCloud(const std::vector<PointXYZ>& cloud, int img_width, int img_height, Eigen::Vector3d& origin, Eigen::Vector3d& normal, double scale,std::vector<Vector3d> originCutPlanes,std::vector<Vector3d> inclinationCutPlanes)
{
	// Creazione immagine OpenCV nera
	cv::Mat otp_image(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));

	checkPointCloud(cloud, "[Debug] Point Cloud points pre - conversion: ");

    PointCloud pointCloud = convertPointCloud(cloud);

	checkPointCloud(pointCloud.points, "[Debug] Point Cloud points post - conversion: ");

	// [Debug] Proietto la PointCloud originale
	projectPointCloud(otp_image, pointCloud, origin, normal, scale, img_width, img_height);

	// Salviamo un istanza della PointCloud prima di Tagliarla
	PointCloud originalPointCloud = pointCloud;

	// Taglio della PointCloud con i piani di taglio definiti in configurazione
	cutPointCloud(pointCloud, originCutPlanes, inclinationCutPlanes);

	checkPointCloud(pointCloud.points, "[Debug] Point Cloud points post - plane cut: ");

	// [Debug] Proietto la PointCloud rimasta dopo il taglio
	//projectPointCloudBasedAnotherCloud(otp_image, pointCloud, originalPointCloud, origin, normal, scale, img_width, img_height, cv::Scalar(0, 255, 0));

	// Hough per trovare i tornidi orizzontali
	PointCloud remainingPoints;
	std::vector<Line3D> houghLines = getLineSteelBars(pointCloud, remainingPoints);
	std::cout << "Rilevate " << houghLines.size() << " rette" << "\n" << std::endl;
	Vector3d minPshifted, maxPshifted;
	pointCloud.getMinMax3D(&minPshifted, &maxPshifted);
	PointCloud linesPointCloud;
	for(const auto& line : houghLines)
	{
		auto pc = generateProjectionLinePointCloud(line.point, line.direction, minPshifted, maxPshifted, 0.05);
		for (const auto& p : pc.points)
		{
			linesPointCloud.points.push_back(p);
		}
	}

	// [Debug] Proietto le rette trovate
	//projectPointCloudBasedAnotherCloud(otp_image, linesPointCloud, originalPointCloud, origin, normal, scale, img_width, img_height,cv::Scalar(0,0,255));
	
	// [Debug] Proietto i punti rimasti, non utilizzati per il rilevamento delle rette
	//projectPointCloudBasedAnotherCloud(otp_image, remainingPoints, originalPointCloud, origin, normal, scale, img_width, img_height,cv::Scalar(255,0,0));

	// Pulisco la PointCloud da eventuali punti isolati o troppo lontani
	//filterPointCloud(remainingPoints,4.0,56,15.0);

	// [Debug] Proietto i punti rimasti dopo la pulizia
	//projectPointCloudBasedAnotherCloud(otp_image, remainingPoints, originalPointCloud, origin, normal, scale, img_width, img_height);

	// Applico l'algoritmo di Hough per trovare gli archi
	std::vector<ArcPlane> houghArcs = getArcSteelBars(remainingPoints);
	std::cout << "Rilevate " << houghArcs.size() << " semi circonferenze" << "\n" << std::endl;

	// Calcolo e proiezione delle intersezioni
	std::vector<Vector3d> intersectionPoints = getIntersectionPoints(houghLines, houghArcs);

	// Trasforma intersectionPoints in PointCloud
	PointCloud intersectionCloud;
	for (const auto& pt : intersectionPoints)
		intersectionCloud.points.push_back(pt);

	// Filtra le intersezioni sospese nel vuoto
	double neighbor_radius = 10.0; // raggio entro cui cercare punti vicini
	int min_neighbors = 4;        // numero minimo di punti per conservare l'intersezione
	PointCloud filteredIntersection = filterIntersection(intersectionCloud, originalPointCloud, neighbor_radius, min_neighbors);

	std::cout << "Rilevate " << filteredIntersection.points.size() << " intersezioni" << "\n" << std::endl;

	// Proietta i punti filtrati
	projectPointCloudBasedAnotherCloud(
		otp_image,
		filteredIntersection,
		originalPointCloud,
		origin, normal, scale,
		img_width, img_height,
		cv::Scalar(0, 255, 0),       
		6, 4                        
	);


	//
	// Visualizzazione 3D con Open3D
	//

	std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geoms;

	// Aggiungiamo alle nostre geometrie la PointCloud originale
	geoms.push_back(MakePointCloud(originalPointCloud, cv::Scalar(0, 0, 0)));

	// Aggiungiamo i punti dell'intersezione
	for (const auto& i3d : filteredIntersection.points)
	{
		auto s = open3d::geometry::TriangleMesh::CreateSphere(20);
		s->PaintUniformColor({ 1.0,0,0 });
		s->Translate({ i3d.x, i3d.y, i3d.z });
		geoms.push_back(s);
	}
	// Visualizzatore 3D delle geometrie 
	open3d::visualization::DrawGeometries(geoms);

    return otp_image;
}