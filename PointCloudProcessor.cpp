//
//   Inclusione di librerie
//
#include "PointCloudProcessor.h"
#include "pointcloud.h"
#include "vector3d.h"


// Funzione per convertire gli angoli in gradi in radianti
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
	std::array<Vector3d, 3> planePoints;
	planePoints = plane_points_from_anchor_and_euler(origin, pitch, roll, yaw);
	inputCloud = cloudPlaneCut(inputCloud, planePoints[0], planePoints[1], planePoints[2], true);

	return inputCloud;
}

//-----------------------------------------------------------------------------------------------------------------------

//
//   Funzioni Helper
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

		//pointCloud = applyCutPlane(pointCloud, pt, inc.x, inc.y, inc.z);
	}
}

void projectAndDrawPointCloud(const cv::Mat& image, const PointCloud& cloud, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, double scale, int img_width, int img_height, cv::Scalar color)
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
	Eigen::Vector3d u_axis = (arbitrary - arbitrary.dot(n) * n).normalized();
	Eigen::Vector3d v_axis = n.cross(u_axis).normalized();

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
		int y_img = static_cast<int>((1.0 - (v - min_v) / (max_v - min_v)) * (img_height - 1));

		if (x_img >= 0 && x_img < img_width && y_img >= 0 && y_img < img_height)
		{
			cv::circle(image, cv::Point(x_img, y_img), 1, color, -1);
		}
	}
}

//
// Funzione Principale
//

cv::Mat processPointCloud(const std::vector<PointXYZ>& cloud, int img_width, int img_height, Eigen::Vector3d& origin, Eigen::Vector3d& normal, double scale,std::vector<Vector3d> originCutPlanes,std::vector<Vector3d> inclinationCutPlanes)
{
	cv::Mat otp_image(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0)); // Disegno l'immagine nera di sfondo

    if (cloud.empty())
        return otp_image;
	else
		std::cout << "[Debug] Point Cloud pre-conversion points:" << cloud.size() << "\n" << std::endl;

    PointCloud pointCloud = convertPointCloud(cloud);

    if (pointCloud.points.empty())
        return otp_image;
	else
		std::cout << "[Debug] Point Cloud points:"<< pointCloud.points.size()<<"\n" << std::endl;

	// Salviamo un istanza della PointCloud prima di Tagliarla
	PointCloud pointCloud_preCut = pointCloud;

	cutPointCloud(pointCloud, originCutPlanes, inclinationCutPlanes);

	if (pointCloud.points.empty())
		return otp_image;
	else
		std::cout << "[Debug] Point Cloud points post-cleaning:" << pointCloud.points.size() << "\n" << std::endl;

    projectAndDrawPointCloud(otp_image, pointCloud, origin, normal, scale, img_width, img_height);

    return otp_image;
}