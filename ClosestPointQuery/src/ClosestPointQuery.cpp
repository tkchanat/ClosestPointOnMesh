#define TINYOBJLOADER_IMPLEMENTATION
#include "ClosestPointQuery.h"
#include <tiny_obj_loader.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

const char* MODEL_PATH = "Assets/bunny.obj";
const char* VISUALIZER_CSV_PATH = "Visualizer/query_points.csv";

std::vector<Mesh> load_obj_model(const char* model_path) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn, err;

	if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, model_path)) {
		throw std::runtime_error(warn + err);
	}

	std::vector<Mesh> meshes(shapes.size());
	for (size_t i = 0; i < shapes.size(); ++i) {
		meshes[i].vertices.reserve(attrib.vertices.size() / 3);
		for (size_t j = 0; j < attrib.vertices.size(); j += 3) {
			meshes[i].vertices.push_back(Point(attrib.vertices[j], attrib.vertices[j + 1], attrib.vertices[j + 2]));
		}
		meshes[i].indices.reserve(shapes[i].mesh.indices.size());
		for (const auto& index : shapes[i].mesh.indices) {
			meshes[i].indices.push_back(index.vertex_index);
		}
	}
	return meshes;
}

ClosestPointQuery::ClosestPointQuery(const Mesh& m) {
	// Construct the R-Tree
	for (size_t i = 0; i < m.indices.size(); i += 3) {
		const auto& p1 = m.indices[i + 0];
		const auto& p2 = m.indices[i + 1];
		const auto& p3 = m.indices[i + 2];
		Triangle* tri = new Triangle(m.vertices[p1], m.vertices[p2], m.vertices[p3]);
		triangles_r_tree.Insert(glm::value_ptr(tri->min), glm::value_ptr(tri->max), tri);
	}
}

std::string format_threejs_vector(Vec3 vec) {
	return "new THREE.Vector3(" + std::to_string(vec.x) + "," + std::to_string(vec.y) + "," + std::to_string(vec.z) + ")";
}

bool ClosestPointQuery::operator()(const Point& query_point, float max_dist, Point& closest_point) const {
	// First, get the bounding box of the radius positioned at query point.
	const Point search_min(query_point - Vec3(max_dist));
	const Point search_max(query_point + Vec3(max_dist));
	double best_distance = DBL_MAX;
	auto search_callback = [&](Triangle* tri) -> bool {
		const auto& vert = tri->vertices;
		const Vec3 normal = glm::normalize(glm::cross(vert[1] - vert[0], vert[2] - vert[0]));
		const Vec3 projection = glm::dot(vert[0] - query_point, normal) * normal;
		const double distance_to_plane = glm::length2(projection);
		const Point projected = query_point + projection;
		uint8_t outside_count = 0;
		if (distance_to_plane > best_distance) return true;
		for (uint8_t i = 0; i < 3; ++i) {
			const Point& v1 = vert[i];
			const Point& v2 = vert[(i + 1) % 3];
			bool outside = glm::dot(glm::cross(v1 - projected, v2 - projected), normal) < 0.f;
			if (outside) {
				outside_count++;
				float t = glm::clamp(glm::dot(v2 - v1, projected - v1) / glm::distance2(v1, v2), 0.f, 1.f);
				Point closest_point_on_edge = v1 * (1.f - t) + v2 * t;
				const double distance_to_edge = glm::distance2(query_point, closest_point_on_edge);
				if (distance_to_edge < best_distance) {
					closest_point = closest_point_on_edge;
					best_distance = distance_to_edge;
				}
			}
			if (outside_count > 1) break;
		}
		if (outside_count == 0) {
			closest_point = projected;
			best_distance = distance_to_plane;
		}
		return true; // keep traversing
	};
	triangles_r_tree.Search(glm::value_ptr(search_min), glm::value_ptr(search_max), search_callback);
	return best_distance != DBL_MAX;
}

inline double random_double() {
	static std::uniform_real_distribution<double> distribution(0.0, 1.0);
	static std::mt19937 generator;
	return distribution(generator);
}
inline double random_double(double min, double max) {
	// Returns a random real in [min,max).
	return min + (max - min) * random_double();
}
Vec3 random_in_unit_sphere() {
	while (true) {
		Vec3 p = Vec3(random_double(-1, 1), random_double(-1, 1), random_double(-1, 1));
		if (glm::length2(p) >= 1) continue;
		return p;
	}
}

int main() {
	const std::vector<Mesh> meshes = load_obj_model(MODEL_PATH);
	std::vector<std::pair<float, Point>> query_points;
	for (size_t i = 0; i < 10000; ++i) query_points.push_back({ 1.f, 2.f * random_in_unit_sphere() });
	std::vector<std::pair<bool, Point>> closest_points(query_points.size());

	auto start = std::chrono::high_resolution_clock::now();
	for (const Mesh& mesh : meshes) {
		ClosestPointQuery query(mesh);
		std::cout << "Constructed ClosestPointQuery!" << "\n";
		for (size_t i = 0; i < query_points.size(); ++i) {
			closest_points[i].first = query(
				query_points[i].second,	  // Query point position
				query_points[i].first, 	  // Maximum query distance
				closest_points[i].second  // Output closest point
			);
		}
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::cout << "Querying " << query_points.size() << " points on " << meshes.size() << " mesh took "
		<< std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count() / 1000.0
		<< " ms\n";

	// Query points CSV
	std::ofstream query_points_csv(VISUALIZER_CSV_PATH, std::ofstream::trunc);
	if (query_points_csv.is_open()) {
		for (size_t i = 0; i < query_points.size(); ++i) {
			// One single query will be in:
			// <max_dist>,<query_x>,<query_y>,<query_z>, <found>,<closest_x>,<closest_y>,<closest_z>
			query_points_csv << query_points[i].first << "," << query_points[i].second.x << "," << query_points[i].second.y << "," << query_points[i].second.z << ",";
			query_points_csv << closest_points[i].first << "," << closest_points[i].second.x << "," << closest_points[i].second.y << "," << closest_points[i].second.z;
			query_points_csv << "\n";
		}
		query_points_csv.close();
	}

	return 0;
}