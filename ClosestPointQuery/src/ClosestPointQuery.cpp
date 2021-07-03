#include "ClosestPointQuery.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/normal.hpp>
#include <chrono>

namespace geoutils {

	ClosestPointQuery::ClosestPointQuery(const Mesh& m) {
		// Construct the R-Tree
		for (size_t i = 0; i < m.indices.size(); i += 3) {
			const auto& p1 = m.vertices[m.indices[i + 0]];
			const auto& p2 = m.vertices[m.indices[i + 1]];
			const auto& p3 = m.vertices[m.indices[i + 2]];
			Triangle* tri = new Triangle(p1, p2, p3);
			Vec3 min = glm::min(glm::min(p1, p2), p3);
			Vec3 max = glm::max(glm::max(p1, p2), p3);
			triangles_r_tree.Insert(glm::value_ptr(min), glm::value_ptr(max), tri);
		}
	}

	ClosestPointQuery::~ClosestPointQuery() {
		// Release memory
		int itIndex = 0;
		TriangleRTree::Iterator it;
		triangles_r_tree.GetFirst(it);
		while (!it.IsNull()) {
			Triangle* tri = *it;
			++it;
			delete tri;
		}
	}

	bool ClosestPointQuery::operator() (const Point& query_point, float max_dist, Point& closest_point) const {
		// First, get the bounding box of the radius positioned at query point.
		double shortest_distance = DBL_MAX;
		const Point search_min(query_point - Vec3(max_dist));
		const Point search_max(query_point + Vec3(max_dist));
		const auto search_callback = [&](Triangle* tri) -> bool {
			uint8_t outside_count = 0;

			// Determine the triangle normal and projected point.
			const auto& vert = tri->vertices;
			const Vec3 normal = glm::normalize(glm::cross(vert[1] - vert[0], vert[2] - vert[0]));
			const Vec3 projection = glm::dot(vert[0] - query_point, normal) * normal;
			const double distance_to_plane = glm::length2(projection);

			// Early termination. (distance_to_plane is already the shortest possible distance to the triangle, there's no reason to proceed)
			if (distance_to_plane > shortest_distance) return true;

			const Point projected = query_point + projection;
			for (uint8_t i = 0; i < 3; ++i) {
				const Point& v1 = vert[i];
				const Point& v2 = vert[(i + 1) % 3];

				// Utilize the winding order to determine if the point lies outside of an edge.
				const bool outside = glm::dot(glm::cross(v1 - projected, v2 - projected), normal) < 0.f;
				if (outside) {
					outside_count++;
					// Clamp the projection value to be in-between of the two ends of the edge.
					const float t = glm::clamp(glm::dot(v2 - v1, projected - v1) / glm::distance2(v1, v2), 0.f, 1.f);
					const Point closest_point_on_edge = v1 * (1.f - t) + v2 * t;
					const double distance_to_edge = glm::distance2(query_point, closest_point_on_edge);
					if (distance_to_edge < shortest_distance) {
						closest_point = closest_point_on_edge;
						shortest_distance = distance_to_edge;
					}
				}

				// Early termination. (A point can only be outside of at most 2 edges)
				if (outside_count > 1) return true;
			}

			// Projection of the query point lies within the triangle.
			if (outside_count == 0) {
				closest_point = projected;
				shortest_distance = distance_to_plane;
			}
			return true; // Keep traversing
		};

		// Query the R-Tree around the bounding box which we defined eariler.
		// For each overlapping triangles, find the closest point from the query point to the triangle.
		// A detailed explanation can be found in README.md.
		triangles_r_tree.Search(
			glm::value_ptr(search_min),
			glm::value_ptr(search_max),
			search_callback
		);

		return shortest_distance != DBL_MAX; // Return true if the closest point is found, else false.
	}

} // namespace geoutils