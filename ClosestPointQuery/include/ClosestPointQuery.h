#pragma once
#include "RStarTree.h"

namespace geoutils {

	// Define a mesh by a collection of vertices and indices
	struct Mesh {
		std::vector<Point> vertices;
		std::vector<int> indices;
		Mesh() = default;
		~Mesh() = default;
		Mesh(const Mesh&) = default;
		Mesh& operator=(const Mesh&) = default;
	};

	class ClosestPointQuery {
	private:
		// Define a triangle with 3 points
		struct Triangle {
			Point vertices[3];
			explicit Triangle(Point p1, Point p2, Point p3) : vertices{ p1, p2, p3 } {}
		};
	public:
		explicit ClosestPointQuery(const Mesh& m);
		~ClosestPointQuery() = default;
		ClosestPointQuery(const ClosestPointQuery&) = default;
		ClosestPointQuery& operator=(const ClosestPointQuery&) = default;

		// Extract the closest point on the mesh within the specified maximum search distance.
		// Return true if closest point is found, else false.
		bool operator()(const Point& query_point, float max_dist, Point& closest_point) const;
	private:
		std::vector<Triangle> triangles;
		RStarTree<Triangle*, 64> r_star_tree;
	};

} // namespace geoutils