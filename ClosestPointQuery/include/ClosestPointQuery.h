#pragma once
#define GLM_FORCE_INTRINSICS
#include <vector>
#include <RTree.h>
#include <glm/glm.hpp>

namespace geoutils {

	typedef glm::vec3 Point;
	typedef glm::vec3 Vec3;

	// Define a mesh by a collection of vertices and indices
	struct Mesh {
		std::vector<Point> vertices;
		std::vector<int> indices;
	};

	class ClosestPointQuery {
	private:
		// Define a triangle with 3 points
		__declspec(align(16)) struct Triangle {
			Point vertices[3];
			explicit Triangle(Point p1, Point p2, Point p3) : vertices{ p1, p2, p3 } {}
			void* operator new(size_t i) { return _mm_malloc(i, 16); }
			void operator delete(void* p) { _mm_free(p); }
		};
		typedef RTree<Triangle*, float, 3, double, 8> TriangleRTree;
	public:
		explicit ClosestPointQuery(const Mesh& m);
		~ClosestPointQuery();
		// Extract the closest point on the mesh within the specified maximum search distance.
		// Return true if closest point is found, else false.
		bool operator()(const Point& query_point, float max_dist, Point& closest_point) const;
	private:
		TriangleRTree triangles_r_tree;
	};

} // namespace geoutils