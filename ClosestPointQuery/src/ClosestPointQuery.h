#pragma once
#include <vector>
#include <RTree.h>
#include <glm/glm.hpp>

typedef glm::vec3 Point;
typedef glm::vec3 Vec3;
struct Triangle {
	Point vertices[3];
	Point min, max;
	Triangle(Point p1, Point p2, Point p3) : vertices{ p1, p2, p3 }, min(glm::min(glm::min(p1, p2), p3)), max(glm::max(glm::max(p1, p2), p3)) {}
};

struct Mesh {
	std::vector<Point> vertices;
	std::vector<int> indices;
};

class ClosestPointQuery {
public:
	ClosestPointQuery(const Mesh& m);
	//! Return the closest point on the mesh within the specified maximum search distance.
	bool operator() (const Point& query_point, float max_dist, Point& closest_point) const;
private:
	RTree<Triangle*, float, 3, double, 8> triangles_r_tree;
};