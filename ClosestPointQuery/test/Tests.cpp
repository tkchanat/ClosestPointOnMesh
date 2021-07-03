#include <random>
#include <gtest/gtest.h>
#include <ClosestPointQuery.h>

using namespace geoutils;

// Constants declaration
const Mesh TRIANGLE_MESH = { {Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(-1.0, 0.0, 0.0)} /*vertices*/, {0, 1, 2} /*indices*/ };

// Given a point lies inside the triangle, the closest point should be the point itself.
TEST(ClosestPointQuery_CoplanarCases, Coplanar) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(0.0, 0.5, 0.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(0.0, 0.5, 0.0));
}
// Given a point lies on one edge, the closest point should be the point itself.
TEST(ClosestPointQuery_CoplanarCases, OnEdge) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(0.0, 0.0, 0.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(0.0, 0.0, 0.0));
}
// Given a point lies on one of the vertices , the closest point should be the point itself.
TEST(ClosestPointQuery_CoplanarCases, OnVertex) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(1.0, 0.0, 0.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(1.0, 0.0, 0.0));
}
// Given a point nowhere near the triangle with an unreachable query distance, there shouldn't be any closest points found.
TEST(ClosestPointQuery_CoplanarCases, NotFound) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(2.0, 0.0, 0.0), 0.5, closest_point);
	EXPECT_FALSE(found);
}

// Given a point directly above the triangle, the closest point should be the projection onto the triangle.
TEST(ClosestPointQuery_ProjectionCases, ProjectOnFace) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(0.0, 0.5, 1.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(0.0, 0.5, 0.0));
}
// Given a point above the triangle and outward from the edge direction, the closest point should be the on the closest edge.
TEST(ClosestPointQuery_ProjectionCases, ProjectOnEdge) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(0.0, -1.0, 1.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(0.0, 0.0, 0.0));
}
// Given a point above the triangle and outward from the vertex direction, the closest point should be the on the closest vertex.
TEST(ClosestPointQuery_ProjectionCases, ProjectOnVertex) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(1.0, -1.0, 1.0), FLT_MAX, closest_point);
	EXPECT_TRUE(found);
	EXPECT_EQ(closest_point, Point(1.0, 0.0, 0.0));
}
// Given a point nowhere near the triangle with an unreachable query distance, there shouldn't be any closest points found.
TEST(ClosestPointQuery_ProjectionCases, NotFound) {
	ClosestPointQuery query(TRIANGLE_MESH);
	Point closest_point;
	bool found = query(Point(1.0, -1.0, 1.0), 0.5, closest_point);
	EXPECT_FALSE(found);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
