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

TEST(BoundingBox_Intersection, Overlap) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(0.5, 0.5, 0.5), Point(1.5, 1.5, 1.5) };
	EXPECT_TRUE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_FALSE(b.is_inside(a));
	EXPECT_FALSE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Intersection, Inside) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(0.2, 0.2, 0.2), Point(0.8, 0.8, 0.8) };
	EXPECT_TRUE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_TRUE(b.is_inside(a));
	EXPECT_TRUE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Intersection, Disjoint) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(1.5, 1.5, 1.5), Point(2, 2, 2) };
	EXPECT_FALSE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_FALSE(b.is_inside(a));
	EXPECT_FALSE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Intersection, Touching) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(1, 0, 0), Point(2, 1, 1) };
	EXPECT_FALSE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_FALSE(b.is_inside(a));
	EXPECT_FALSE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Geometry, Overlap) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(0.5, 0.5, 0.5), Point(1.5, 1.5, 1.5) };
	EXPECT_FLOAT_EQ(a.area(), 1.f);
	EXPECT_FLOAT_EQ(a.margin(), 3.f);
	EXPECT_FLOAT_EQ(a.overlap(b), 0.125f);
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Inside) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(0.2, 0.2, 0.2), Point(0.8, 0.8, 0.8) };
	EXPECT_FLOAT_EQ(b.area(), 0.216f);
	EXPECT_FLOAT_EQ(b.margin(), 1.8f);
	EXPECT_FLOAT_EQ(b.overlap(a), b.area());
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Disjoint) {
	BoundingBox a{ Point(0, 0, 0), Point(1, 1, 1) };
	BoundingBox b{ Point(1.5, 1.5, 1.5), Point(2, 2, 2) };
	EXPECT_FLOAT_EQ(b.area(), 0.125f);
	EXPECT_FLOAT_EQ(b.margin(), 1.5f);
	EXPECT_FLOAT_EQ(b.overlap(a), 0.f);
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Real) {
	BoundingBox a{ Point(-1.23428202, -0.985212982, -0.565617025), Point(1.16378295, 0.548205018, 0.691652000) };
	BoundingBox b{ Point(-1.21273303, 0.519062996, -0.932524025), Point(0.356427014, 1.31115603, 0.387724012) };
	BoundingBox c{ Point(-0.568542,0.886272,0.005542), Point(-0.533288,0.965194,0.060187) };
	EXPECT_FALSE(a.is_overlapping(c));
	EXPECT_TRUE(b.is_overlapping(c));
	EXPECT_LT(a.overlap(c), b.overlap(c));
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
