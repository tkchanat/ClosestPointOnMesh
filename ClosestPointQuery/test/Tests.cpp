#include <gtest/gtest.h>
#include <ClosestPointQuery.h>
using namespace geoutils;

// Constants declaration
const Mesh TRIANGLE_MESH = { {Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(-1.0, 0.0, 0.0)} /*vertices*/, {0, 1, 2} /*indices*/ };

TEST(Math_Vec3, Construct) {
	math::Vec3 a;
	EXPECT_FLOAT_EQ(a.x(), 0.f);
	EXPECT_FLOAT_EQ(a.y(), 0.f);
	EXPECT_FLOAT_EQ(a.z(), 0.f);

	math::Vec3 b(1.f, 2.f, 3.f);
	EXPECT_FLOAT_EQ(b.x(), 1.f);
	EXPECT_FLOAT_EQ(b.y(), 2.f);
	EXPECT_FLOAT_EQ(b.z(), 3.f);
}
TEST(Math_Vec3, Arithmetic) {
	math::Vec3 a(1.f, 2.f, 3.f);
	math::Vec3 b(2.f, 4.f, 6.f);
	EXPECT_EQ(a + b, math::Vec3(3.f, 6.f, 9.f));
	EXPECT_EQ(a - b, math::Vec3(-1.f, -2.f, -3.f));
	EXPECT_EQ(a * b, math::Vec3(2.f, 8.f, 18.f));
	EXPECT_EQ(a / b, math::Vec3(.5f, .5f, .5f));
	EXPECT_EQ(-a , math::Vec3(-1.f, -2.f, -3.f));
	EXPECT_EQ(-b, math::Vec3(-2.f, -4.f, -6.f));
}
TEST(Math_Vec3, LinearAlgebra) {
	math::Vec3 a(1.f, 2.f, 3.f);
	math::Vec3 b(2.f, 4.f, 6.f);
	math::Vec3 c(6.f, 3.f, 8.f);
	EXPECT_FLOAT_EQ(a.dot(a), 14.f);
	EXPECT_FLOAT_EQ(a.dot(b), 28.f);
	EXPECT_FLOAT_EQ(b.dot(b), 56.f);
	EXPECT_FLOAT_EQ(a.length(), 3.7416573867739413f);
	EXPECT_FLOAT_EQ(b.length(), 7.483314773547883f);
	EXPECT_FLOAT_EQ(a.length2(), a.dot(a));
	EXPECT_FLOAT_EQ(b.length2(), b.dot(b));
	EXPECT_EQ(a.cross(b), math::Vec3(0.f, 0.f, 0.f));
	EXPECT_EQ(a.cross(math::Vec3(3.f, 2.f, 1.f)), math::Vec3(-4.f, 8.f, -4.f));
	EXPECT_EQ(b.cross(c), math::Vec3(14.f, 20.f, -18.f));
	EXPECT_EQ(a.normalize(), b.normalize());
	EXPECT_EQ(a.normalize(), math::Vec3(0.2672612419124244f, 0.5345224838248488f, 0.8017837257372732f));
	EXPECT_EQ(c.normalize(), math::Vec3(0.5746957711326908f, 0.2873478855663454f, 0.7662610281769211f));
}
TEST(Math_Vec3, Miscellaneous) {
	math::Vec3 a(1.f, 8.f, 3.f);
	math::Vec3 b(2.f, 4.f, 6.f);
	EXPECT_EQ(a.min(b), b.min(a));
	EXPECT_EQ(a.max(b), b.max(a));
	EXPECT_EQ(a.min(b), math::Vec3(1.f, 4.f, 3.f));
	EXPECT_EQ(a.max(b), math::Vec3(2.f, 8.f, 6.f));
	EXPECT_TRUE(math::Vec3(0.0000001f, 0.f, 0.f).nearly_zero());
}


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
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(0.2f, 0.2f, 0.2f), Point(0.8f, 0.8f, 0.8f) };
	EXPECT_TRUE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_TRUE(b.is_inside(a));
	EXPECT_TRUE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Intersection, Disjoint) {
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(1.5f, 1.5f, 1.5f), Point(2.f, 2.f, 2.f) };
	EXPECT_FALSE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_FALSE(b.is_inside(a));
	EXPECT_FALSE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Intersection, Touching) {
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(1.f, 0.f, 0.f), Point(2.f, 1.f, 1.f) };
	EXPECT_FALSE(a.is_overlapping(b));
	EXPECT_FALSE(a.is_inside(b));
	EXPECT_FALSE(b.is_inside(a));
	EXPECT_FALSE(a.is_enclosing(b));
	EXPECT_FALSE(b.is_enclosing(a));
}
TEST(BoundingBox_Geometry, Overlap) {
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(0.5f, 0.5f, 0.5f), Point(1.5f, 1.5f, 1.5f) };
	EXPECT_FLOAT_EQ(a.area(), 1.f);
	EXPECT_FLOAT_EQ(a.margin(), 3.f);
	EXPECT_FLOAT_EQ(a.overlap(b), 0.125f);
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Inside) {
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(0.2f, 0.2f, 0.2f), Point(0.8f, 0.8f, 0.8f) };
	EXPECT_FLOAT_EQ(b.area(), 0.216f);
	EXPECT_FLOAT_EQ(b.margin(), 1.8f);
	EXPECT_FLOAT_EQ(b.overlap(a), b.area());
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Disjoint) {
	BoundingBox a{ Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f) };
	BoundingBox b{ Point(1.5f, 1.5f, 1.5f), Point(2.f, 2.f, 2.f) };
	EXPECT_FLOAT_EQ(b.area(), 0.125f);
	EXPECT_FLOAT_EQ(b.margin(), 1.5f);
	EXPECT_FLOAT_EQ(b.overlap(a), 0.f);
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}
TEST(BoundingBox_Geometry, Real) {
	BoundingBox a{ Point(-1.23428202f, -0.985212982f, -0.565617025f), Point(1.16378295f, 0.548205018f, 0.691652000f) };
	BoundingBox b{ Point(-1.21273303f, 0.519062996f, -0.932524025f), Point(0.356427014f, 1.31115603f, 0.387724012f) };
	BoundingBox c{ Point(-0.568542f,0.886272f,0.005542f), Point(-0.533288f,0.965194f,0.060187f) };
	EXPECT_FALSE(a.is_overlapping(c));
	EXPECT_TRUE(b.is_overlapping(c));
	EXPECT_LT(a.overlap(c), b.overlap(c));
	EXPECT_FLOAT_EQ(a.overlap(b), b.overlap(a));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
