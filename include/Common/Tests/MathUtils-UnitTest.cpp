#include "../MathUtils.h"
#include "gtest/gtest.h"
#include <new>

using namespace Rigid2D;

//=============================================================================
// SignedArea Tests
//=============================================================================
TEST(SignedAreaTest, Vertice_Count_0){
  unsigned int num_vertices = 0;
  Vector2 *vertices = 0;

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);
}

TEST(SignedAreaTest, Vertice_Count_1){
  unsigned int num_vertices = 1;
  Vector2 *vertices = 0;

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);
}

TEST(SignedAreaTest, Vertice_Count_2){
  unsigned int num_vertices = 2;
  Vector2 *vertices = new Vector2 [num_vertices];
  vertices[0] = Vector2(3,2);
  vertices[1] = Vector2(1,1);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);

  delete [] vertices;
}

TEST(SignedAreaTest, Vertice_Count_3){
  unsigned int num_vertices = 3;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(5,0);
  vertices[2] = Vector2(3,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 10.0);

  delete [] vertices;
}

TEST(SignedAreaTest, Vertice_Count_3_Negative){
  unsigned int num_vertices = 3;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[2] = Vector2(0,0);
  vertices[1] = Vector2(5,0);
  vertices[0] = Vector2(3,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), -10.0);

  delete [] vertices;
}

TEST(SignedAreaTest, Vertice_Count_4){
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[0] = Vector2(1,1);
  vertices[1] = Vector2(4,1);
  vertices[2] = Vector2(4,4);
  vertices[3] = Vector2(1,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 9.0);

  delete [] vertices;
}

TEST(SignedAreaTest, Square_2x2){
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(2,0);
  vertices[2] = Vector2(2,2);
  vertices[3] = Vector2(0,2);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 4.0);

  delete [] vertices;
}

//=============================================================================
// Centroid Tests
//=============================================================================

TEST(CentroidTest, Vertice_Count_0_Throws_Error){
  unsigned int num_vertices = 0;
  Vector2 *vertices = new Vector2 [1];
  vertices[0] = Vector2(0, 0);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  delete [] vertices;
}

TEST(CentroidTest, Vertice_Count_1_Throws_Error){
  unsigned int num_vertices = 1;
  Vector2 *vertices = new Vector2 [1];
  vertices[0] = Vector2(0, 0);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  delete [] vertices;
}

TEST(CentroidTest, Vertice_Count_2_Throws_Error){
  unsigned int num_vertices = 2;
  Vector2 *vertices = new Vector2 [num_vertices];
  vertices[0] = Vector2(0, 0);
  vertices[1] = Vector2(1, 1);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  delete [] vertices;
}

TEST(CentroidTest, Vertice_Count_4){
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(4,0);
  vertices[2] = Vector2(4,4);
  vertices[3] = Vector2(0,4);

  Vector2 C = centroid(num_vertices, vertices);

  EXPECT_FLOAT_EQ(C.x, 2.0);
  EXPECT_FLOAT_EQ(C.y, 2.0);

  delete [] vertices;
}

TEST(CentroidTest, Vertice_Count_8){
  unsigned int num_vertices = 8;
  Vector2 *vertices = new Vector2 [num_vertices];

  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(2,0);
  vertices[2] = Vector2(4,0);
  vertices[3] = Vector2(4,2);
  vertices[4] = Vector2(4,4);
  vertices[5] = Vector2(2,4);
  vertices[6] = Vector2(0,4);
  vertices[7] = Vector2(0,2);

  Vector2 C = centroid(num_vertices, vertices);

  EXPECT_FLOAT_EQ(C.x, 2.0);
  EXPECT_FLOAT_EQ(C.y, 2.0);

  delete [] vertices;
}

//=============================================================================
// realArrayToVector2Array Tests
//=============================================================================
TEST(realArrayToVector2ArrayTest, Num_Vertices_0_Throws_Error){
  unsigned int num_vertices = 0;
  Real vIn[2] = {0,0}; // dummy var

  EXPECT_THROW(realArrayToVector2Array(num_vertices, vIn), InvalidParameterException);
}

TEST(realArrayToVector2ArrayTest, Num_Vertices_1){
  unsigned int num_vertices = 1;
  Real vIn[2] = {1, 1};
  Vector2 *vOut = realArrayToVector2Array(num_vertices, vIn);

  EXPECT_TRUE(vOut[0] == Vector2(1,1));

  delete [] vOut;
}

TEST(realArrayToVector2ArrayTest, Num_Vertices_2){
  unsigned int num_vertices = 2;
  Real vIn[4] = {0,1,2,3};
  Vector2 *vOut = realArrayToVector2Array(num_vertices, vIn);

  EXPECT_TRUE(vOut[0] == Vector2(0,1));
  EXPECT_TRUE(vOut[1] == Vector2(2,3));

  delete [] vOut;
}

TEST(realArrayToVector2ArrayTest, Num_Vertices_3){
  unsigned int num_vertices = 3;
  Real vIn[6] = {0,1,2,3,4,5};
  Vector2 *vOut = realArrayToVector2Array(num_vertices, vIn);

  EXPECT_TRUE(vOut[0] == Vector2(0,1));
  EXPECT_TRUE(vOut[1] == Vector2(2,3));
  EXPECT_TRUE(vOut[2] == Vector2(4,5));

  delete [] vOut;
}

//=============================================================================
// amoi_triangle Tests
//=============================================================================
TEST(amoi_triangleTest, SpecialTriangle_3_4_5){
	Vector2 v0(0,0);
	Vector2 v1(4,0);
	Vector2 v2(0,3);

	EXPECT_FLOAT_EQ(amoi_triangle(v0, v1, v2),
			(3.0*3.0 + 4.0*4.0 + 5.0*5.0) / 36.0);
}

TEST(amoi_triangleTest, SpecialTriangle_5_12_13){
	Vector2 v0(0,0);
	Vector2 v1(5,0);
	Vector2 v2(0,12);

	EXPECT_FLOAT_EQ(amoi_triangle(v0, v1, v2),
			(5.0*5.0 + 12.0*12.0 + 13.0*13.0) / 36.0);
}

TEST(amoi_triangleTest, IsoscelesTrianlge_13_13_10){
	Vector2 v0(0,0);
	Vector2 v1(10,0);
	Vector2 v2(5,12);

	EXPECT_FLOAT_EQ(amoi_triangle(v0, v1, v2),
			(10.0*10.0 + 13.0*13.0 + 13.0*13.0) / 36.0);
}

//=============================================================================
// momentOfInertia Tests
//=============================================================================
TEST(momentOfInertiaTest, Triangle_3_4_5){
  Real mass = 1;
  unsigned int num_vertices = 3;
  Vector2 *vertices = new Vector2[num_vertices];

  // 3-4-5 triangle
  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(3,0);
  vertices[2] = Vector2(0,4);

  Real result = momentOfInertia(num_vertices, vertices, mass);
  EXPECT_FLOAT_EQ(result, (3.0*3.0 + 4.0*4.0 + 5.0*5.0) / 36.0);

  delete [] vertices;
}

TEST(momentOfInertiaTest, Square_2x2){
  Real mass = 1;
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2[num_vertices];

  // 2x2 square
  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(2,0);
  vertices[2] = Vector2(2,2);
  vertices[3] = Vector2(0,2);

  Real result = momentOfInertia(num_vertices, vertices, mass);

	// 2(m*a^2)/3 with a = 1, m = 1.
  EXPECT_FLOAT_EQ(result, 2.0/3.0);

  delete [] vertices;
}

TEST(momentOfInertiaTest, Rectangle_2x6){
  Real mass = 1;
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2[num_vertices];

  // 2x6 rectangle
  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(2,0);
  vertices[2] = Vector2(2,6);
  vertices[3] = Vector2(0,6);

  Real result = momentOfInertia(num_vertices, vertices, mass);

	// m*(a^2 + b^2)/3 with 2a = 6, 2b = 2, m = 1.
  EXPECT_FLOAT_EQ(result, 10.0/3.0);

  delete [] vertices;
}

TEST(momentOfInertiaTest, Rectangle_2x6_heavy){
  Real mass = 100;
  unsigned int num_vertices = 4;
  Vector2 *vertices = new Vector2[num_vertices];

  // 2x6 rectangle
  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(2,0);
  vertices[2] = Vector2(2,6);
  vertices[3] = Vector2(0,6);

  Real result = momentOfInertia(num_vertices, vertices, mass);

	// m*(a^2 + b^2)/3 with 2a = 6, 2b = 2, m = 100.
  EXPECT_FLOAT_EQ(result, 100.0*10.0/3.0);

  delete [] vertices;
}

//=============================================================================
// ClosestPtPointSegment Tests
//=============================================================================
TEST(ClosestPtPointSegment, outside_interval_closest_to_a){
  Vector2 a = Vector2(0,0);
  Vector2 b = Vector2(1,0);
  Vector2 c = Vector2(-1,1);
  Vector2 d;

  ClosestPtPointSegment(a, b, c, d);
  EXPECT_TRUE(d == a);
}

TEST(ClosestPtPointSegment, outside_interval_closest_to_b){
  Vector2 a = Vector2(0,0);
  Vector2 b = Vector2(1,0);
  Vector2 c = Vector2(2,1);
  Vector2 d;

  ClosestPtPointSegment(a, b, c, d);
  EXPECT_TRUE(d == b);
}

TEST(ClosestPtPointSegment, within_interval_above){
  Vector2 a = Vector2(0,0);
  Vector2 b = Vector2(1,0);
  Vector2 c = Vector2(0.5,3);
  Vector2 d;

  ClosestPtPointSegment(a, b, c, d);
  EXPECT_TRUE(d == Vector2(0.5,0));
}

TEST(ClosestPtPointSegment, within_interval_below){
  Vector2 a = Vector2(0,0);
  Vector2 b = Vector2(1,0);
  Vector2 c = Vector2(0.5, -3);
  Vector2 d;

  ClosestPtPointSegment(a, b, c, d);
  EXPECT_TRUE(d == Vector2(0.5,0));
}
