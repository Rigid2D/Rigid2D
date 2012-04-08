#include "../MathUtils.h"
#include "gtest/gtest.h"
#include <new>

using namespace Rigid2D;

void free(unsigned int num_vertices, Vector2 **vertices) {
  for(unsigned i = 0; i < num_vertices; ++i) {
    delete vertices[i];
  }

  delete [] vertices;
}

//=============================================================================
// SignedArea Tests
//=============================================================================
TEST(SignedAreaTest, Vertice_Count_0){
  unsigned int num_vertices = 0;
  Vector2 **vertices = 0;

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);
}

TEST(SignedAreaTest, Vertice_Count_1){
  unsigned int num_vertices = 1;
  Vector2 **vertices = 0;

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);
}

TEST(SignedAreaTest, Vertice_Count_2){
  unsigned int num_vertices = 2;
  Vector2 **vertices = new Vector2 * [num_vertices];
  vertices[0] = new Vector2(3,2);
  vertices[1] = new Vector2(1,1);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 0.0);

  free(num_vertices, vertices);
}

TEST(SignedAreaTest, Vertice_Count_3){
  unsigned int num_vertices = 3;
  Vector2 **vertices = new Vector2 *[num_vertices];

  vertices[0] = new Vector2(0,0);
  vertices[1] = new Vector2(5,0);
  vertices[2] = new Vector2(3,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 10.0);

  free(num_vertices, vertices);
}

TEST(SignedAreaTest, Vertice_Count_3_Negative){
  unsigned int num_vertices = 3;
  Vector2 **vertices = new Vector2 *[num_vertices];

  vertices[2] = new Vector2(0,0);
  vertices[1] = new Vector2(5,0);
  vertices[0] = new Vector2(3,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), -10.0);

  free(num_vertices, vertices);
}

TEST(SignedAreaTest, Vertice_Count_4){
  unsigned int num_vertices = 4;
  Vector2 **vertices = new Vector2 *[num_vertices];

  vertices[0] = new Vector2(1,1);
  vertices[1] = new Vector2(4,1);
  vertices[2] = new Vector2(4,4);
  vertices[3] = new Vector2(1,4);

  EXPECT_FLOAT_EQ(signedArea(num_vertices, vertices), 9.0);

  free(num_vertices, vertices);
}

//=============================================================================
// Centroid Tests
//=============================================================================

TEST(CentroidTest, Vertice_Count_0_Throws_Error){
  unsigned int num_vertices = 0;
  Vector2 **vertices = new Vector2 * [1];
  vertices[0] = new Vector2(0, 0);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  free(1, vertices);
}

TEST(CentroidTest, Vertice_Count_1_Throws_Error){
  unsigned int num_vertices = 1;
  Vector2 **vertices = new Vector2 * [1];
  vertices[0] = new Vector2(0, 0);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  free(num_vertices, vertices);
}

TEST(CentroidTest, Vertice_Count_2_Throws_Error){
  unsigned int num_vertices = 2;
  Vector2 **vertices = new Vector2 * [num_vertices];
  vertices[0] = new Vector2(0, 0);
  vertices[1] = new Vector2(1, 1);

  EXPECT_THROW(centroid(num_vertices, vertices), InvalidParameterException);

  free(num_vertices, vertices);
}

TEST(CentroidTest, Vertice_Count_4){
  unsigned int num_vertices = 4;
  Vector2 **vertices = new Vector2 *[num_vertices];

  vertices[0] = new Vector2(0,0);
  vertices[1] = new Vector2(4,0);
  vertices[2] = new Vector2(4,4);
  vertices[3] = new Vector2(0,4);

  Vector2 C = centroid(num_vertices, vertices);

  EXPECT_FLOAT_EQ(C.x, 2.0);
  EXPECT_FLOAT_EQ(C.y, 2.0);

  free(num_vertices, vertices);
}

TEST(CentroidTest, Vertice_Count_8){
  unsigned int num_vertices = 8;
  Vector2 **vertices = new Vector2 *[num_vertices];

  vertices[0] = new Vector2(0,0);
  vertices[1] = new Vector2(2,0);
  vertices[2] = new Vector2(4,0);
  vertices[3] = new Vector2(4,2);
  vertices[4] = new Vector2(4,4);
  vertices[5] = new Vector2(2,4);
  vertices[6] = new Vector2(0,4);
  vertices[7] = new Vector2(0,2);

  Vector2 C = centroid(num_vertices, vertices);

  EXPECT_FLOAT_EQ(C.x, 2.0);
  EXPECT_FLOAT_EQ(C.y, 2.0);

  free(num_vertices, vertices);
}