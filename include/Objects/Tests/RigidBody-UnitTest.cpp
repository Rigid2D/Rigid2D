#include "gtest/gtest.h"
#include "../RigidBody.h"
#include "Common/Vector2.h"
#include "Common/feq.h"
#include "RigidSettings.h"

using namespace Rigid2D;

//=============================================================================
// RBState Tests
//=============================================================================
class RBStateTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    pos1 = new Vector2 (1.0, 1.0);
    linMomentum1 = new Vector2 (1.0, 1.0);
    orientation1 = 180.0;
    angularMomentum1 = 22.2;

    s1 = new RBState(*pos1, *linMomentum1, orientation1, angularMomentum1);

    pos2 = new Vector2 (2.0, 2.0);
    linMomentum2 = new Vector2 (2.0, 2.0);
    orientation2 = -270.0;
    angularMomentum2 = -33.3;

    s2 = new RBState(*pos2, *linMomentum2, orientation2, angularMomentum2);
  }

  virtual void TearDown(){
    delete s1;
    delete s2;
    delete pos1;
    delete pos2;
    delete linMomentum1;
    delete linMomentum2;
  }

  RBState *s1;
  RBState *s2;
  Vector2 *pos1, *pos2;
  Vector2 *linMomentum1, *linMomentum2;
  Angle orientation1, orientation2;
  Real angularMomentum1, angularMomentum2;
};

TEST_F(RBStateTest, Construction){
  EXPECT_TRUE(*pos1 == s1->position);
  EXPECT_TRUE(*linMomentum1 == s1->linearMomentum);
  EXPECT_TRUE(orientation1 == s1->orientation);
  EXPECT_TRUE(angularMomentum1 == s1->angularMomentum);
}

TEST_F(RBStateTest, Operator_TimesEquals_Scalar){
  *s1 *= 2.0;
  EXPECT_TRUE(s1->position == (*pos1) * 2.0);
  EXPECT_TRUE(s1->linearMomentum == (*linMomentum1) * 2.0);
  EXPECT_TRUE(s1->orientation == (orientation1) * 2.0);
  EXPECT_TRUE(s1->angularMomentum == (angularMomentum1) * 2.0);
}

TEST_F(RBStateTest, Operator_DivideEquals_Scalar){
  *s1 /= 2.0;
  EXPECT_TRUE(s1->position == (*pos1) / 2.0);
  EXPECT_TRUE(s1->linearMomentum == (*linMomentum1) / 2.0);
  EXPECT_TRUE(s1->orientation == (orientation1) / 2.0);
  EXPECT_TRUE(s1->angularMomentum == (angularMomentum1) / 2.0);
}

TEST_F(RBStateTest, Operator_Add_RBState){
  RBState s3 = *s1 + *s2;
  EXPECT_TRUE(s3.position == (s1->position + s2->position));
  EXPECT_TRUE(s3.linearMomentum == (s1->linearMomentum + s2->linearMomentum));
  EXPECT_TRUE(s3.orientation == (s1->orientation + s2->orientation));
  EXPECT_TRUE(s3.angularMomentum == (s1->angularMomentum + s2->angularMomentum));
}

TEST_F(RBStateTest, Operator_Multiply_Scalar_Right){
  *s2 = *s1 * 2.0;
  EXPECT_TRUE(s2->position == (*pos1) * 2.0);
  EXPECT_TRUE(s2->linearMomentum == (*linMomentum1) * 2.0);
  EXPECT_TRUE(s2->orientation == (orientation1) * 2.0);
  EXPECT_TRUE(s2->angularMomentum == (angularMomentum1) * 2.0);
}

TEST_F(RBStateTest, Operator_Subtract_RBState){
  RBState s3 = *s1 - *s2;
  EXPECT_TRUE(s3.position == (s1->position - s2->position));
  EXPECT_TRUE(s3.linearMomentum == (s1->linearMomentum - s2->linearMomentum));
  EXPECT_TRUE(s3.orientation == (s1->orientation - s2->orientation));
  EXPECT_TRUE(s3.angularMomentum == (s1->angularMomentum - s2->angularMomentum));
}

TEST_F(RBStateTest, Operator_Multiply_Scalar_Left){
  *s2 = 2.0 * (*s1);
  EXPECT_TRUE(s2->position == (*pos1) * 2.0);
  EXPECT_TRUE(s2->linearMomentum == (*linMomentum1) * 2.0);
  EXPECT_TRUE(s2->orientation == (orientation1) * 2.0);
  EXPECT_TRUE(s2->angularMomentum == (angularMomentum1) * 2.0);
}

TEST_F(RBStateTest, Operator_Divide_Scalar_Right){
  *s2 = *s1 / 2.0;
  EXPECT_TRUE(s2->position == (*pos1) / 2.0);
  EXPECT_TRUE(s2->linearMomentum == (*linMomentum1) / 2.0);
  EXPECT_TRUE(s2->orientation == (orientation1) / 2.0);
  EXPECT_TRUE(s2->angularMomentum == (angularMomentum1) / 2.0);
}

TEST_F(RBStateTest, NormalizeOrientAngle){
  s1->orientation = TAU + 0.5;
  s1->normalizeOrientAngle();
  EXPECT_FLOAT_EQ(s1->orientation, 0.5);

  s1->orientation = TAU + 10.25;
  s1->normalizeOrientAngle();
  EXPECT_FLOAT_EQ(s1->orientation, 10.25 - TAU);

  s1->orientation = TAU;
  s1->normalizeOrientAngle();
  EXPECT_FLOAT_EQ(s1->orientation, 0.0);

  s1->orientation = TAU - 2*TAU;
  s1->normalizeOrientAngle();
  EXPECT_FLOAT_EQ(s1->orientation, 0.0);

  s1->orientation = TAU - 0.5;
  s1->normalizeOrientAngle();
  EXPECT_FLOAT_EQ(s1->orientation, TAU - 0.5);
}

//=============================================================================
// RigidBody Tests
//=============================================================================

TEST(RigidBodyTest, Creation_with_RealArray){
  Vector2 pos(1,1);
  Vector2 velocity(100,0);
  Real mass = 20;
  Angle orientation = 5.2;
  unsigned int num_vertices = 3;
  Vector2 const *vertices;

  // 3-4-5 triangle with one vertex at origin
  Real vertex_array[6] = {0,0, 3,0, 0,4};

  RigidBody rb(num_vertices, vertex_array, pos, mass, velocity, orientation);
  vertices = rb.getVertices();

  EXPECT_TRUE(rb.getPosition() == pos);
  EXPECT_TRUE(rb.getVelocity() == velocity);
  EXPECT_TRUE(rb.getNumVertices() == num_vertices);
  EXPECT_TRUE(rb.getLinearMomentum() == mass * velocity);
  EXPECT_FLOAT_EQ(rb.getMass(), mass);
  EXPECT_FLOAT_EQ(rb.getOrientation(), orientation);
  EXPECT_FLOAT_EQ(rb.getMomentOfInertia(), mass*(3.0*3.0 + 4.0*4.0 + 5.0*5.0) / 36.0);

  EXPECT_TRUE(vertices[0] == Vector2(0,0));
  EXPECT_TRUE(vertices[1] == Vector2(3,0));
  EXPECT_TRUE(vertices[2] == Vector2(0,4));
}

TEST(RigidBodyTest, Creation_with_Vector2Array){
  Vector2 pos(1,1);
  Vector2 velocity(100,0);
  Real mass = 20;
  Angle orientation = 5.2;
  unsigned int num_vertices = 3;
  Vector2 const *vOut;

  // 3-4-5 triangle with one vertex at origin
  Vector2 *vertices = new Vector2[num_vertices];

  vertices[0] = Vector2(0,0);
  vertices[1] = Vector2(3,0);
  vertices[2] = Vector2(0,4);


  RigidBody rb(num_vertices, vertices, pos, mass, velocity, orientation);
  vOut = rb.getVertices();

  EXPECT_TRUE(rb.getPosition() == pos);
  EXPECT_TRUE(rb.getVelocity() == velocity);
  EXPECT_TRUE(rb.getNumVertices() == num_vertices);
  EXPECT_TRUE(rb.getLinearMomentum() == mass * velocity);
  EXPECT_FLOAT_EQ(rb.getMass(), mass);
  EXPECT_FLOAT_EQ(rb.getOrientation(), orientation);
  EXPECT_FLOAT_EQ(rb.getMomentOfInertia(), mass*(3.0*3.0 + 4.0*4.0 + 5.0*5.0) / 36.0);

  EXPECT_TRUE(vOut[0] == Vector2(0,0));
  EXPECT_TRUE(vOut[1] == Vector2(3,0));
  EXPECT_TRUE(vOut[2] == Vector2(0,4));

  delete [] vertices;
}
