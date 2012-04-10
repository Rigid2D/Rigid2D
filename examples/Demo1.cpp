#include "Demo1.h"

using namespace Rigid2D;


Demo1::Demo1(QWidget *parent)
      : DemoBase(parent)
{
	// Init RigidBodySystem
	rigidBodySystem = new RigidBodySystem();

  // Create a spring force for the mouse
  mouseForce = new Force(mouseSpringForce, userData_mouseForce);

	// Init sample rigid body;
  Real vertex_array[12] = {-5, 5, 
                          0, 7,
                          5, 4,
                          5, -5,
                          0, -7,
                          -4, -4};
  Real mass = 10;

  body1 = new RigidBody(Vector2(0, 0), Vector2(0,0), mass, vertex_array, 6);
  body2 = new RigidBody(Vector2(15, 10), Vector2(0,0), mass, vertex_array, 4);

	// Add bodies to rigidBodySystem
	rigidBodySystem->addRigidBody(body1);
	rigidBodySystem->addRigidBody(body2);

  userData_mouseForce[0] = 0;
  userData_mouseForce[1] = 0;
  userData_mouseForce[2] = 1000;  // Spring constant ks
  userData_mouseForce[3] = 100;  // Damping constant kd
  rbActedOn = NULL;
}


Demo1::~Demo1()
{
	delete rigidBodySystem;
  delete mouseForce;
	delete body1;
  delete body2;
}


void Demo1::paintGL()
{
  DemoBase::paintGL();

  // Do drawing here!
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Draw body1
  glPushMatrix();
  AABB *bb = body1->getWorldBB();
  if (body1->bp_isIntersecting()) {
    glColor3ub(180,50,50);
  } else {
    glColor3ub(50,50,180);
  }
  glBegin(GL_QUADS);
    glVertex2f(bb->minVertex_.x-0.2, bb->minVertex_.y-0.2);
    glVertex2f(bb->minVertex_.x-0.2, bb->maxVertex_.y+0.2);
    glVertex2f(bb->maxVertex_.x+0.2, bb->maxVertex_.y+0.2);
    glVertex2f(bb->maxVertex_.x+0.2, bb->minVertex_.y-0.2);
  glEnd();
  glTranslatef(body1->getPosition()[0], body1->getPosition()[1], 0);
  glColor3ub (255, 255, 255);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  const Vector2 *vertices = body1->getVertices();
  Real num_vertices = body1->getNumVertices();
  glBegin(GL_POLYGON);
    for (unsigned i = 0; i < num_vertices; i++) {
      glVertex2f(vertices[i].x, vertices[i].y);
    }
  glEnd();
  glPopMatrix();

  // Draw body2
  glPushMatrix();
  bb = body2->getWorldBB();
  if (body2->bp_isIntersecting()) {
    glColor3ub(180,50,50);
  } else {
    glColor3ub(50,50,180);
  }
  glBegin(GL_QUADS);
    glVertex2f(bb->minVertex_.x-0.2, bb->minVertex_.y-0.2);
    glVertex2f(bb->minVertex_.x-0.2, bb->maxVertex_.y+0.2);
    glVertex2f(bb->maxVertex_.x+0.2, bb->maxVertex_.y+0.2);
    glVertex2f(bb->maxVertex_.x+0.2, bb->minVertex_.y-0.2);
  glEnd();
  glTranslatef(body2->getPosition()[0], body2->getPosition()[1], 0);
  glColor3ub (255, 255, 255);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  vertices = body2->getVertices();
  num_vertices = body2->getNumVertices();
  glBegin(GL_POLYGON);
    for (unsigned i = 0; i < num_vertices; i++) {
      glVertex2f(vertices[i].x, vertices[i].y);
    }
  glEnd();
  glPopMatrix();

  // Draw the spring as a line
  if (rbActedOn != NULL) {
    glBegin(GL_LINE);
      glColor3ub(50, 200, 50);
      glVertex2f(userData_mouseForce[0], userData_mouseForce[1]);
      glVertex2f(rbActedOn->getPosition()[0], rbActedOn->getPosition()[1]);
    glEnd();
  }

  // Update ALL THE THINGS!! (unless paused)
	if (!paused) {
    rigidBodySystem->update();
  }
}


void Demo1::mousePressEvent(QMouseEvent *event) 
{
  makeCurrent();
  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  GLfloat winX, winY, winZ;
  GLdouble posX, posY, posZ;

  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );

  winX = event->x();
  winY = viewport[3] - event->y();
  glReadPixels( winX, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

  gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

  // set one end of mouse spring
  userData_mouseForce[0] = posX;
  userData_mouseForce[1] = posY;

  // transform mouse position to match RB
  GLdouble posRb1X = posX - body1->getPosition()[0];
  GLdouble posRb1Y = posY - body1->getPosition()[1];
  GLdouble posRb2X = posX - body2->getPosition()[0];
  GLdouble posRb2Y = posY - body2->getPosition()[1];

  if (body1->pointIsInterior(posRb1X, posRb1Y))
  {
    rbActedOn = body1;
    body1->addForce(mouseForce);
  } 
  else if (body2->pointIsInterior(posRb2X, posRb2Y))
  {
    rbActedOn = body2;
    body2->addForce(mouseForce);
  }
}


void Demo1::mouseReleaseEvent(QMouseEvent *event) 
{
  if (rbActedOn != NULL) 
  {
    rbActedOn->removeForce(mouseForce);
    rbActedOn = NULL;
  }
}


void Demo1::mouseMoveEvent(QMouseEvent *event) 
{
  if (rbActedOn != NULL) {
    makeCurrent();
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    QPoint pos = this->mapFromGlobal(QCursor::pos());
    winX = pos.x();
    winY = viewport[3] - pos.y();
    glReadPixels( winX, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

    userData_mouseForce[0] = posX;
    userData_mouseForce[1] = posY;
  }
}


void Demo1::keyPressEvent(QKeyEvent *event)
{

}

