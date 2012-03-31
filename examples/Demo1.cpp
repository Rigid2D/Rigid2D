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
  Real vertex_array[8] = {-5, 5, 5, 5,
                          5, -5, -5, -5};
  body = new RigidBody(Vector2(0, 0), Vector2(0,0), 10.0, vertex_array, 4);
  body->addForce(mouseForce);

	// Add body and force to rigidBodySystem
	rigidBodySystem->addRigidBody(body);

  userData_mouseForce[0] = 0;
  userData_mouseForce[1] = 0.01;
  userData_mouseForce[2] = 5;
  userData_mouseForce[3] = 4;

}


Demo1::~Demo1()
{
	delete rigidBodySystem;
  delete mouseForce;
	delete body;
}


void Demo1::paintGL()
{
  DemoBase::paintGL();

  // Do drawing here!
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glPushMatrix();
  glTranslatef(body->getPosition()[0], body->getPosition()[1], 0);
  glColor3f (1, 1, 1);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glVertexPointer(2, GL_FLOAT, 0, body->getVertexArray());
  glDrawArrays(GL_POLYGON, 0, body->getVertexCount());
  glPopMatrix();

  // Draw the spring as a line
  glBegin(GL_LINE);
    glColor3ub(50, 200, 50);
    glVertex2f(userData_mouseForce[0], userData_mouseForce[1]);
    glVertex2f(body->getPosition()[0], body->getPosition()[1]);
  glEnd();

  // Update ALL THE THINGS!! (unless paused)
	if (!paused) {
    //std::cout.precision(3);
    //std::cout << "RB{" << body->getPosition()[0] << " " << body->getPosition()[1] << "}\n";
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

  //if (body->pointIsInterior(posX, posY))
  {
    userData_mouseForce[0] = posX;
    userData_mouseForce[1] = posY;
  }
}


void Demo1::mouseMoveEvent(QMouseEvent *event) 
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

  QPoint pos = this->mapFromGlobal(QCursor::pos());
  winX = pos.x();
  winY = viewport[3] - pos.y();
  glReadPixels( winX, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

  gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

  {
    userData_mouseForce[0] = posX;
    userData_mouseForce[1] = posY;
  }
}


void Demo1::keyPressEvent(QKeyEvent *event)
{

}

