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
                           -4,-4,
                            0,-7,
                            5,-5,
                            5, 4,
                            0, 7};

  Real mass = 10;

  body1 = new RigidBody(6,                // number of vertices
                        vertex_array,
                        Vector2(0, 0),    // position
                        mass);

  body2 = new RigidBody(4,                // number of vertices
                        vertex_array,
                        Vector2(15, 10),  // position
                        mass);

  gravityForce = new Force(gravity);
  //body1->addForce(gravityForce);
  //body2->addForce(gravityForce);

	// Add bodies to rigidBodySystem
	rigidBodySystem->addRigidBody(body1);
	rigidBodySystem->addRigidBody(body2);

  userData_mouseForce[0] = 0;
  userData_mouseForce[1] = 0;
  userData_mouseForce[4] = 10;  // Spring constant ks
  userData_mouseForce[5] = 5;   // Damping constant kd
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
  AABB *bb2 = body1->getStaticBB();
  if (body1->np_isIntersecting()) {
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
  glRotatef(180/PI * body1->getOrientation(), 0, 0, 1);
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
  bb2 = body2->getStaticBB();
  if (body2->np_isIntersecting()) {
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
  glRotatef(180/PI * body2->getOrientation(), 0, 0, 1);
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
      glVertex2f(userData_mouseForce[2], userData_mouseForce[3]);
      glVertex2f(userData_mouseForce[0], userData_mouseForce[1]);
    glEnd();
  }

  // Draw all MTVs and contact edges/vertices
  std::vector<Contact*> * contacts = rigidBodySystem->getContacts();
  std::vector<Contact*>::iterator it;
  for (it = contacts->begin(); it < contacts->end(); it++) {
    Vector2 mtv = (*it)->mtv;
    Vector2 pos = (*it)->a->getPosition();
    //std::cout << (*it)->va_index;
    Vector2 v = (*it)->a->localToWorldTransform((*it)->a->getVertex((*it)->va_index));
    Vector2 edge_v0 = (*it)->b->localToWorldTransform((*it)->b->getVertex((*it)->vb1_index));
    Vector2 edge_v1 = (*it)->b->localToWorldTransform((*it)->b->getVertex((*it)->vb2_index));
    //body1->setPosition(body1->getPosition() + mtv);
    glTranslatef(0,0,1);
    glBegin(GL_LINE);
      glColor3ub(100, 100, 230);
      glVertex2f(0, 0);
      glVertex2f(mtv.x, mtv.y);
      // contact edge
      glColor3ub(250,50, 250);
      glVertex2f(edge_v0.x, edge_v0.y);
      glVertex2f(edge_v1.x, edge_v1.y);
    glEnd();
    glPointSize(4);
    glBegin(GL_POINT);
      glVertex2f(v.x, v.y);
    glEnd();
  }

  // Update ALL THE THINGS!! (unless paused)
	if (!paused) {
    // update mouseClicked position
    if (rbActedOn != NULL) {
      Vector2 pt = rbActedOn->prevWorldToCurrentLocalTransform(Vector2(userData_mouseForce[2], userData_mouseForce[3]));
      pt += rbActedOn->getPosition();
      userData_mouseForce[2] = pt.x;
      userData_mouseForce[3] = pt.y;
    }
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
  userData_mouseForce[2] = posX;
  userData_mouseForce[3] = posY;

  if (body1->pointIsInterior(posX, posY))
  {
    rbActedOn = body1;
    body1->addForce(mouseForce);
    Vector2 transformedPoint = body1->prevWorldToCurrentLocalTransform(Vector2(posX, posY));
  }
  else if (body2->pointIsInterior(posX, posY))
  {
    rbActedOn = body2;
    body2->addForce(mouseForce);
  } else {
    rbActedOn = NULL;
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

