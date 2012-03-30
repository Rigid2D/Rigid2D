#include "SampleDemo.h"
#include "ForceFunctions.h"
#include "Common/MathUtils.h"
#include <QMouseEvent>
#include <QCursor>

using namespace Rigid2D;

SampleDemo::SampleDemo(QWidget *parent)
        : QGLWidget(parent) 
{
  setMouseTracking(true);
  setAutoBufferSwap(true);
 
  animationTimer = new QTimer(this);
  connect(animationTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
  animationTimer->start(0);

  fpsTimer = new QTime;
  fpsTimer->start();
  frameCount = 0;

	// Init RigidBodySystem
	rigidBodySystem = new RigidBodySystem();

  // Create a spring force for the mouse
  mouseForce = new Force(mouseSpringForce, userData_mouseForce);

	// Init sample rigid body;
  Real vertex_array[8] = {-5, 5, 5, 5,
                          5, -5, -5, -5};
  body = new RigidBody(Vector2(5, 0), Vector2(0,0), 10.0, vertex_array, 4);
  body->addForce(mouseForce);

	// Add body and force to rigidBodySystem
	rigidBodySystem->addRigidBody(body);

  //userData_mouseForce[0] = -15;
  //userData_mouseForce[1] = -15;
  userData_mouseForce[2] = 1;
  userData_mouseForce[3] = 4;

	paused = false;
}

SampleDemo::~SampleDemo()
{
	delete animationTimer;
	delete fpsTimer;
	delete rigidBodySystem;
	delete body;
}

void SampleDemo::initializeGL()
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);

  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
}

void SampleDemo::resizeGL(int w, int h)
{
  h = h?h:1;

  glViewport( 0, 0, (GLint)w, (GLint)h );

  //std::cout << "W " << w << "\n";
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //gluPerspective(45.0f,(GLfloat)w / (GLfloat)h, 0.1f, 1000.0f);
  gluOrtho2D(-50, 50, -50, 50);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void SampleDemo::togglePause()
{
	paused = !paused;
}

void SampleDemo::paintGL()
{
  calculateFps();

  // Do drawing here!
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  //glTranslatef(0, 0, -2);
  glPushMatrix();
  glTranslatef(body->getPosition()[0], body->getPosition()[1], 0);
  glColor3f (1, 1, 1);
  glVertexPointer(2, GL_FLOAT, 0, body->getVertexArray());
  glDrawArrays(GL_POLYGON, 0, body->getVertexCount());
  glPopMatrix();

  std::cout.precision(3);


  // Draw the spring as a line
  glBegin(GL_LINE);
    glVertex2f(userData_mouseForce[0], userData_mouseForce[1]);
    glVertex2f(body->getPosition()[0], body->getPosition()[1]);
  glEnd();

  // Update ALL THE THINGS!! (unless paused)
	if (!paused) {
    std::cout << "RB{" << body->getPosition()[0] << " " << body->getPosition()[1] << "}\n";
    rigidBodySystem->update();
  }
}

void SampleDemo::mousePressEvent(QMouseEvent *event) 
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
  /*
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
    //std::cout << posX << "   " << winY << std::endl;
    //std::cout << pos.x() << "   " << pos.y() << std::endl;
    //userdata_mouseforce[0] = pos.x() * (100.0/572) - 50;
    //userdata_mouseforce[1] = -(pos.y() * (100.0/572) - 50);

    userData_mouseForce[0] = pos.x() * (100.0/572) - 50;
    userData_mouseForce[1] = -(pos.y() * (100.0/572) - 50);
  }*/
}

void SampleDemo::mouseMoveEvent(QMouseEvent *event) 
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
    //std::cout << posX << "   " << winY << std::endl;
    //std::cout << pos.x() << "   " << pos.y() << std::endl;
   // userData_mouseForce[0] = pos.x() * (100.0/700);
   // userData_mouseForce[1] = pos.y() * (100.0/700);
  }
}

void SampleDemo::keyPressEvent(QKeyEvent *event)
{

}

int SampleDemo::getFps()
{
  return fps;
}

void SampleDemo::calculateFps()
{
  // Figure out fps
  float elapsed = fpsTimer->elapsed();
  if (elapsed >= 1000) {
    fps = frameCount * (elapsed/1000);
    fpsTimer->restart();
    frameCount = 0;
    emit fpsChanged(fps);
  }
  frameCount++;
}
