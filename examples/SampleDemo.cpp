#include "DemoBase.h"
#include "ForceFunctions.h"
#include "Common/MathUtils.h"
#include <QMouseEvent>
#include <QCursor>

using namespace Rigid2D;

DemoBase::DemoBase(QWidget *parent)
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

	paused = false;
}

DemoBase::~DemoBase()
{
	delete animationTimer;
	delete fpsTimer;
}

void DemoBase::initializeGL()
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);

  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
}

void DemoBase::resizeGL(int w, int h)
{
  h = h?h:1;

  glViewport( 0, 0, (GLint)w, (GLint)h );

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-50, 50, -50, 50);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void DemoBase::togglePause()
{
	paused = !paused;
}

void DemoBase::paintGL()
{
  calculateFps();
}

void DemoBase::mousePressEvent(QMouseEvent *event) 
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

void DemoBase::mouseMoveEvent(QMouseEvent *event) 
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

void DemoBase::keyPressEvent(QKeyEvent *event)
{

}

int DemoBase::getFps()
{
  return fps;
}

void DemoBase::calculateFps()
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
