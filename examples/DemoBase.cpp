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
  animationTimer->start(1000/100);     // time per frame

  fpsTimer = new QTime();
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
  if (w >= h) {
    gluOrtho2D(-50 * (GLfloat)w/h, 50 * (GLfloat)w/h, -50, 50);
  } else {
    gluOrtho2D(-50, 50, -50 * (GLfloat)h/w, 50 * (GLfloat)h/w);
  }

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
