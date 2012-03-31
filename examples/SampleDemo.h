#ifndef DEMO_BASE_H
#define DEMO_BASE_H

#include <QGLWidget>
#include <QTime>
#include <QTimer>
#include <iostream>
#include "Rigid2D.h"

class DemoBase : public QGLWidget
{
  Q_OBJECT

  public: 
    DemoBase(QWidget *parent = NULL);
    ~DemoBase();
    int getFps();
    void pause();

  signals:
    void fpsChanged(int value);

  public slots:
    void paintGL();
    void togglePause();

  protected:
    QTimer *animationTimer;
    QTime *fpsTimer;
    int frameCount;
    int fps;
    bool paused;

  protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void calculateFps();
};

#endif
