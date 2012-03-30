#ifndef SAMPLE_DEMO_H
#define SAMPLE_DEMO_H

#include <QGLWidget>
#include <QTime>
#include <QTimer>
#include <iostream>
#include "Rigid2D.h"

class SampleDemo : public QGLWidget
{
  Q_OBJECT

  public: 
    SampleDemo(QWidget *parent = NULL);
    ~SampleDemo();
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
    Rigid2D::RigidBodySystem *rigidBodySystem;
    Rigid2D::RigidBody *body;
    Rigid2D::Force *mouseForce;
    // mouse coordinates for the mouse force object (x, y) 
    // and spring constants (strength, damp)
    Rigid2D::Real userData_mouseForce[4];

  protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void calculateFps();
};

#endif
