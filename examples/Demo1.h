#ifndef DEMO1_H
#define DEMO1_H

#include "DemoBase.h"
#include "ForceFunctions.h"
#include <QMouseEvent>

class Demo1 : public DemoBase
{
  Q_OBJECT

  public: 
    Demo1(QWidget *parent = NULL);
    ~Demo1();

  public slots:
    void paintGL();

  protected:
    Rigid2D::RigidBodySystem *rigidBodySystem;
    Rigid2D::RigidBody *body;

    // mouse coordinates for the mouse force object (x, y) 
    // and spring constants (strength, damp)
    Rigid2D::Force *mouseForce;
    Rigid2D::Real userData_mouseForce[4];
    Rigid2D::RigidBody *rbActedOn;

  protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
};

#endif
