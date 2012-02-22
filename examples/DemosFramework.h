#ifndef DEMOS_FRAMEWORK_H
#define DEMOS_FRAMEWORK_H

#include <Qt/qapplication.h>
#include <Qt/qframe.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QFrame>
#include <QLabel>
#include <QGLWidget>
#include <QPushButton>
#include "SampleDemo.h"

class DemosFramework : public QObject
{
  Q_OBJECT

  public:
    DemosFramework();
    ~DemosFramework();

  public slots:
    void restartDemo();


  private:
    QFrame *MainFrame;
    QHBoxLayout *MainLayout;

    QVBoxLayout *SelectionLayout;
    QVBoxLayout *GLLayout;
    QVBoxLayout *DemoLayout;

    QTreeWidget *SelectionTree;
    
    QPushButton *StartButton;
    QPushButton *PauseButton;
    QPushButton *RestartButton;
    SampleDemo *CurrentDemo;

    QHBoxLayout *FPSLayout;
    QLabel *FPSTextLabel;
    QLabel *FPSNumLabel;
};

#endif
