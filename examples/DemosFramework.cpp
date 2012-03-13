#include "DemosFramework.h"

DemosFramework::DemosFramework()
  : QObject()
{
  // Create the main frame
  MainFrame = new QFrame;
  MainFrame->setWindowTitle("Rigid2D Examples");
  MainFrame->resize(800, 800);
  MainLayout = new QHBoxLayout;
  MainFrame->setLayout(MainLayout);

  // Create layouts for selection and demo
  SelectionLayout = new QVBoxLayout;
  DemoLayout = new QVBoxLayout;
  DemoLayout->setAlignment(Qt::AlignLeft);
  MainLayout->addLayout(SelectionLayout);
  MainLayout->addLayout(DemoLayout);

  // Create TreeWidget in the selection layout
  SelectionTree = new QTreeWidget();
  SelectionTree->setMaximumWidth(200);
  SelectionTree->setColumnCount(1);
  SelectionTree->setHeaderLabel(QString("Examples"));
  // DemosHeader
  QTreeWidgetItem *demosHeader = new QTreeWidgetItem((QTreeWidget*)0, 
      QStringList(QString("Demos")));
  SelectionTree->addTopLevelItem(demosHeader);
  SelectionTree->expandItem(demosHeader);
  QTreeWidgetItem *demo1 = new QTreeWidgetItem((QTreeWidget*)0, 
      QStringList(QString("Demo 1")));
  demosHeader->addChild(demo1);
  // BenchmarksHeader
  QTreeWidgetItem *benchmarksHeader = new QTreeWidgetItem((QTreeWidget*)0, 
      QStringList(QString("Benchmarks")));
  SelectionTree->addTopLevelItem(benchmarksHeader);
  SelectionLayout->addWidget(SelectionTree);

  // DemoToolBar layer (top of demo layer)
  QHBoxLayout *DemoToolBarLayout = new QHBoxLayout;
  DemoLayout->addLayout(DemoToolBarLayout);

  // Create an OGLDemo
  GLLayout = new QVBoxLayout;
  CurrentDemo = new SampleDemo;
  CurrentDemo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  GLLayout->addWidget(CurrentDemo);

  DemoLayout->addLayout(GLLayout);

  // DemoToolBar Buttons
  StartButton = new QPushButton("&Start");
  DemoToolBarLayout->addWidget(StartButton);
  PauseButton = new QPushButton("&Pause");
  DemoToolBarLayout->addWidget(PauseButton);
  RestartButton = new QPushButton("&Restart");
  DemoToolBarLayout->addWidget(RestartButton);
  
  // Pause button
  QObject::connect(PauseButton, SIGNAL(clicked()), CurrentDemo, SLOT(togglePause()));

  // Restart button
  QObject::connect(RestartButton, SIGNAL(clicked()), this, SLOT(restartDemo()));

  // Create and connect an fps label display
  FPSLayout = new QHBoxLayout;
  FPSLayout->setAlignment(Qt::AlignRight);
  FPSTextLabel = new QLabel("Fps: ");
  FPSNumLabel = new QLabel;
  QObject::connect(CurrentDemo, SIGNAL(fpsChanged(int)), FPSNumLabel, SLOT(setNum(int)));
  FPSLayout->addWidget(FPSTextLabel);
  FPSLayout->addWidget(FPSNumLabel);
  DemoLayout->addLayout(FPSLayout);

  MainFrame->show();
}

DemosFramework::~DemosFramework()
{
  // delete a bunch of stuff here
  // not really necessary though cause kernel
}

//TODO: Make a method for loading a demo - restartDemo should call that method.
void DemosFramework::restartDemo()
{
  delete CurrentDemo;
  CurrentDemo = new SampleDemo();
  CurrentDemo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  GLLayout->addWidget(CurrentDemo);
  QObject::connect(PauseButton, SIGNAL(clicked()), CurrentDemo, SLOT(togglePause()));
  QObject::connect(CurrentDemo, SIGNAL(fpsChanged(int)), FPSNumLabel, SLOT(setNum(int)));
}
