#include "DemosFramework.h"

int main(int argc, char **argv)
{
  QApplication App(argc, argv);
  DemosFramework *demos = new DemosFramework;
  return App.exec();
}
