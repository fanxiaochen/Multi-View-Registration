#include <QApplication>

#include "main_window.h"

int main(int argc, char *argv[])
{
  // Make Xlib and GLX thread safe under X11
  QApplication::setAttribute(Qt::AA_X11InitThreads);
  QApplication application(argc, argv);

  MainWindow main_window;
  main_window.showMaximized();

  return application.exec();
}

