#include <QApplication>
#include <QDesktopWidget>
#include "consoleWindow.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

	ConsoleWindow window;
    window.resize(window.sizeHint());
    int desktopArea = QApplication::desktop()->width() *
                     QApplication::desktop()->height();
    int widgetArea = window.width() * window.height();

    window.setWindowTitle("Laser Sensor Console Window");

					/*
    if (((float)widgetArea / (float)desktopArea) < 0.75f)
        window.show();
    else
        window.showMaximized();
	 				  */
    return app.exec();
}


