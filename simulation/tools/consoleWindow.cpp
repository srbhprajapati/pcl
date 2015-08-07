#include <gl/glew.h>
#include <QtWidgets>
#include "consoleWindow.h"
#include "ui_consoleWindow.h"
#include <iostream>

//using namespace pcl::simulation;


ConsoleWindow::ConsoleWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ConsoleWindow)
{
    ui->setupUi(this);
}

ConsoleWindow::~ConsoleWindow()
{
    delete ui;
}


