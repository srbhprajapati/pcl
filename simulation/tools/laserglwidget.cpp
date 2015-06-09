//#include "lasersensorthread.h"
#include "laserglwidget.h"
#include <iostream>
#include <QtWidgets>
#include <QtOpenGL>
#include <Windows.h>
#include <qcolordialog.h>
#include <QMouseEvent>



LaserGLWidget::LaserGLWidget(QWidget *parent) :
    QGLWidget(parent),
	QGLFunctions(),
	glt(*this)
{
    setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

    // Buffer swap is handled in the rendering thread
    setAutoBufferSwap(false);

    // start the rendering thread
    initRendering();
}

LaserGLWidget::~LaserGLWidget()
{
}


void LaserGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}



void LaserGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // modify scence variables and render the next frame
    GLfloat dx = (GLfloat)(event->x() - lastPos.x()) / width();
    GLfloat dy = (GLfloat)(event->y() - lastPos.y()) / height();

    if (true)
    {
        glt.setRotation(180 * dy, 180 * dx, 0.0);
        render();
    }
    else if (event->button() & Qt::RightButton)
    {
        glt.setRotation(180 * dy,0.0,180 * dx);
        render();
    }
    lastPos = event->pos();
}

void LaserGLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    // get the name of the clicked surface
    int face = glt.faceAtPosition(event->pos());
    if (face != -1)
    {
        QColor color = QColorDialog::getColor(glt.faceColors[face],
                                              this);
        if (color.isValid())
        {
            glt.faceColors[face] = color;
        }
    }
}

void LaserGLWidget::initRendering( )
{
    doneCurrent();
    context()->moveToThread((QThread*)&glt);

    // start the rendering thread
    glt.start();
    // wake the waiting render thread
    renderCondition().wakeAll();
}

void LaserGLWidget::finishRendering( )
{
    // request stopping
    glt.stop();
    // wake up render thread to actually perform stopping
    renderCondition().wakeAll();
    // wait till the thread has exited
    glt.wait();
}

void LaserGLWidget::closeEvent( QCloseEvent * _e )
{
    // request stopping
    finishRendering();
    // close the widget (base class)
    QGLWidget::closeEvent(_e);
}

void LaserGLWidget::paintEvent( QPaintEvent * )
{
   render();
}

void LaserGLWidget::resizeEvent( QResizeEvent * _e )
{
    // signal the rendering thread that a resize is needed
    glt.resizeViewport(_e->size());

    render();
}

void LaserGLWidget::lockGLContext( )
{
    // lock the render mutex for the calling thread
    renderMutex().lock();
    // make the render context current for the calling thread
    makeCurrent();
}

void LaserGLWidget::unlockGLContext( )
{
    // release the render context for the calling thread
    // to make it available for other threads
    doneCurrent();
    // unlock the render mutex for the calling thread
    renderMutex().unlock();
}

void LaserGLWidget::render( )
{
    // let the wait condition wake up the waiting thread
    renderCondition().wakeAll();
}

void LaserGLWidget::start_laser(int azm, int scan)
{
	std::cout<<"Azimuth : "<<azm<<std::endl;
	std::cout<<"ScanLine : "<<scan<<std::endl;



	glt.start_laser_sensor(azm, scan);
		
}

void LaserGLWidget::stop_laser_sensor()
{
	glt.stop_laser_sensor();
}

void LaserGLWidget::changeSensorPosition(float x, float y, float z)
{
	glt.changeSensorPosition(x, y, z);
}

void LaserGLWidget::changeScanMode(int mode, int Azimuthal, int Scanline, float upper_bound, float lower_bound, float lAngular, float rAngular)
{
	glt.changeScanMode(mode, Azimuthal, Scanline, upper_bound, lower_bound, lAngular, rAngular);
}

QWaitCondition & LaserGLWidget::renderCondition( )
{
    return(render_condition);
}

QMutex & LaserGLWidget::renderMutex()
{
    return(render_mutex);
}

void LaserGLWidget::changeModel(QString path)
{
	glt.changeModel(path);
}


