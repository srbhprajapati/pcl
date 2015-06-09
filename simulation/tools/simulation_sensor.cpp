/**
 * Demo program for sensor simulation 
 * A virtual sensor generates depth map for a scene.
 * Scene in the demo are generated in the Fragment Shader using Distance Field.
 * Two Shaders:
 *	Viewport 1: Display the scene for which the sensor data needs to be generated.
 *	Viewport 2: Sensor simulated depth map for the scene.
 */


//TODO: 
/*
1. use shader to display the same scene
2. Produce values in the offset buffer (make different function for initial steps)
3. give two texture to fragment shader
4. Add two textures in the fragment shader (reformat the texel vaues so as to add both the textures)
5. produce the result of onto a screen
6. make it using 4 views

use a bigger scene

//sampling 
6. 64 rays 360 degree velodyne (first CPU then GPU)


........................................
generate different texture of reaal depth values
pass the scene a set of vertices equal to width x height
dont use the vertices real values insteal create new values from depth textures
 adn finish ta job on monday..!!

*/

#include <stdlib.h>
#include "applicationwindow.h"


#include <iostream>

#include <QApplication>
#include <QDesktopWidget>


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    ApplicationWindow window;
    window.resize(window.sizeHint());
    int desktopArea = QApplication::desktop()->width() *
                     QApplication::desktop()->height();
    int widgetArea = window.width() * window.height();

    window.setWindowTitle("OpenGL with Qt");

    if (((float)widgetArea / (float)desktopArea) < 0.75f)
        window.show();
    else
        window.showMaximized();
    return app.exec();
}
