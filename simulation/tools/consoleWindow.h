#ifndef _CONSOLEWINDOW_H_
#define _CONSOLEWINDOW_H_

#include <QMainWindow>
#include <QtNetwork/QUdpSocket>



namespace Ui {
	class ConsoleWindow;
}


class ConsoleWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ConsoleWindow(QWidget *parent = 0);
    ~ConsoleWindow();


private:
    Ui::ConsoleWindow *ui;

};


#endif //_CONSOLEWINDOW_H_