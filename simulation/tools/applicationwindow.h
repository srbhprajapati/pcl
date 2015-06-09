#ifndef APPLICATIONWINDOW_H
#define APPLICATIONWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QtNetwork/QUdpSocket>


namespace Ui {
class ApplicationWindow;
}

class ApplicationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ApplicationWindow(QWidget *parent = 0);
    ~ApplicationWindow();

protected:
    void keyPressEvent(QKeyEvent *event);


	
public slots:
    //reads pending datagram sent by another Program at
    // a particular port
    void readPendingDatagrams();


private slots:
	void on_startLaserButton_clicked();
	void on_stopLaserButton_clicked();
	void on_fullScanModeButton_clicked();
	void on_boundedElevationModeButton_clicked();
	void on_regionScanModeButton_clicked();
	void on_LaserPositionButton_clicked();
	void on_openAction_clicked();
	
private:
    Ui::ApplicationWindow *ui;

	QUdpSocket *udpSocket;
};

#endif // APPLICATIONWINDOW_H
