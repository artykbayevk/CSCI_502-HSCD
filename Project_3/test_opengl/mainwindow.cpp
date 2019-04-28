#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // do not change this!
    ui->setupUi(this);
    connect(&mThread, SIGNAL(setQuats(float,float,float,float)),
            &mGL, SLOT(getQuaternions(float,float,float,float)));
}

MainWindow::~MainWindow()
{
    // do not change this!
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    //connect and show: do not change this!

    // connect thread
    mThread.run_flag = true;
    mThread.start();

    // start opengl to visualize
    if(!mGL.isVisible())
    {
        mGL.resize(640, 480);
        mGL.show();
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    //disconnect: do not change this!

    // connect thread
    mThread.run_flag = false;
    mThread.wait();
}
