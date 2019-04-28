#include "mythread.h"
#include <QDebug>
#include <qmath.h>
#include <QQuaternion>


MyThread::MyThread(QObject *parent)
    :QThread(parent)
{

}

MyThread::~MyThread()
{

}


void MyThread::run()
{

     // ADD CODE HERE: add here connection setup for TCP/IP

    while(run_flag)
    {
        // ADD CODE HERE: this is infinite loop, you need parse here the TCP/IP read buffer


        // SAMPLE CODE FOR RANDOM QUATERNIONS: please, delete these codes, it is only for demonstration
        q0 = float(qrand() % 100);
        q1 = float(qrand() % 100);
        q2 = float(qrand() % 100);
        q3 = float(qrand() % 100);

        QQuaternion temp_q = QQuaternion(q0,q1,q2,q3);
        temp_q.normalize();

        emit setQuats(temp_q.scalar(), temp_q.x(),temp_q.y(), temp_q.z());
        msleep(20);
    }

    // ADD CODE HERE: safely disconnect from TCP/IP
}
