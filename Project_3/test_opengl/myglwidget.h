#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QObject>
#include <QGLWidget>
#include <QTimer>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QtMath>


class MyGLWidget : public QGLWidget
{
    Q_OBJECT
public:
    MyGLWidget(QWidget *parent=0);
    ~MyGLWidget();
    bool Reversed;

protected:
    // Set up the rendering context, define display lists etc.:
   void initializeGL();
   // draw the scene:
   void paintGL();
   // setup viewport, projection etc.:
   void resizeGL (int width, int height);

public slots:
    void getQuaternions(const float, const float, const float, const float);
    void updateAll();

private:
    float q0, q1, q2, q3;
    QQuaternion rotation;
    QTimer *timer;
    float _q0_b, _q1_b, _q2_b, _q3_b;


};

#endif // MYGLWIDGET_H
