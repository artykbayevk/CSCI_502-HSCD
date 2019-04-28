#include "myglwidget.h"
#include <QDebug>


MyGLWidget::MyGLWidget(QWidget *parent)
    :QGLWidget(parent)
{
    q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
    timer = new QTimer(this);
    timer ->start(100);


    connect(timer, SIGNAL(timeout()),
            this, SLOT(updateAll()));

    qDebug() << "started";
}

MyGLWidget::~MyGLWidget()
{
}

void MyGLWidget::updateAll()
{
    updateGL();
}

void MyGLWidget::getQuaternions(const float a, const float b , const float c, const float d)
{
    q0 = a;
    q1 = b;
    q2 = c;
    q3 = d;
}


void MyGLWidget::initializeGL(){
    //activate the depth buffer
    glEnable(GL_DEPTH_TEST);
    //qDebug() << "init";

}


/*
 *  Sets up the OpenGL viewport, projection, etc. Gets called whenever the widget has been resized
 *  (and also when it is shown for the first time because all newly created widgets get a resize event automatically).
 */
void MyGLWidget::resizeGL (int width, int height){
    glViewport( 0, 0, (GLint)width, (GLint)height );

    /* create viewing cone with near and far clipping planes */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum( -1.0, 1.0, -1.0, 1.0, 5.0, 30.0);

    glMatrixMode( GL_MODELVIEW );
}



/*
 * Renders the OpenGL scene. Gets called whenever the widget needs to be updated.
 */
void MyGLWidget::paintGL(){

    //delete color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);


    glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();
        //glTranslatef(0.0f,0.0f,-20.0f); //move along z-axis
        //glRotatef(30.0,0.0,1.0,0.0); //rotate 30 degress around y-axis
        //glRotatef(15.0,1.0,0.0,0.0); //rotate 15 degress around x-axis


        QMatrix4x4 matrix;
        matrix.translate(0.0, 0.0, -20.0);
        rotation = QQuaternion();



        rotation *= QQuaternion(q0, q1, q2, q3);

        matrix.rotate(rotation);

        //qDebug() << "paint";
        glLoadMatrixf(matrix.constData());
    /* create 3D-Cube */
    glBegin(GL_QUADS);

        //front
        glColor3f(0.0,0.0,0.0);

        glVertex3f(1.0,0.5,1.0);
        glVertex3f(-1.0,0.5,1.0);
        glVertex3f(-1.0,-0.5,1.0);
        glVertex3f(1.0,-0.5,1.0);


        //back

        glColor3f(0.0,0.0,0.0);

        glVertex3f(1.0,0.5,-1.0);
        glVertex3f(-1.0,0.5,-1.0);
        glVertex3f(-1.0,-0.5,-1.0);
        glVertex3f(1.0,-0.5,-1.0);


        //top
        glColor3f(0.0,0.0,1.0);

        glVertex3f(-1.0,0.5,1.0);
        glVertex3f(1.0,0.5,1.0);
        glVertex3f(1.0,0.5,-1.0);
        glVertex3f(-1.0,0.5,-1.0);


        //bottom
        glColor3f(0.0,1.0,1.0);

        glVertex3f(1.0,-0.5,1.0);
        glVertex3f(1.0,-0.5,-1.0);
        glVertex3f(-1.0,-0.5,-1.0);
        glVertex3f(-1.0,-0.5,1.0);

        //right
        glColor3f(0.0,0.0,0.0);

        glVertex3f(1.0,0.5,1.0);
        glVertex3f(1.0,-0.5,1.0);
        glVertex3f(1.0,-0.5,-1.0);
        glVertex3f(1.0,0.5,-1.0);


        //left
        glColor3f(0.0,0.0,0.0);

        glVertex3f(-1.0,0.5,1.0);
        glVertex3f(-1.0,-0.5,1.0);
        glVertex3f(-1.0,-0.5,-1.0);
        glVertex3f(-1.0,0.5,-1.0);


    glEnd();

}
