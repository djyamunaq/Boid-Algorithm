#include <eigen3/Eigen/Dense>
#include <GL/glut.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <iostream>
#include "Flock.h"

#define UPDATE_TIME 1

void reshape(int w, int h);
void display();
void update(int value);

Flock flock;

int main(int argc, char** argv) {
    int width = 1000;
    int height = 1000;
    int nBoids = 1e2;

    flock = Flock(nBoids);
    flock.initializeQtree(Eigen::Vector2d::Zero(), width, height, 5);

    /* Initialize OpenGL */
    glutInit(&argc, argv);
    /* Set double buffered window with RGBA mode window */
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);

    glutInitWindowSize(width, height);

    /* Create window */
    int win = glutCreateWindow("Points");

    /* Set color used when clearing the window */
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    /* Binds callback function to render */
    glutDisplayFunc(display);

    glutReshapeFunc(reshape);

    glutTimerFunc(UPDATE_TIME, update, 0);

    /* Transfer control to OpenGL */
    glutMainLoop();

    return 0;
}

void display() {
    /* Clear window */
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    glPointSize(5.0);

    /* Define type of primitive -> Point */
    glBegin(GL_TRIANGLES);

    /*============================================================================================*/
    std::vector<Boid*> Boids = flock.getBoids();
    std::vector<Boid*>::iterator it = Boids.begin();

    for(; it!=Boids.end(); it++) {
        Boid* boid = (*it);
        Eigen::Vector2d pos = boid->getPos();
        Eigen::Vector2d vel = boid->getVel();

        if(pos(0) < -500) {
            boid->setPos(Eigen::Vector2d(500, pos(1)));
            // boid->setVel(Eigen::Vector2d(-vel(0), vel(1)));
        } else if(pos(0) > 500) {
            boid->setPos(Eigen::Vector2d(-500, pos(1)));
            // boid->setVel(Eigen::Vector2d(-vel(0), vel(1)));
        }
        if(pos(1) < -500) {
            boid->setPos(Eigen::Vector2d(pos(0), 500));
            // boid->setVel(Eigen::Vector2d(-vel(0), vel(1)));
        } else if(pos(1) > 500) {
            boid->setPos(Eigen::Vector2d(pos(0), -500));
            // boid->setVel(Eigen::Vector2d(-vel(0), vel(1)));
        }

        Eigen::Vector2d velU = vel/vel.norm();
        Eigen::Vector2d orthoVelU = velU.unitOrthogonal();

        Eigen::Vector2d p0 = pos + 20*velU;
        Eigen::Vector2d p1 = pos + 10*orthoVelU;
        Eigen::Vector2d p2 = pos - 10*orthoVelU;

        glVertex2d(p0(0), p0(1));
        glVertex2d(p1(0), p1(1));
        glVertex2d(p2(0), p2(1));
    }
    /*============================================================================================*/

    /* Finish drawing primitive */
    glEnd();

    glFlush();

    /* Swap front and back buffers */
    glutSwapBuffers();
    glutPostRedisplay();
}

void update(int value) {
    flock.run();

    glutTimerFunc(UPDATE_TIME, update, 0);
}

void reshape(int w, int h) {
    // /* Set position of view port */
    glViewport(0, 0, w, h);
    
    /* Switch to projection matrix */
    glMatrixMode(GL_PROJECTION);
    /* Resets parameters of projection matrix */
    glLoadIdentity();
    /* Set orthogonal projection parameters */
    gluOrtho2D(-500, 500, -500, 500);
    /**/
    glMatrixMode(GL_MODELVIEW);
}