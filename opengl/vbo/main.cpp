/*
 *  Copyright (c) 2014 Evgeny Proydakov <lord.tiran@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

#define GL_GLEXT_PROTOTYPES

#include <common/iglut.h>

GLuint vertexes;
GLuint colors;

void createBuffers();
void display();
void deleteBuffers();

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutCreateWindow("VBO");
    glutDisplayFunc(display);
 
    glMatrixMode(GL_MODELVIEW);

    createBuffers();
    glutMainLoop();
    deleteBuffers();

    return 0;
}

void createBuffers() {
    static const GLfloat planeVertexes[4][3] = {
        {0.5, 0.5, 0.0},
        {-0.5, 0.5, 0.0},
        {-0.5, -0.5, 0.0}, 
        {0.5, -0.5, 0.0}
    };

    static const GLfloat planeColors[4][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 1.0, 1.0}
    };

    glGenBuffers(1, &vertexes);
    glGenBuffers(1, &colors);

    /* Vertex data */
    glBindBuffer(GL_ARRAY_BUFFER, vertexes);
    glBufferData(GL_ARRAY_BUFFER, 4*3*sizeof(GLfloat), planeVertexes, GL_STATIC_DRAW);

    /* Color data */
    glBindBuffer(GL_ARRAY_BUFFER, colors);
    glBufferData(GL_ARRAY_BUFFER, 4*3*sizeof(GLfloat), planeColors, GL_STATIC_DRAW);
}

void display() {
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    /* Vertex data */
    glBindBuffer(GL_ARRAY_BUFFER, vertexes);
    glVertexPointer(3, GL_FLOAT, 0, NULL);
    glEnableClientState(GL_VERTEX_ARRAY);

    /* Color data */
    glBindBuffer(GL_ARRAY_BUFFER, colors);
    glColorPointer(3, GL_FLOAT, 0, NULL);
    glEnableClientState(GL_COLOR_ARRAY);
   
    glDrawArrays(GL_POLYGON, 0, 4);
    glFlush();
}

void deleteBuffers() {
    glDeleteBuffers(1, &vertexes);
    glDeleteBuffers(1, &colors);
}
