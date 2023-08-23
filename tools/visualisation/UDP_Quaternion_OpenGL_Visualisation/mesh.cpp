/****************************************************************************
**
** BSD License
**
** Copyright (C) 2023 Fritz Webering <fritz.webering@ims.uni-hannover.de>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
** 
** 1. Redistributions of source code must retain the above copyright notice, this
**    list of conditions and the following disclaimer.
** 
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
** 
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
** 
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
** ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
** ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
****************************************************************************/
#include "mesh.h"

void Mesh::addVertex(const QVector3D &v, const QVector3D &color)
{
    m_data.append(v.x());
    m_data.append(v.y()); // OpenGL Y axis is inverted, leading to a left-handed coordinate system
    m_data.append(v.z());
    m_data.append(color.x());
    m_data.append(color.y());
    m_data.append(color.z());
}

void Mesh::addFaceXY(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2, GLfloat x3, GLfloat y3, GLfloat x4, GLfloat y4, GLfloat z, const QVector3D &color)
{
    addVertex(QVector3D(x4, y4, z), color);
    addVertex(QVector3D(x1, y1, z), color);
    addVertex(QVector3D(x2, y2, z), color);

    addVertex(QVector3D(x2, y2, z), color);
    addVertex(QVector3D(x3, y3, z), color);
    addVertex(QVector3D(x4, y4, z), color);
}

void Mesh::extrudeLineXY(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2, GLfloat z_start, GLfloat z_stop, const QVector3D &color)
{
    addVertex(QVector3D(x1, y1, z_start), color);
    addVertex(QVector3D(x1, y1, z_stop), color);
    addVertex(QVector3D(x2, y2, z_start), color);

    addVertex(QVector3D(x2, y2, z_stop), color);
    addVertex(QVector3D(x2, y2, z_start), color);
    addVertex(QVector3D(x1, y1, z_stop), color);
}

void Mesh::setDrawMode(GLenum mode)
{
    assert(mode == GL_TRIANGLES || mode == GL_LINES);
    m_drawMode = mode;
}
