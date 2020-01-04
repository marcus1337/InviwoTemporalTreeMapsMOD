/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2013-2019 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/opengl/rendering/meshdrawergl.h>
#include <modules/opengl/buffer/buffergl.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/shader/shader.h>

#define X(e) std::get<0>(e)
#define Y(e) std::get<1>(e) 

namespace inviwo {

MeshDrawerGL::MeshDrawerGL() : meshToDraw_(nullptr) {}

MeshDrawerGL::MeshDrawerGL(const Mesh* mesh) : meshToDraw_(mesh) {
    if (mesh == nullptr) throw NullPointerException("input mesh is null", IVW_CONTEXT);
}

MeshDrawerGL::DrawObject MeshDrawerGL::getDrawObject() const {
    return DrawObject(meshToDraw_->getRepresentation<MeshGL>(), meshToDraw_->getDefaultMeshInfo());
}

MeshDrawerGL::DrawObject MeshDrawerGL::getDrawObject(const Mesh* mesh) {
    return DrawObject(mesh->getRepresentation<MeshGL>(), mesh->getDefaultMeshInfo());
}

void MeshDrawerGL::drawTextInfo(std::string str) {
        std::fstream fs;
        std::string fullFilePath = "D:/School/TemporalTreeMaps/modules/temporaltreemaps/glsl/mytextinfo.txt";
        fs.open(fullFilePath, std::fstream::in | std::fstream::out | std::fstream::app);
        fs << str;
        fs.close();
}

std::vector<std::vector<std::tuple<float, float>>> MeshDrawerGL::loadAllTriangles(
    std::string fileNameTxt) {
    std::vector<std::vector<std::tuple<float, float>>> result;
    std::string fullFilePath = "D:/School/TemporalTreeMaps/modules/temporaltreemaps/glsl/" + fileNameTxt;
    std::ifstream file(fullFilePath);

    float x;
    while (file >> x) {
        std::vector<std::tuple<float, float>> res;
        float y;
        file >> y;
        std::tuple<float, float> P = std::make_tuple(x, y);
        res.push_back(P);
        file >> x >> y;
        P = std::make_tuple(x, y);
        res.push_back(P);

        file >> x >> y;
        P = std::make_tuple(x, y);
        res.push_back(P);
        result.push_back(res);
    }

    return result;
}

void MeshDrawerGL::loadBeziers(std::vector<std::vector<std::tuple<float, float>>>& bezierss, std::string fileNameTxt) {
    bezierss.clear();
    std::string fullFilePath = "D:/School/TemporalTreeMaps/modules/temporaltreemaps/glsl/"+ fileNameTxt;
    std::ifstream file(fullFilePath);
    float x;
    while (file >> x) {
        std::vector<std::tuple<float, float>> res;
        float y;
        file >> y;
        std::tuple<float, float> P = std::make_tuple(x,y);
        res.push_back(P);
        for (int i = 0; i < 7; i++) {
            file >> x >> y;
            P = std::make_tuple(x, y);
            res.push_back(P);
        }
        bezierss.push_back(res);
    }
}

void MeshDrawerGL::draw() {
    if (meshToDraw_->getNumberOfBuffers() == 0) {
        // empty mesh, do nothing
        return;
    }

    typedef std::vector<std::tuple<float, float>> tupvec;

    std::vector<std::vector<std::tuple<float, float>>> beziers;
    std::vector<std::vector<std::tuple<float, float>>> beziersSplits;
    auto triangles = loadAllTriangles();

    if (drawInfo == 999) {
        loadBeziers(beziers);
        loadBeziers(beziersSplits, "beziersplitinfo.txt");
    }

    auto meshGL = meshToDraw_->getRepresentation<MeshGL>();

    int yMax = 0;
    
    utilgl::Enable<MeshGL> enable(meshGL);

    int normalBezierCounter = 0;
    int splitBezierCounter = 0;
    int triangleCounter = 0;

    std::size_t numIndexBuffers = meshGL->getIndexBufferCount();
    if (numIndexBuffers > 0) {
        // draw mesh using the index buffers
        for (std::size_t i = 0; i < numIndexBuffers; ++i) {
            auto indexBuffer = meshGL->getIndexBuffer(i);            

            auto numIndices = indexBuffer->getSize();
            if (numIndices > 0) {

                auto drawMode = getGLDrawMode(meshGL->getMeshInfoForIndexBuffer(i));
                indexBuffer->bind();

                auto numInds = static_cast<GLsizei>(numIndices);
                 
               //  glDrawElements(drawMode, static_cast<GLsizei>(numIndices),
               //                indexBuffer->getFormatType(), nullptr);

                glPatchParameteri(GL_PATCH_VERTICES, 3);
                if (drawInfo == 999 && numIndices == 6 &&
                    (drawMode == GL_TRIANGLE_STRIP || drawMode == GL_TRIANGLES)) {
                    glm::bool1 b1 = true;

                    (*((Shader*)bandShader)).setUniform("isLimited", b1);
                    tupvec bezier;

                    if (drawMode == GL_TRIANGLES) {
                        bezier = beziersSplits[splitBezierCounter++];
                    } else {
                        bezier = beziers[normalBezierCounter++];
                    }

                    

                    glm::vec2 p0T(X(bezier[0]), Y(bezier[0]));
                    glm::vec2 p1T(X(bezier[2]), Y(bezier[2]));
                    glm::vec2 p2T(X(bezier[4]), Y(bezier[4]));
                    glm::vec2 p3T(X(bezier[6]), Y(bezier[6]));

                    glm::vec2 p0B(X(bezier[1]), Y(bezier[1]));
                    glm::vec2 p1B(X(bezier[3]), Y(bezier[3]));
                    glm::vec2 p2B(X(bezier[5]), Y(bezier[5]));
                    glm::vec2 p3B(X(bezier[7]), Y(bezier[7]));

                    (*((Shader*)bandShader)).setUniform("p0T", p0T);
                    (*((Shader*)bandShader)).setUniform("p1T", p1T);
                    (*((Shader*)bandShader)).setUniform("p2T", p2T);
                    (*((Shader*)bandShader)).setUniform("p3T", p3T);

                    (*((Shader*)bandShader)).setUniform("p0B", p0B);
                    (*((Shader*)bandShader)).setUniform("p1B", p1B);
                    (*((Shader*)bandShader)).setUniform("p2B", p2B);
                    (*((Shader*)bandShader)).setUniform("p3B", p3B);

                } else            
                {
                    glm::bool1 b2 = false;
                    (*((Shader*)bandShader)).setUniform("isLimited", b2);
                }

                int tricnt = std::min<int>(triangleCounter, triangles.size() - 2);
                tupvec triangle = triangles[tricnt];
                tupvec tri1 = triangles[tricnt];
                tupvec tri2 = triangles[tricnt + 1];

                typedef std::vector<glm::vec2> edge;
                edge edge1({glm::vec2(X(tri1[0]), Y(tri1[0])), glm::vec2(X(tri1[1]), Y(tri1[1]))});
                edge edge2({glm::vec2(X(tri1[0]), Y(tri1[0])), glm::vec2(X(tri1[2]), Y(tri1[2]))});
                edge edge3({glm::vec2(X(tri1[1]), Y(tri1[1])), glm::vec2(X(tri1[2]), Y(tri1[2]))});

                edge edge11({glm::vec2(X(tri2[0]), Y(tri2[0])), glm::vec2(X(tri2[1]), Y(tri2[1]))});
                edge edge22({glm::vec2(X(tri2[0]), Y(tri2[0])), glm::vec2(X(tri2[2]), Y(tri2[2]))});
                edge edge33({glm::vec2(X(tri2[1]), Y(tri2[1])), glm::vec2(X(tri2[2]), Y(tri2[2]))});

                glm::vec2 d1(-999);
                glm::vec2 d2(-999);

                glm::vec2 d3(-999);
                glm::vec2 d4(-999);

                if (abs(edge1[0].x - edge1[1].x) > 0.0005) {
                    d1 = edge1[0];
                    d2 = edge1[1];
                }
                if ((abs(edge2[0].x - edge2[1].x) > 0.0005) &&
                    abs(edge2[0].y - edge2[1].y) > abs(d1.y - d2.y)) {
                    d1 = edge2[0];
                    d2 = edge2[1];
                }
                if ((abs(edge3[0].x - edge3[1].x) > 0.0005) &&
                    abs(edge3[0].y - edge3[1].y) > abs(d1.y - d2.y)) {
                    d1 = edge3[0];
                    d2 = edge3[1];
                }


                if ((abs(edge11[0].x - edge11[1].x) > 0.0005)) {
                    d3 = edge11[0];
                    d4 = edge11[1];
                }
                if (abs(edge22[0].x - edge22[1].x) > 0.0005 && abs(edge22[0].y - edge22[1].y) > abs(d3.y-d4.y))  {
                    d3 = edge22[0];
                    d4 = edge22[1];
                }
                if (abs(edge33[0].x - edge33[1].x) > 0.0005 &&
                    abs(edge33[0].y - edge33[1].y) > abs(d3.y - d4.y)) {
                    d3 = edge33[0];
                    d4 = edge33[1];
                }
               

                if ((abs(d1.x - d2.x) < 0.0005)) {
                    d1 = d3;
                    d2 = d4;
                } else if ((abs(d1.x - d2.x) > 0.0005) && (abs(d3.x - d4.x) > 0.0005)) {
                    if ((abs(d1.y - d2.y) < abs(d3.y - d4.y))) {
                        d1 = d3;
                        d2 = d4;
                    }
                }


                if (d1.x > d2.x) {
                    auto tmp = d1; 
                    d1 = d2;
                    d2 = tmp;
                }

                (*((Shader*)bandShader)).setUniform("d1", d1);
                (*((Shader*)bandShader)).setUniform("d2", d2);

                triangleCounter += 2;
                glm::vec2 Or0(X(triangle[0]), Y(triangle[0]));
                glm::vec2 Or1(X(triangle[1]), Y(triangle[1]));
                glm::vec2 Or2(X(triangle[2]), Y(triangle[2]));
                (*((Shader*)bandShader)).setUniform("Or0", Or0);
                (*((Shader*)bandShader)).setUniform("Or1", Or1);
                (*((Shader*)bandShader)).setUniform("Or2", Or2);

                if (drawInfo == 999) {
                    utilgl::setShaderUniforms(*((Shader*)bandShader), *(getMesh()), "geometry_");
                }

               // if (normalBezierCounter != 6) continue;

                glm::bool1 seconddraw = false;
                (*((Shader*)bandShader)).setUniform("seconddraw", seconddraw);

                glDrawElements(GL_PATCHES, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);

                seconddraw = true;
                (*((Shader*)bandShader)).setUniform("seconddraw", seconddraw);
                glDrawElements(GL_PATCHES, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);
            }
        }
    } else {
        // the mesh does not contain index buffers, render all vertices
        auto drawMode = getGLDrawMode(meshToDraw_->getDefaultMeshInfo());
        glDrawArrays(drawMode, 0, static_cast<GLsizei>(meshGL->getBufferGL(0)->getSize()));
    }
}

void MeshDrawerGL::draw(DrawMode drawMode) {
    auto meshGL = meshToDraw_->getRepresentation<MeshGL>();
    utilgl::Enable<MeshGL> enable(meshGL);

    auto drawModeGL = getGLDrawMode(drawMode);

    std::size_t numIndexBuffers = meshGL->getIndexBufferCount();
    if (numIndexBuffers > 0) {
        // draw mesh using the index buffers
        for (std::size_t i = 0; i < numIndexBuffers; ++i) {
            auto indexBuffer = meshGL->getIndexBuffer(i);
            auto numIndices = indexBuffer->getSize();
            if (numIndices > 0) {
                indexBuffer->bind();
                glDrawElements(drawModeGL, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);
            }
        }
    } else {
        // the mesh does not contain index buffers, render all vertices
        glDrawArrays(drawModeGL, 0, static_cast<GLsizei>(meshGL->getBufferGL(0)->getSize()));
    }
}

MeshDrawerGL::DrawMode MeshDrawerGL::getDrawMode(DrawType dt, ConnectivityType ct) {
    switch (dt) {
        case DrawType::Triangles:
            switch (ct) {
                case ConnectivityType::None:
                    return DrawMode::Triangles;

                case ConnectivityType::Strip:
                    return DrawMode::TriangleStrip;

                case ConnectivityType::Fan:
                    return DrawMode::TriangleFan;

                case ConnectivityType::Adjacency:
                    return DrawMode::TrianglesAdjacency;

                case ConnectivityType::StripAdjacency:
                    return DrawMode::TriangleStripAdjacency;

                case ConnectivityType::Loop:
                case ConnectivityType::NumberOfConnectivityTypes:
                default:
                    return DrawMode::Points;
            }

        case DrawType::Lines:
            switch (ct) {
                case ConnectivityType::None:
                    return DrawMode::Lines;

                case ConnectivityType::Strip:
                    return DrawMode::LineStrip;

                case ConnectivityType::Loop:
                    return DrawMode::LineLoop;

                case ConnectivityType::Adjacency:
                    return DrawMode::LinesAdjacency;

                case ConnectivityType::StripAdjacency:
                    return DrawMode::LineStripAdjacency;

                case ConnectivityType::Fan:
                case ConnectivityType::NumberOfConnectivityTypes:
                default:
                    return DrawMode::Points;
            }

        case DrawType::Points:
        case DrawType::NotSpecified:
        case DrawType::NumberOfDrawTypes:
        default:
            return DrawMode::Points;
    }
}

GLenum MeshDrawerGL::getGLDrawMode(DrawMode dm) {
    switch (dm) {
        case DrawMode::Points:
            return GL_POINTS;
        case DrawMode::Lines:
            return GL_LINES;
        case DrawMode::LineStrip:
            return GL_LINE_STRIP;
        case DrawMode::LineLoop:
            return GL_LINE_LOOP;
        case DrawMode::LinesAdjacency:
            return GL_LINES_ADJACENCY;
        case DrawMode::LineStripAdjacency:
            return GL_LINE_STRIP_ADJACENCY;
        case DrawMode::Triangles:
            return GL_TRIANGLES;
        case DrawMode::TriangleStrip:
            return GL_TRIANGLE_STRIP;
        case DrawMode::TriangleFan:
            return GL_TRIANGLE_FAN;
        case DrawMode::TrianglesAdjacency:
            return GL_TRIANGLES_ADJACENCY;
        case DrawMode::TriangleStripAdjacency:
            return GL_TRIANGLE_STRIP_ADJACENCY;
        case DrawMode::NumberOfDrawModes:
        case DrawMode::NotSpecified:
        default:
            return GL_POINTS;
    }
}

GLenum MeshDrawerGL::getGLDrawMode(Mesh::MeshInfo meshInfo) {
    return getGLDrawMode(getDrawMode(meshInfo.dt, meshInfo.ct));
}

MeshDrawerGL::DrawObject::DrawObject(const MeshGL* mesh, Mesh::MeshInfo arrayMeshInfo)
    : enable_(mesh), meshGL_(mesh), arrayMeshInfo_(arrayMeshInfo) {}

void MeshDrawerGL::DrawObject::draw() {
    const std::size_t numIndexBuffers = meshGL_->getIndexBufferCount();
    if (numIndexBuffers > 0) {
        // draw mesh using the index buffers
        for (std::size_t i = 0; i < numIndexBuffers; ++i) {
            const auto indexBuffer = meshGL_->getIndexBuffer(i);
            const auto numIndices = indexBuffer->getSize();
            if (numIndices > 0) {
                indexBuffer->bind();
                const auto drawModeGL = getGLDrawMode(meshGL_->getMeshInfoForIndexBuffer(i));
                glDrawElements(drawModeGL, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);
            }
        }
    } else if (!meshGL_->empty()) {
        // the mesh does not contain index buffers, render all vertices
        const auto drawModeGL = getGLDrawMode(arrayMeshInfo_);
        glDrawArrays(drawModeGL, 0, static_cast<GLsizei>(meshGL_->getBufferGL(0)->getSize()));
    }
}

void MeshDrawerGL::DrawObject::draw(DrawMode drawMode) {
    const auto drawModeGL = getGLDrawMode(drawMode);

    const std::size_t numIndexBuffers = meshGL_->getIndexBufferCount();
    if (numIndexBuffers > 0) {
        // draw mesh using the index buffers
        for (std::size_t i = 0; i < numIndexBuffers; ++i) {
            const auto indexBuffer = meshGL_->getIndexBuffer(i);
            const auto numIndices = indexBuffer->getSize();
            if (numIndices > 0) {
                indexBuffer->bind();
                glDrawElements(drawModeGL, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);
            }
        }
    } else if (!meshGL_->empty()) {
        // the mesh does not contain index buffers, render all vertices
        glDrawArrays(drawModeGL, 0, static_cast<GLsizei>(meshGL_->getBufferGL(0)->getSize()));
    }
}

void MeshDrawerGL::DrawObject::draw(std::size_t index) {
    const std::size_t numIndexBuffers = meshGL_->getIndexBufferCount();
    if (index >= numIndexBuffers) {
        throw RangeException("Index (" + std::to_string(index) + ") for indexbuffer of size " +
                                 std::to_string(numIndexBuffers) + " is out-of-range.",
                             IVW_CONTEXT);
    }
    const auto indexBuffer = meshGL_->getIndexBuffer(index);
    const auto numIndices = indexBuffer->getSize();
    if (numIndices > 0) {
        indexBuffer->bind();
        const auto drawModeGL = getGLDrawMode(meshGL_->getMeshInfoForIndexBuffer(index));
        glDrawElements(drawModeGL, static_cast<GLsizei>(numIndices), indexBuffer->getFormatType(),
                       nullptr);
    }
}

void MeshDrawerGL::DrawObject::draw(DrawMode drawMode, std::size_t index) {
    const std::size_t numIndexBuffers = meshGL_->getIndexBufferCount();
    if (index >= numIndexBuffers) {
        throw RangeException("Index (" + std::to_string(index) + ") for indexbuffer of size " +
                                 std::to_string(numIndexBuffers) + " is out-of-range.",
                             IVW_CONTEXT);
    }
    const auto indexBuffer = meshGL_->getIndexBuffer(index);
    const auto numIndices = indexBuffer->getSize();
    if (numIndices > 0) {
        indexBuffer->bind();
        const auto drawModeGL = getGLDrawMode(drawMode);
        glDrawElements(drawModeGL, static_cast<GLsizei>(numIndices), indexBuffer->getFormatType(),
                       nullptr);
    }
}

std::size_t MeshDrawerGL::DrawObject::size() const { return meshGL_->getIndexBufferCount(); }

}  // namespace inviwo
