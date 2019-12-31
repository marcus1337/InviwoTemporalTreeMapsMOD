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

    /*std::string test;
    for (auto arr : bezierss) {
        for (auto v : arr) {
            float x = std::get<0>(v);
            float y = std::get<1>(v);
            test += "(" + std::to_string(x) + "," + std::to_string(y)+") ";
        }
        test += "\n";
    }
    drawTextInfo(test);*/

}

void MeshDrawerGL::draw() {
    if (meshToDraw_->getNumberOfBuffers() == 0) {
        // empty mesh, do nothing
        return;
    }

    typedef std::vector<std::tuple<float, float>> tupvec;

    std::vector<std::vector<std::tuple<float, float>>> beziers;
    std::vector<std::vector<std::tuple<float, float>>> beziersSplits;

    if (drawInfo == 999) {
        loadBeziers(beziers);
        loadBeziers(beziersSplits, "beziersplitinfo.txt");
      //  utilgl::setShaderUniforms(*((Shader*)bandShader), *(getMesh()), "geometry_");
    }

    auto meshGL = meshToDraw_->getRepresentation<MeshGL>();

    int yMax = 0;
    
    utilgl::Enable<MeshGL> enable(meshGL);

    int normalBezierCounter = 0;
    int splitBezierCounter = 0;

    std::size_t numIndexBuffers = meshGL->getIndexBufferCount();
    if (numIndexBuffers > 0) {
        // draw mesh using the index buffers
        for (std::size_t i = 0; i < numIndexBuffers; ++i) {
            auto indexBuffer = meshGL->getIndexBuffer(i);

           // auto test = *indexBuffer->getBufferObject();
            

            auto numIndices = indexBuffer->getSize();
            if (numIndices > 0) {

                auto drawMode = getGLDrawMode(meshGL->getMeshInfoForIndexBuffer(i));
                indexBuffer->bind();

                auto numInds = static_cast<GLsizei>(numIndices);
                 
           
               //  glDrawElements(drawMode, static_cast<GLsizei>(numIndices),
               //                indexBuffer->getFormatType(), nullptr);

                //glPatchParameteri(GL_PATCH_VERTICES, 3);
                if (drawInfo == 999 && numIndices == 6 &&
                    (drawMode == GL_TRIANGLE_STRIP || drawMode == GL_TRIANGLES)) {
                    glm::bool1 b1 = true;
                   // if (normalBezierCounter != 1) b1 = false;

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

                if (drawInfo == 999) {

                    utilgl::setShaderUniforms(*((Shader*)bandShader), *(getMesh()), "geometry_");
                }

                glDrawElements(GL_PATCHES, static_cast<GLsizei>(numIndices),
                               indexBuffer->getFormatType(), nullptr);

                //numInds varies, for now don't worry about GL_TRIANGLES
                //
                //std::fstream fs; //Some are, mainly splits, GL_TRIANGLES, rest are GL_TRIANGLE_STRIP
               /* fs.open("test.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                fs << " inds: " << numInds << " type: " << indexBuffer->getFormatType() << " numIndBuffs: " << numIndexBuffers << " drawmode: " << drawMode
                   << " EXTRA: " << "\n";
                fs.close();*/
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
