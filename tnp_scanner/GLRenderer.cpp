/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

#include "GLRenderer.h"
#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

std::unique_ptr<GLRendererMngr> GLRendererMngr::m_instance;
std::once_flag GLRendererMngr::m_onceFlagInit;
std::once_flag GLRendererMngr::m_onceFlagGlewInit;


GLWindow::GLWindow()
{
    oldLeftDownInit = false;
    oldRightDownInit = false;
    oldXInit = false;
    oldYInit = false;
    oldScrollXInit = false;
    oldScrollYInit = false;

    oldLeftDown = false;
    oldRightDown = false;
    oldX = 0;
    oldY = 0;
    oldScrollX = 0;
    oldScrollY = 0;
}

cv::Point2d GLWindow::deltaMouse(double mouseX, double mouseY)
{
    cv::Point2d delta(0,0);
    if(oldXInit)
        delta.x = mouseX - oldX;
    if(oldYInit)
        delta.y = mouseY - oldY;
    return delta;
}

cv::Point2d GLWindow::deltaScrollMouse(double scrollX, double scrollY)
{
    cv::Point2d delta(0,0);
    if(oldScrollXInit)
        delta.x = scrollX - oldScrollX;
    if(oldScrollYInit)
        delta.y = scrollY - oldScrollY;
    return delta;
}

GLWindow_GLFW::GLWindow_GLFW(int w, int h, const char *title)
{
    printf("create GLWindow_GLFW\n");
    GLRendererMngr& mngr = GLRendererMngr::GetInstance();
    std::promise<bool> promiseObj;
    mngr.doInGLThread([w,h,title](void *arg){
        printf("create GLWindow_GLFW : in thread\n");
        GLWindow_GLFW *this_ptr = (GLWindow_GLFW*)arg;
        GLenum err;
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("CreateWindow 0 OpenGL error: %d\n", err);
        }
        printf("glfwCreateWindow\n");
        this_ptr->window = glfwCreateWindow( w, h, title, NULL, NULL);
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("CreateWindow 1 OpenGL error: %d\n", err);
        }
        printf("makeCurrent\n");
        this_ptr->makeCurrent();
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("CreateWindow 2 OpenGL error: %d\n", err);
        }

        GLRendererMngr::GetInstance().initGlew();
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("CreateWindow 3 OpenGL error: %d\n", err);
        }
        glViewport(0, 0, w, h);
        this_ptr->viewportSize = cv::Size(w,h);
        glfwSetMouseButtonCallback(this_ptr->window, [](GLFWwindow *win, int button, int action, int mods)
            {
                GLWindow *window = GLRendererMngr::GetInstance().getRendererByVoidPtr(win)->win;
                bool newLeftDown = window->oldLeftDown;
                bool newRightDown = window->oldRightDown;
                if(button == GLFW_MOUSE_BUTTON_LEFT)
                    newLeftDown = (action==GLFW_PRESS);
                if(button == GLFW_MOUSE_BUTTON_RIGHT)
                    newRightDown = (action==GLFW_PRESS);
                window->onMouseEvent(newLeftDown, newRightDown, window->oldX, window->oldY, 0, 0);
                window->oldLeftDown = newLeftDown;
                window->oldRightDown = newRightDown;
                window->oldLeftDownInit = true;
                window->oldRightDownInit = true;
            });
        glfwSetCursorPosCallback(this_ptr->window, [](GLFWwindow* win, double xpos, double ypos)
            {
                GLWindow *window = GLRendererMngr::GetInstance().getRendererByVoidPtr(win)->win;
                window->onMouseEvent(window->oldLeftDown, window->oldRightDown, xpos, ypos, 0, 0);
                window->oldX = xpos;
                window->oldY = ypos;
                window->oldXInit = true;
                window->oldYInit = true;
            });
        glfwSetScrollCallback(this_ptr->window, [](GLFWwindow* win, double scrollX, double scrollY)
            {
                GLWindow *window = GLRendererMngr::GetInstance().getRendererByVoidPtr(win)->win;
                window->onMouseEvent(window->oldLeftDown, window->oldRightDown, window->oldX, window->oldY, scrollX, scrollY);
                window->oldScrollX = scrollX;
                window->oldScrollY = scrollY;
                window->oldScrollXInit = true;
                window->oldScrollYInit = true;
            });
        glfwSetKeyCallback(this_ptr->window, [](GLFWwindow* win, int key, int scancode, int action, int mods)
        {
            GLWindow *window = GLRendererMngr::GetInstance().getRendererByVoidPtr(win)->win;
            window->onKeyboardEvent(key, action);
        });
        glfwSetFramebufferSizeCallback(this_ptr->window, [](GLFWwindow* win, int width, int height)
            {
                printf("size %d %d\n", width, height);
                glViewport(0, 0, width, height);
                GLWindow *window = GLRendererMngr::GetInstance().getRendererByVoidPtr(win)->win;
                window->viewportSize = cv::Size(width,height);
            });
        if( this_ptr->window == NULL ){
            printf( "Failed to open GLFW window.\n" );
            glfwTerminate();
        }
        glfwSetInputMode(this_ptr->window, GLFW_STICKY_KEYS, GL_TRUE);
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("CreateWindow 4 OpenGL error: %d\n", err);
        }
    }, this, &promiseObj);
    promiseObj.get_future().get();
}

void GLWindow_GLFW::makeCurrent()
{
    glfwMakeContextCurrent(window);
}


void GLWindow_GLFW::swapBuffer()
{
    glfwSwapBuffers(window);
}

void GLWindow_GLFW::pollEvents()
{
    glfwPollEvents();
}

void GLWindow_GLFW::destroyWindow()
{
    glfwDestroyWindow(window);
}

void *GLWindow_GLFW::getVoidPtr()
{
    return window;
}

cv::Size GLWindow_GLFW::getSize()
{
    cv::Size size;
    glfwGetWindowSize(window, &size.width,&size.height);
    return size;
}

bool GLWindow_GLFW::shouldClose()
{
    return glfwWindowShouldClose(window) != 0;
}

GLShaderData::GLShaderData(GLenum shaderType, std::string shader)
    :shaderType(shaderType)
{
    shaderId = -1;
    current_version = -1;
    built_version = -1;
    setShader(shader);
}

GLShaderData::~GLShaderData()
{
    glDeleteShader(shaderId);
}

void GLShaderData::setShader(std::string shader)
{
    mutex.lock();
    this->shader = shader;
    current_version++;
    mutex.unlock();
}

void GLShaderData::compile(bool lock)
{
    printf("compile ShaderData\n");
    if(lock)
        mutex.lock();

    if(built_version < 0)
        shaderId = glCreateShader(shaderType);

    if(shader.size() > 0)
    {        
        char const * sourcePointer = shader.c_str();
        glShaderSource(shaderId, 1, &sourcePointer , NULL);
        glCompileShader(shaderId);

        GLint Result = GL_FALSE;
        int InfoLogLength = 0;
        glGetShaderiv(shaderId, GL_COMPILE_STATUS, &Result);
        if(Result == GL_FALSE)
        {
            glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &InfoLogLength);
            std::vector<char> shaderErrorMessage(InfoLogLength+1);
            glGetShaderInfoLog(shaderId, InfoLogLength, NULL, &shaderErrorMessage[0]);
            printf("%s\n", &shaderErrorMessage[0]);
            printf("compile error\n");
        }
        else built_version = current_version;
    }
    if(lock)
        mutex.unlock();
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("OpenGL error: %d\n", err);
    }
}

bool GLShaderData::isBuilt() const
{
    return shaderId >= 0 && built_version >= 0;
}

bool GLShaderData::isCurrentVersionBuilt() const
{
    return isBuilt() && built_version == current_version;
}

GLShaderProgram::GLShaderProgram()
{
    current_version = -1;
    built_version = -1;
}

void GLShaderProgram::setShader(std::shared_ptr<GLShaderData> vertexShader, std::shared_ptr<GLShaderData> fragmentShader)
{
    mutex.lock();
    this->vertexShader = vertexShader;
    this->fragmentShader = fragmentShader;
    current_version++;
    mutex.unlock();
}

void GLShaderProgram::compile(bool lock)
{
    printf("compile ShaderProgram\n");
    if(lock)
        mutex.lock();

    if(built_version < 0)
        programId = glCreateProgram();

    if(lock)
    {
        vertexShader->mutex.lock();
        fragmentShader->mutex.lock();
    }

    if(!vertexShader->isCurrentVersionBuilt())
        vertexShader->compile(false);
    if(!fragmentShader->isCurrentVersionBuilt())
        fragmentShader->compile(false);

    if(vertexShader->isBuilt() && fragmentShader->isBuilt())
    {
        glAttachShader(programId, vertexShader->shaderId);
        glAttachShader(programId, fragmentShader->shaderId);
        glLinkProgram(programId);

        GLint Result = GL_FALSE;
        int InfoLogLength;
        glGetProgramiv(programId, GL_LINK_STATUS, &Result);
        glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &InfoLogLength);
        std::vector<char> ProgramErrorMessage( std::max(InfoLogLength, int(1)) );
        glGetProgramInfoLog(programId, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);

        vertexAttribId = glGetAttribLocation(programId, "vertexPosition");
        normalAttribId = glGetAttribLocation(programId, "vertexNormal");
        colorAttribId = glGetAttribLocation(programId, "vertexColor");
        uniformAmbientColorId = glGetUniformLocation(programId, "ambientColor");
        uniformDiffuseColorId = glGetUniformLocation(programId, "diffuseColor");
        uniformSpecularColorId = glGetUniformLocation(programId, "specularColor");
        uniformShininessId = glGetUniformLocation(programId, "shininess");
        uniformCameraPosId = glGetUniformLocation(programId, "cameraPos");
        uniformLightPosId = glGetUniformLocation(programId, "lightPos");
        uniformCameraId = glGetUniformLocation(programId, "camera");
        uniformNormalMatId = glGetUniformLocation(programId, "normalMat");
        uniformModelMatId = glGetUniformLocation(programId, "modelMat");
        uniformViewMatId = glGetUniformLocation(programId, "viewMat");
        uniformProjMatId = glGetUniformLocation(programId, "projMat");
        //printf("vertexPosition %d color %d camera %d\n", vertexAttribId, uniformColorId,uniformCameraId);
        if(Result == GL_TRUE)
            built_version = current_version;
    }
    else printf("not build : %d %d\n", vertexShader->isBuilt(), fragmentShader->isBuilt());

    if(lock)
    {
        fragmentShader->mutex.unlock();
        vertexShader->mutex.unlock();
        mutex.unlock();
    }
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("OpenGL error: %d\n", err);
    }
}

bool GLShaderProgram::isBuilt() const
{
    return programId >= 0 && built_version >= 0;
}
bool GLShaderProgram::isCurrentVersionBuilt() const
{
    return isBuilt() && built_version == current_version;
}

void GLShaderProgram::use()
{
    glUseProgram(programId);
}

GLMeshData::GLMeshData(GLMeshDataType type)
    :type(type)
{
    autoBuildNormal = true;
    current_version_vertex = -1;
    current_version_index = -1;
    current_version_normal = -1;
    current_version_color = -1;
    built_version_vertex = -1;
    built_version_index = -1;
    built_version_normal = -1;
    built_version_color = -1;
}

void GLMeshData::setAutoBuildNormal(bool autoBuildNormal)
{
    this->autoBuildNormal = autoBuildNormal;
}

void GLMeshData::updateNormal(bool useLock)
{
    if(useLock)
        mutex.lock();
    if(type == GLMeshDataType::Triangles || type == GLMeshDataType::Quads)
    {
        int nbVertexPerElem = 3;
        if(type == GLMeshDataType::Quads)
            nbVertexPerElem = 4;
        normal = std::vector<GLfloat>(this->vertex.size());
        for(int i = 0; i < normal.size(); i++)
            normal[i] = 0;

        if(index.size() > 0)
        {
            for(int i = 0; i+nbVertexPerElem-1 < index.size(); i+=nbVertexPerElem)
            {
                cv::Point3f a(vertex[index[i]*3], vertex[index[i]*3+1], vertex[index[i]*3+2]);
                cv::Point3f b(vertex[index[i+1]*3], vertex[index[i+1]*3+1], vertex[index[i+1]*3+2]);
                cv::Point3f c(vertex[index[i+2]*3], vertex[index[i+2]*3+1], vertex[index[i+2]*3+2]);
                cv::Point3f ab = b-a;
                cv::Point3f ac = c-a;
                cv::Point3f n = ab.cross(ac);
                n /= sqrt(n.dot(n));
                normal[index[i]*3] += n.x; normal[index[i]*3+1] += n.y; normal[index[i]*3+2] += n.z;
                normal[index[i+1]*3] += n.x; normal[index[i+1]*3+1] += n.y; normal[index[i+1]*3+2] += n.z;
                normal[index[i+2]*3] += n.x; normal[index[i+2]*3+1] += n.y; normal[index[i+2]*3+2] += n.z;
                if(type == GLMeshDataType::Quads)
                {
                    cv::Point3f d(vertex[index[i+3]*3], vertex[index[i+3]*3+1], vertex[index[i+3]*3+2]);
                    cv::Point3f ad = d-a;
                    cv::Point3f n2 = ac.cross(ad);
                    n2 /= sqrt(n2.dot(n2));
                    normal[index[i]*3] += n2.x; normal[index[i]*3+1] += n2.y; normal[index[i]*3+2] += n2.z;
                    normal[index[i+2]*3] += n2.x; normal[index[i+2]*3+1] += n2.y; normal[index[i+2]*3+2] += n2.z;
                    normal[index[i+3]*3] += n2.x; normal[index[i+3]*3+1] += n2.y; normal[index[i+3]*3+2] += n2.z;
                }
            }
        }
        else
        {
            for(int i = 0; i+3*nbVertexPerElem-1 < vertex.size(); i+=3*nbVertexPerElem)
            {
                cv::Point3f a(vertex[i*3], vertex[i*3+1], vertex[i*3+2]);
                cv::Point3f b(vertex[i*3+3], vertex[i*3+4], vertex[i*3+5]);
                cv::Point3f c(vertex[i*3+6], vertex[i*3+7], vertex[i*3+8]);
                cv::Point3f ab = b-a;
                cv::Point3f ac = c-a;
                cv::Point3f n = ab.cross(ac);
                n /= sqrt(n.dot(n));
                normal[i*3] += n.x; normal[i*3+1] += n.y; normal[i*3+2] += n.z;
                normal[i*3+3] += n.x; normal[i*3+4] += n.y; normal[i*3+5] += n.z;
                normal[i*3+6] += n.x; normal[i*3+7] += n.y; normal[i*3+8] += n.z;
                if(type == GLMeshDataType::Quads)
                {
                    cv::Point3f d(vertex[i*3+9], vertex[i*3+10], vertex[i*3+11]);
                    cv::Point3f ad = d-a;
                    cv::Point3f n2 = ac.cross(ad);
                    n2 /= sqrt(n2.dot(n2));
                    normal[i*3] += n2.x; normal[i*3+1] += n2.y; normal[i*3+2] += n2.z;
                    normal[i*3+6] += n2.x; normal[i*3+7] += n2.y; normal[i*3+8] += n2.z;
                    normal[i*3+9] += n2.x; normal[i*3+10] += n2.y; normal[i*3+11] += n2.z;
                }
            }
        }
        for(int i = 0; i < normal.size(); i+=3)
        {
            GLfloat s = sqrt(normal[i]*normal[i]+normal[i+1]*normal[i+1]+normal[i+2]*normal[i+2]);
            normal[i] /= s;
            normal[i+1] /= s;
            normal[i+2] /= s;
        }
        current_version_normal++;
    }
    if(useLock)
        mutex.unlock();
}

void GLMeshData::clear()
{
    mutex.lock();
    if(this->vertex.size()>0)
    {
        this->vertex.clear();
        current_version_vertex++;
    }
    if(this->index.size()>0)
    {
        this->index.clear();
        current_version_index++;
    }
    if(this->normal.size()>0)
    {
        this->normal.clear();
        current_version_normal++;
    }
    if(this->color.size()>0)
    {
        this->color.clear();
        current_version_color++;
    }
    mutex.unlock();
}

void GLMeshData::setData(const std::vector<GLfloat>& vertex, const std::vector<unsigned int>& index, const std::vector<GLfloat> &color)
{
    mutex.lock();
    if(vertex.size() > 0)
    {
        this->vertex = vertex;
        current_version_vertex++;
    }
    if(index.size() > 0)
    {
        this->index = index;
        current_version_index++;
    }
    if(color.size() > 0)
    {
        this->color = color;
        current_version_index++;
    }
    if(autoBuildNormal)
        updateNormal(false);
    mutex.unlock();
}

void GLMeshData::setData(const std::vector<cv::Point3f>& vertex, const std::vector<unsigned int>& index)
{
    std::vector<GLfloat> vertex2(vertex.size()*3);
    for(int i = 0; i < vertex.size(); i++)
    {
        vertex2[i*3] = vertex[i].x;
        vertex2[i*3+1] = vertex[i].y;
        vertex2[i*3+2] = vertex[i].z;
    }
    setData(vertex2, index);
}

unsigned int GLMeshData::addVertex(cv::Point3f p)
{
    mutex.lock();
    unsigned int id = vertex.size()/3;
    vertex.push_back(p.x);
    vertex.push_back(p.y);
    vertex.push_back(p.z);
    current_version_vertex++;
    mutex.unlock();
    return id;
}

unsigned int GLMeshData::addVertex(cv::Point3f p, cv::Point3f color)
{
    mutex.lock();
    unsigned int id = vertex.size()/3;
    while(this->color.size() < vertex.size())
        this->color.push_back(1.0);
    vertex.push_back(p.x);
    vertex.push_back(p.y);
    vertex.push_back(p.z);
    this->color.push_back(color.x);
    this->color.push_back(color.y);
    this->color.push_back(color.z);
    current_version_vertex++;
    current_version_color++;
    mutex.unlock();
    return id;
}

void GLMeshData::addLine(unsigned int p1, unsigned int p2)
{
    mutex.lock();
    index.push_back(p1);
    index.push_back(p2);
    current_version_index++;
    mutex.unlock();
}

void GLMeshData::addLine(cv::Point3f p1, cv::Point3f p2, bool useIndex)
{
    unsigned int i1 = addVertex(p1);
    unsigned int i2 = addVertex(p2);
    if(useIndex)
        addLine(i1, i2);
}


void GLMeshData::addLine(cv::Point3f p1, cv::Point3f p2, cv::Point3f color, bool useIndex)
{
    unsigned int i1 = addVertex(p1, color);
    unsigned int i2 = addVertex(p2, color);
    if(useIndex)
        addLine(i1, i2);
}

void GLMeshData::addTriangle(unsigned int p1, unsigned int p2, unsigned int p3)
{
    mutex.lock();
    index.push_back(p1);
    index.push_back(p2);
    index.push_back(p3);
    current_version_index++;
    if(autoBuildNormal)
        updateNormal(false);//todo : avoid recompute everything
    mutex.unlock();
}

void GLMeshData::addTriangle(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, bool useIndex)
{
    unsigned int i1 = addVertex(p1);
    unsigned int i2 = addVertex(p2);
    unsigned int i3 = addVertex(p3);
    if(useIndex)
        addTriangle(i1, i2, i3);
}

void GLMeshData::addTriangle(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f color, bool useIndex)
{
    unsigned int i1 = addVertex(p1, color);
    unsigned int i2 = addVertex(p2, color);
    unsigned int i3 = addVertex(p3, color);
    if(useIndex)
        addTriangle(i1, i2, i3);
}

void GLMeshData::addQuad(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4)
{
    mutex.lock();
    if(type == GLMeshDataType::Triangles)
    {
        index.push_back(p1);
        index.push_back(p2);
        index.push_back(p3);
        index.push_back(p1);
        index.push_back(p3);
        index.push_back(p4);
    }
    else
    {
        index.push_back(p1);
        index.push_back(p2);
        index.push_back(p3);
        index.push_back(p4);
    }
    current_version_index++;
    if(autoBuildNormal)
        updateNormal(false);//todo : avoid recompute everything
    mutex.unlock();
}

void GLMeshData::addQuad(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f p4, bool useIndex)
{
    unsigned int i1 = addVertex(p1);
    unsigned int i2 = addVertex(p2);
    unsigned int i3 = addVertex(p3);
    unsigned int i4 = addVertex(p4);
    if(useIndex)
        addQuad(i1, i2, i3, i4);
}

void GLMeshData::addQuad(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f p4, cv::Point3f color, bool useIndex)
{
    unsigned int i1 = addVertex(p1, color);
    unsigned int i2 = addVertex(p2, color);
    unsigned int i3 = addVertex(p3, color);
    unsigned int i4 = addVertex(p4, color);
    if(useIndex)
        addQuad(i1, i2, i3, i4);
}

void GLMeshData::addCube(cv::Point3f p1, cv::Point3f dir1, cv::Point3f dir2, cv::Point3f dir3, bool useIndex)
{
    cv::Point3f list[8];
    unsigned int id[8];
    for(int i = 0; i < 8; i++)
    {
        cv::Point3f p = p1;
        if(i >= 4)
            p += dir1;
        if((i%4) >= 2)
            p += dir2;
        if(((i%4)==1||(i%4)==2))
            p += dir3;
        list[i] = p;
    }

    addQuad(list[0], list[1], list[2], list[3], useIndex);
    addQuad(list[0], list[4], list[7], list[3], useIndex);
    addQuad(list[0], list[4], list[5], list[1], useIndex);
    addQuad(list[3], list[7], list[6], list[2], useIndex);
    addQuad(list[1], list[2], list[6], list[5], useIndex);
    addQuad(list[4], list[5], list[6], list[7], useIndex);
}

void GLMeshData::addCube(cv::Point3f p1, cv::Point3f dir1, cv::Point3f dir2, cv::Point3f dir3, cv::Point3f color, bool useIndex)
{
    cv::Point3f list[8];
    unsigned int id[8];
    for(int i = 0; i < 8; i++)
    {
        cv::Point3f p = p1;
        if(i >= 4)
            p += dir1;
        if((i%4) >= 2)
            p += dir2;
        if(((i%4)==1||(i%4)==2))
            p += dir3;
        list[i] = p;
    }

    addQuad(list[0], list[1], list[2], list[3], color, useIndex);
    addQuad(list[0], list[4], list[7], list[3], color, useIndex);
    addQuad(list[0], list[4], list[5], list[1], color, useIndex);
    addQuad(list[3], list[7], list[6], list[2], color, useIndex);
    addQuad(list[1], list[2], list[6], list[5], color, useIndex);
    addQuad(list[4], list[5], list[6], list[7], color, useIndex);
}

void GLMeshData::compile(bool lock)
{
    printf("compile MeshData\n");
    if(lock)
        mutex.lock();
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("0 OpenGL error: %d\n", err);
    }
    if(current_version_vertex > built_version_vertex)
    {
        if(built_version_vertex < 0)
            glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*vertex.size(), &vertex[0], GL_STATIC_DRAW);
        built_version_vertex = current_version_vertex;
    }
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("1 OpenGL error: %d\n", err);
    }
    if(current_version_index > built_version_index)
    {
        if(built_version_index < 0)
            glGenBuffers(1, &elementbuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*index.size(), &index[0], GL_STATIC_DRAW);
        built_version_index = current_version_index;
    }
    if(current_version_color > built_version_color)
    {
        if(built_version_color < 0)
            glGenBuffers(1, &colorbuffer);
        while(color.size() < vertex.size())
            color.push_back(1.0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, colorbuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLfloat)*color.size(), &color[0], GL_STATIC_DRAW);
        built_version_color = current_version_color;
    }

    if(current_version_normal > built_version_normal)
    {
        if(built_version_normal < 0)
            glGenBuffers(1, &normalbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*normal.size(), &normal[0], GL_STATIC_DRAW);
        built_version_normal = current_version_normal;
    }
    if(lock)
        mutex.unlock();
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("2 OpenGL error: %d\n", err);
    }
}

void GLMeshData::render(std::shared_ptr<GLShaderProgram> shaderData)
{
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("render 0 OpenGL error: %d\n", err);
    }
    glEnableVertexAttribArray(shaderData->vertexAttribId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("render 0b OpenGL error: %d\n", err);
    }

    glVertexAttribPointer( shaderData->vertexAttribId, 3, GL_FLOAT, GL_FALSE, 0, NULL );
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("render 1 OpenGL error: %d\n", err);
    }

    if(shaderData->colorAttribId >= 0)
    {
        glEnableVertexAttribArray(shaderData->colorAttribId);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer( shaderData->colorAttribId, 3, GL_FLOAT, GL_FALSE, 0, NULL );
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("render 1b OpenGL error: %d\n", err);
        }
    }

    if(shaderData->normalAttribId >= 0)
    {
        glEnableVertexAttribArray(shaderData->normalAttribId);
        glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
        glVertexAttribPointer( shaderData->normalAttribId, 3, GL_FLOAT, GL_FALSE, 0, NULL );
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("render 1c OpenGL error: %d\n", err);
        }
    }

    if(index.size() > 0)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
        while((err = glGetError()) != GL_NO_ERROR)
        {
          printf("render 2 OpenGL error: %d\n", err);
        }
        if(type == GLMeshDataType::Points)
            glDrawElements(GL_POINTS, index.size(), GL_UNSIGNED_INT, (void*)0);
        else if(type == GLMeshDataType::Lines)
            glDrawElements(GL_LINES, index.size(), GL_UNSIGNED_INT, (void*)0);
        else if(type == GLMeshDataType::Triangles)
            glDrawElements(GL_TRIANGLES, index.size(), GL_UNSIGNED_INT, (void*)0);
        else glDrawElements(GL_QUADS, index.size(), GL_UNSIGNED_INT, (void*)0);
    }
    else
    {
        if(type == GLMeshDataType::Points)
            glDrawArrays(GL_POINTS, 0, vertex.size());
        else if(type == GLMeshDataType::Lines)
            glDrawArrays(GL_LINES, 0, vertex.size());
        else if(type == GLMeshDataType::Lines)
            glDrawArrays(GL_TRIANGLES, 0, vertex.size());
        else glDrawArrays(GL_QUADS, 0, vertex.size());
    }

    if(shaderData->colorAttribId >= 0)
        glDisableVertexAttribArray(shaderData->colorAttribId);
    if(shaderData->normalAttribId >= 0)
        glDisableVertexAttribArray(shaderData->normalAttribId);

    glDisableVertexAttribArray(shaderData->vertexAttribId);
    while((err = glGetError()) != GL_NO_ERROR)
    {
      printf("render 3 OpenGL error: %d\n", err);
    }
}

bool GLMeshData::isBuilt() const
{
    return built_version_vertex >= 0 && (current_version_index < 0 || built_version_index >= 0);
}

bool GLMeshData::isCurrentVersionBuilt() const
{
    return isBuilt() && built_version_vertex == current_version_vertex && built_version_index == current_version_index;
}

GLMesh::GLMesh(std::shared_ptr<GLMeshData> meshData)
    :meshData(meshData)
{
    normalMesh = nullptr;
    ambientColor = cv::Point3f(1,0,1);
    renderMode = GLMeshRenderMode::Solid;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            modelMat[i+j*4] = (i==j?1:0);
    picking = false;
}

void GLMesh::render(GLRenderer &renderer)
{
    if(renderMode == GLMeshRenderMode::Solid)
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    else if(renderMode == GLMeshRenderMode::Wireframe || meshData->type == GLMeshData::GLMeshDataType::Lines)
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    else if(renderMode == GLMeshRenderMode::Points || meshData->type == GLMeshData::GLMeshDataType::Points)
        glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );

    glDisable(GL_CULL_FACE);

    if(shaderData != nullptr)
    {
        shaderData->use();
        float vec3[3];

        if(shaderData->uniformAmbientColorId >= 0)
        {
            vec3[0] = ambientColor.x; vec3[1] = ambientColor.y; vec3[2] = ambientColor.z;
            glUniform3fv(shaderData->uniformAmbientColorId, 1, vec3);
        }

        if(shaderData->uniformDiffuseColorId >= 0)
        {
            vec3[0] = diffuseColor.x; vec3[1] = diffuseColor.y; vec3[2] = diffuseColor.z;
            glUniform3fv(shaderData->uniformDiffuseColorId, 1, vec3);
        }
        if(shaderData->uniformSpecularColorId >= 0)
        {
            vec3[0] = specularColor.x; vec3[1] = specularColor.y; vec3[2] = specularColor.z;
            glUniform3fv(shaderData->uniformSpecularColorId, 1, vec3);
        }
        if(shaderData->uniformShininessId >= 0)
            glUniform1f(shaderData->uniformShininessId, shininess);
        if(shaderData->uniformCameraPosId >= 0)
        {
            vec3[0] = renderer.cameraPos.x; vec3[1] = renderer.cameraPos.y; vec3[2] = renderer.cameraPos.z;
            glUniform3fv(shaderData->uniformCameraPosId, 1, vec3);
        }
        if(shaderData->uniformLightPosId >= 0)
        {
            vec3[0] = renderer.cameraPos.x; vec3[1] = renderer.cameraPos.y; vec3[2] = renderer.cameraPos.z;
            glUniform3fv(shaderData->uniformLightPosId, 1, vec3);
        }
        if(shaderData->uniformCameraId >= 0)
            glUniformMatrix4fv(shaderData->uniformCameraId, 1, GL_FALSE, &renderer.cameraMat[0]);
        if(shaderData->uniformModelMatId >= 0)
            glUniformMatrix4fv(shaderData->uniformModelMatId, 1, GL_FALSE, &modelMat[0]);
        if(shaderData->uniformNormalMatId >= 0)
        {
            GLfloat data[9];//only keep the rotation part
            glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(/*glm::make_mat4(renderer.viewMat)*/glm::make_mat4(modelMat))));
            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    data[i*3+j] = normalMat[i][j];
            glUniformMatrix3fv(shaderData->uniformNormalMatId, 1, GL_FALSE, &data[0]);
        }
        if(shaderData->uniformViewMatId >= 0)
            glUniformMatrix4fv(shaderData->uniformViewMatId, 1, GL_FALSE, &renderer.viewMat[0]);
        if(shaderData->uniformProjMatId >= 0)
            glUniformMatrix4fv(shaderData->uniformProjMatId, 1, GL_FALSE, &renderer.projMat[0]);

    }
    else printf("no shader\n");
    if(meshData != nullptr)
        meshData->render(shaderData);

    if(renderMode != GLMeshRenderMode::Solid)
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    if(normalMesh != nullptr)
    {
        printf("render normal\n");
        normalMesh->render(renderer);
    }
}

void GLMesh::setMesh(std::shared_ptr<GLMeshData> meshData)
{
    mutex.lock();
    this->meshData = meshData;
    mutex.unlock();
}

void GLMesh::setShader(std::shared_ptr<GLShaderProgram> shaderData)
{
    mutex.lock();
    this->shaderData = shaderData;
    mutex.unlock();
}

void GLMesh::setDrawNormals(bool drawNormals)
{
    if(normalMesh != nullptr && !drawNormals)
        normalMesh = nullptr;
    else if(normalMesh == nullptr && drawNormals)
    {
        std::shared_ptr<GLMeshData> meshData(new GLMeshData(GLMeshData::GLMeshDataType::Lines));
        normalMesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
        normalMesh->setShader(GLRendererMngr::GetInstance().getSimpleShader());
        normalMesh->ambientColor = cv::Point3f(1.0, 0.0, 0.0);
    }
}

void GLMesh::updateNormalMesh()
{
    if(normalMesh == nullptr)
        return ;
    if(meshData->current_version_normal > normalMesh->meshData->current_version_vertex)
    {
        std::vector<GLfloat> vertex(meshData->normal.size()*2);
        float length = 0.001;
        for(int i = 0; i < meshData->normal.size(); i+=3)
        {
            if(i%100 == 0)
            {
                printf("vertex %d meshData->vertex %d meshData->normal %d\n", vertex.size(), meshData->vertex.size(), meshData->normal.size());
                printf("vertex[%d*2+j] = meshData->vertex[%d+j]\n", i, i);
                printf("vertex[%d*2+3+j] = meshData->vertex[%d+j]+length*meshData->normal[%d+j]\n", i, i, i);
            }
            for(int j = 0; j < 3; j++)
            {
                vertex[i*2+j] = meshData->vertex[i+j];
                vertex[i*2+3+j] = meshData->vertex[i+j]+length*meshData->normal[i+j];
            }
        }
        std::vector<GLuint> index(vertex.size()/3);
        for(int i = 0; i < index.size(); i++)
            index[i] = i;
        normalMesh->meshData->setData(vertex, index);
        normalMesh->meshData->current_version_vertex = meshData->current_version_normal;
    }
}

GLRenderer::GLRenderer(GLWindow *win)
    :win(win)
{
    mostRecentRenderId = 0;
    pause = true;
    viewTheta = 2*CV_PI/4;//CV_PI/5;
    viewPhi = 0;//CV_PI/5;
    cameraPos = cv::Point3f(0,0,1);
    lightPosSet = false;
    near = 0.01;
    far = 100.0;
    GLRenderer *this_ptr = this;
    win->listMouseCallback.push_back([this_ptr](bool leftDown, bool rightDown, int x, int y, double scrollX, double scrollY)
        {
            this_ptr->onMouseEvent(leftDown, rightDown, x, y, scrollX, scrollY);
        });
    win->listKeyboardCallback.push_back([this_ptr](int key, int action)
        {
            this_ptr->onKeyboardEvent(key, action);
        });
}

void GLRenderer::addMesh(std::shared_ptr<GLMesh> mesh)
{
    mutex.lock();
    listMesh.push_back(mesh);
    mutex.unlock();
}

void GLRenderer::render(bool use_lock)
{
    printf("render\n");
    printf("center = %lf,%lf,%lf\n", cameraPos.x, cameraPos.y, cameraPos.z);
    printf("view = %lf,%lf\n", viewTheta, viewPhi);
    if(use_lock)
        mutex.lock();
    pause = false;
    mostRecentRenderId++;
    int id = mostRecentRenderId;
    if(use_lock)
        mutex.unlock();
    GLRendererMngr::GetInstance().doInGLThread(
      [id](void *args)
        {
          GLRenderer *renderer = (GLRenderer*)args;
          renderer->mutex.lock();
          if(renderer->pause || id < renderer->mostRecentRenderId)
          {
              renderer->mutex.unlock();
              return;
          }

          cv::Size size = renderer->getSize();
          cv::Mat modelView = renderer->genModelviewMatrix();

          cv::Mat proj = cv::Mat::zeros(4,4,CV_32F);
          float e = 1.0/tan(0.5*CV_PI/2);
          float r = renderer->near/e, l = -r;
          float t = r*size.height/size.width, b = -t;
          float epsilon = 2.4*10e-7;
          proj.at<float>(0,0) = renderer->near/r;//2*renderer->near/(r-l);
          proj.at<float>(0,2) = 0;//(r+l)/(r-l);
          proj.at<float>(1,1) = renderer->near/t;//2*renderer->near/(t-b);
          proj.at<float>(1,2) = 0;//(t+b)/(t-b);
          proj.at<float>(2,2) = -(renderer->far+renderer->near)/(renderer->far-renderer->near);
          proj.at<float>(2,3) = -2*renderer->far*renderer->near/(renderer->far-renderer->near);
          proj.at<float>(3,2) = -1;
          proj = proj*modelView;

          float ct = cos(renderer->viewTheta), st = sin(renderer->viewTheta);
          float cp = cos(renderer->viewPhi), sp = sin(renderer->viewPhi);
          cv::Point3f look = renderer->cameraPos+cv::Point3f(cp*st,sp,cp*ct);

          glm::mat4 Projection = glm::perspective(45.0f, float(size.width) / size.height, renderer->near, renderer->far);
          // Matrice de la caméra
          glm::mat4 View       = glm::lookAt(
              glm::vec3(renderer->cameraPos.x,renderer->cameraPos.y,renderer->cameraPos.z), // La caméra est à (4,3,3), dans l'espace monde
              glm::vec3(look.x, look.y, look.z), // et regarde l'origine
              glm::vec3(0,1,0)  // La tête est vers le haut (utilisez 0,-1,0 pour regarder à l'envers)
          );
          // Notre matrice ModelViewProjection : la multiplication des trois  matrices
          glm::mat4 MVP        = Projection * View;


          for(int i = 0; i < 4; i++)
              for(int j = 0; j < 4; j++)
                  renderer->projMat[j*4+i] = Projection[j][i];//proj.at<float>(i,j);

          for(int i = 0; i < 4; i++)
              for(int j = 0; j < 4; j++)
                  renderer->viewMat[j*4+i] = View[j][i];

          for(int i = 0; i < 4; i++)
              for(int j = 0; j < 4; j++)
                  renderer->cameraMat[j*4+i] = MVP[j][i];

          printf("update\n");
          renderer->do_update(id, false);
          GLenum err;
          while((err = glGetError()) != GL_NO_ERROR)
          {
            printf("OpenGL error: %d\n", err);
          }
          printf("render\n");
          renderer->do_render(id, false);

          while((err = glGetError()) != GL_NO_ERROR)
          {
            printf("OpenGL error: %d\n", err);
          }
          renderer->mutex.unlock();
        }, this, NULL);
}

void GLRenderer::do_update(int id, bool lock)
{
    printf("do_update\n");
    if(lock)
        mutex.lock();
    if(!pause)
    {
        for(int i = 0; i < listMesh.size(); i++)
        {
            if(!listMesh[i]->meshData->isCurrentVersionBuilt())
                listMesh[i]->meshData->compile();
            if(!listMesh[i]->shaderData->isCurrentVersionBuilt())
                listMesh[i]->shaderData->compile();
            if(listMesh[i]->normalMesh != nullptr)
            {
                listMesh[i]->updateNormalMesh();
                if(!listMesh[i]->normalMesh->meshData->isCurrentVersionBuilt())
                    listMesh[i]->normalMesh->meshData->compile();
                if(!listMesh[i]->normalMesh->shaderData->isCurrentVersionBuilt())
                    listMesh[i]->normalMesh->shaderData->compile();
            }
        }
    }
    if(lock)
        mutex.unlock();
}

void GLRenderer::do_render(int id, bool lock)
{
    printf("do_render\n");
    if(lock)
        mutex.lock();
    if(!pause)
    {
        win->makeCurrent();
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for(int i = 0; i < listMesh.size(); i++)
        {
            listMesh[i]->render(*this);
        }
        win->swapBuffer();
    }
    if(lock)
        mutex.unlock();
}

void GLRenderer::startRenderer()
{
    mutex.lock();
    pause = false;
    mutex.unlock();
}

void GLRenderer::pauseRenderer()
{
    mutex.lock();
    pause = true;
    mutex.unlock();
}

template<typename T, int n>
cv::Point3d multVectorByMatT(cv::Mat M, cv::Point3d p)
{
    cv::Point3d res(0,0,0);
    T in[n];
    in[0] = p.x; in[1] = p.y; in[2] = p.z;
    if(n == 4)
        in[3]= 1;
    for(int i = 0; i < n; i++)
    {
        res.x += M.at<T>(0,i)*in[i];
        res.y += M.at<T>(1,i)*in[i];
        res.z += M.at<T>(2,i)*in[i];
    }
    if(n == 4 && M.rows == 4)
    {
        T w = M.at<T>(3,0)*in[0]+M.at<T>(3,1)*in[1]+M.at<T>(3,2)*in[2]+M.at<T>(3,3);
        res /= w;
    }
    return res;
}


cv::Point3d multVectorByMat(cv::Mat M, cv::Point3d p)
{
    if(M.type() == CV_32F)
    {
        if(M.cols == 3)
            return multVectorByMatT<float, 3>(M, p);
        else if(M.cols == 4)
            return multVectorByMatT<float, 4>(M, p);
    }
    else if(M.type() == CV_64F)
    {
        if(M.cols == 3)
            return multVectorByMatT<double, 3>(M, p);
        else if(M.cols == 4)
            return multVectorByMatT<double, 4>(M, p);
    }
    printf("input error!!!!\n");
    return cv::Point3d(0,0,0);
}

void GLRenderer::onMouseEvent(bool leftDown, bool rightDown, double x, double y, double scrollX, double scrollY)
{
    //printf("left %d, right %d, x %lf, y %lf, Scroll %lf %lf\n", leftDown, rightDown, x, y, scrollX, scrollY);
    if(rightDown)
    {
        viewTheta -= win->deltaMouse(x, y).x * CV_PI/1800;
        viewPhi -= win->deltaMouse(x, y).y * CV_PI/1800;
        viewPhi = std::max(-CV_PI/2, std::min(CV_PI/2, viewPhi));
    }
    else if(scrollY != 0)
    {
        float ct = cos(viewTheta), st = sin(viewTheta);
        float cp = cos(viewPhi), sp = sin(viewPhi);
        cv::Point3f look(cp*st,sp,cp*ct);
        cameraPos += 0.01*scrollY*look;
    }
    else return;
    mutex.lock();
    if(!pause)
        render(false);
    mutex.unlock();
}

void GLRenderer::onKeyboardEvent(int key, int action)
{
    printf("key %d, action %d\n", key, action);
    if(action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        float alpha = 0.01;
        float ct = cos(viewTheta), st = sin(viewTheta);
        float cp = cos(viewPhi), sp = sin(viewPhi);
        cv::Point3f look(cp*st,sp,cp*ct);
        cv::Point3f left = cv::Point3f(0,1,0).cross(look);
        if(key == GLFW_KEY_UP)
            cameraPos += alpha*look;
        else if(key == GLFW_KEY_DOWN)
            cameraPos -= alpha*look;
        else if(key == GLFW_KEY_LEFT)
            cameraPos += alpha*left;
        else if(key == GLFW_KEY_RIGHT)
            cameraPos -= alpha*left;
        else return;
    }
    else return;
    mutex.lock();
    if(!pause)
        render(false);
    mutex.unlock();
}

cv::Mat GLRenderer::genModelviewMatrix()
{
    float ct = cos(viewTheta), st = sin(viewTheta);
    float cp = cos(viewPhi), sp = sin(viewPhi);
    cv::Point3f n(-cp*st,-sp,-cp*ct);
    cv::Point3f u = cv::Point3f(0,1,0).cross(n);
    cv::Point3f v = n.cross(u);
    cv::Point3f n_norm = n/sqrt(n.dot(n));
    cv::Point3f u_norm = u/sqrt(u.dot(u));
    cv::Point3f v_norm = v/sqrt(v.dot(v));
    cv::Mat res = cv::Mat::eye(4,4,CV_32F);
    res.at<float>(0,0) = u.x; res.at<float>(0,1) = u.y; res.at<float>(0,2) = u.z; res.at<float>(0,3) = -cameraPos.dot(u_norm);
    res.at<float>(1,0) = v.x; res.at<float>(1,1) = v.y; res.at<float>(1,2) = v.z; res.at<float>(1,3) = -cameraPos.dot(v_norm);
    res.at<float>(2,0) = n.x; res.at<float>(2,1) = n.y; res.at<float>(2,2) = n.z; res.at<float>(2,3) = -cameraPos.dot(n_norm);
    return res;
}

cv::Size GLRenderer::getSize() const
{
    return win->getSize();
}

void GLRenderer::setCameraPos(cv::Point3f pos)
{
    cameraPos = pos;
}

void GLRenderer::setLightPos(cv::Point3f pos)
{
    lightPos = pos;
    lightPosSet = true;
}

GLRendererMngr& GLRendererMngr::GetInstance()
{
    std::call_once(m_onceFlagInit,
        []{
            m_instance.reset(new GLRendererMngr);
            m_instance.get()->running = true;
            m_instance.get()->GLThread = std::thread(&GLRendererMngr::GLThreadFunc,m_instance.get());
            std::promise<bool> finish;
            m_instance.get()->doInGLThread([](void *)
            {
                if( !glfwInit() )
                    printf("failed to init glfw\n");
                GLenum err;
                while((err = glGetError()) != GL_NO_ERROR)
                {
                  printf("GetInstance 0 OpenGL error: %d\n", err);
                }
                glfwWindowHint(GLFW_SAMPLES, 4);
                while((err = glGetError()) != GL_NO_ERROR)
                {
                  printf("GetInstance 1 OpenGL error: %d\n", err);
                }
            }, NULL, &finish);
            finish.get_future().get();
        });
    return *m_instance.get();
}

GLRendererMngr::GLRendererMngr()
{
}

GLRendererMngr::~GLRendererMngr()
{
    running = false;
    GLThread.join();
}

void GLRendererMngr::GLThreadFunc()
{
    printf("Thread: start\n");
    while(running)
    {
        mngrMutex.lock();
        while(todoInGlThread.size() > 0)
        {
            printf("Thread: execute func\n");
            FuncArgPromise func = todoInGlThread[0];
            todoInGlThread.erase(todoInGlThread.begin());
            mngrMutex.unlock();
            func.func(func.arg);
            if(func.promise != NULL)
                func.promise->set_value(true);
            mngrMutex.lock();
        }
        for(int i = 0; i < listRenderer.size(); i++)
        {
            std::shared_ptr<GLRenderer> renderer = listRenderer[i];
            mngrMutex.unlock();
            renderer->win->pollEvents();
            mngrMutex.lock();
        }
        mngrMutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void GLRendererMngr::initGlew()
{
    std::call_once(m_onceFlagGlewInit,[]() {
    });
}


void GLRendererMngr::doInGLThread(std::function<void (void *)> func, void *arg, std::promise<bool>* finish)
{
    printf("do in GL Thread\n");
    mngrMutex.lock();
    #if 0
    func(arg);
    if(finish != NULL)
        finish->set_value(true);
    #else
    todoInGlThread.push_back(FuncArgPromise(func, arg, finish));
    #endif
    mngrMutex.unlock();
}

std::shared_ptr<GLRenderer> GLRendererMngr::createRenderer(GLWindow *win)
{
    printf("create renderer : create win\n");
    if(win == NULL)
        win = new GLWindow_GLFW();
    printf("create renderer : add to list");
    mngrMutex.lock();
    std::shared_ptr<GLRenderer> renderer(new GLRenderer(win));
    listRenderer.push_back(renderer);
    mngrMutex.unlock();
    return renderer;
}

std::string getFileContent(std::string filename)
{
    std::string code;
    std::ifstream stream(filename.c_str(), std::ios::in);
    if(stream.is_open()){
        std::string Line = "";
        while(getline(stream, Line))
            code += "\n" + Line;
        stream.close();
    }
    return code;
}

void GLRendererMngr::loadShader(std::shared_ptr<GLShaderProgram>* shader, const char *vertShaderFilename, const char *fragShaderFilename)
{
    if(*shader == nullptr)
    {
        rscMutex.lock();
        if(*shader == nullptr)
        {
            std::string VertexShaderCode = getFileContent(vertShaderFilename);
            std::string FragmentShaderCode = getFileContent(fragShaderFilename);
            std::shared_ptr<GLShaderData> vertShader(new GLShaderData(GL_VERTEX_SHADER, VertexShaderCode.c_str()));
            std::shared_ptr<GLShaderData> fragShader(new GLShaderData(GL_FRAGMENT_SHADER, FragmentShaderCode.c_str()));
            *shader = std::shared_ptr<GLShaderProgram>(new GLShaderProgram());
            (*shader)->setShader(vertShader, fragShader);
        }
        rscMutex.unlock();
    }
}

std::shared_ptr<GLShaderProgram> GLRendererMngr::getSimpleShader()
{
    printf("getSimpleShader\n");
    loadShader(&simpleShader, "data/simpleShader.vert", "data/simpleShader.frag");
    return simpleShader;
}

std::shared_ptr<GLShaderProgram> GLRendererMngr::getSimpleShaderWithVertexColor()
{
    printf("getSimpleShaderWithVertexColor\n");
    loadShader(&simpleShaderWithVertexColor, "data/simpleShaderWithVertexColor.vert", "data/simpleShader.frag");
    return simpleShaderWithVertexColor;
}

std::shared_ptr<GLShaderProgram> GLRendererMngr::getPhongShader()
{
    printf("getPhongShader\n");
    loadShader(&phongShader, "data/phongShader.vert", "data/phongShader.frag");
    return phongShader;
}


std::shared_ptr<GLRenderer> GLRendererMngr::getRendererByVoidPtr(void *ptr)
{
    for(int i = 0; i < listRenderer.size(); i++)
    {
        if(listRenderer[i]->win->getVoidPtr() == ptr)
            return listRenderer[i];
    }
    return NULL;
}
