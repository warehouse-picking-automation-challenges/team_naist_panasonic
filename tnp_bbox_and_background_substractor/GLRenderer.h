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

#ifndef GLRENDERER_H
#define GLRENDERER_H

#include <opencv2/opencv.hpp>
#include <epoxy/gl.h>
#include <epoxy/glx.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <thread>
#include <mutex>
#include <future>

class GLRenderer;

class GLWindow
{
public:
    GLWindow();
    virtual void makeCurrent() = 0;
    virtual void swapBuffer() = 0;
    virtual void pollEvents() = 0;
    virtual void destroyWindow() = 0;
    virtual void* getVoidPtr() = 0;
    virtual cv::Size getSize() = 0;
    virtual bool shouldClose()
    {
        return false;
    }
    virtual void onMouseEvent(bool leftDown, bool rightDown, double x, double y, double scrollX, double scrollY)
    {
        for(int i = 0; i < listMouseCallback.size(); i++)
            listMouseCallback[i](leftDown, rightDown, x, y, scrollX, scrollY);
    }

    virtual void onKeyboardEvent(int key,int action)
    {
        for(int i = 0; i < listKeyboardCallback.size(); i++)
            listKeyboardCallback[i](key, action);
    }

    cv::Point2d deltaMouse(double mouseX, double mouseY);
    cv::Point2d deltaScrollMouse(double scrollX, double scrollY);

    bool oldLeftDownInit, oldRightDownInit, oldXInit, oldYInit, oldScrollXInit, oldScrollYInit;
    bool oldLeftDown, oldRightDown;
    double oldX, oldY;
    double oldScrollX, oldScrollY;
    cv::Size viewportSize;
    std::vector<std::function<void (bool, bool, double, double, double, double)> > listMouseCallback;
    std::vector<std::function<void (int, int)> > listKeyboardCallback;
};

class GLWindow_GLFW : public GLWindow
{
public:
    GLWindow_GLFW(int w = 1024, int h = 768, const char *title = "GLwin");
    virtual void makeCurrent();
    virtual void swapBuffer();
    virtual void pollEvents();
    virtual void destroyWindow();
    virtual void* getVoidPtr();
    virtual cv::Size getSize();
    virtual bool shouldClose();

    GLFWwindow* window;
};

class GLShaderData
{
public:
    GLShaderData(GLenum shaderType, std::string shader = "");
    ~GLShaderData();
    void setShader(std::string shader);
    void compile(bool lock = true);
    bool isBuilt() const;
    bool isCurrentVersionBuilt() const;

    std::string shader;
    GLuint shaderId;
    GLenum shaderType;
    int current_version;
    int built_version;
    std::mutex mutex;
};

class GLShaderProgram
{
public:
    GLShaderProgram();
    void setShader(std::shared_ptr<GLShaderData> vertexShader, std::shared_ptr<GLShaderData> fragmentShader);
    void compile(bool lock = true);
    void use();
    bool isBuilt() const;
    bool isCurrentVersionBuilt() const;

    std::shared_ptr<GLShaderData> vertexShader, fragmentShader;
    GLuint programId;
    GLint vertexAttribId;
    GLint normalAttribId;
    GLint colorAttribId;
    GLint uniformAmbientColorId;
    GLint uniformDiffuseColorId;
    GLint uniformSpecularColorId;
    GLint uniformShininessId;
    GLint uniformCameraPosId;
    GLint uniformLightPosId;
    GLint uniformCameraId;
    GLint uniformModelMatId;
    GLint uniformNormalMatId;
    GLint uniformViewMatId;
    GLint uniformProjMatId;
    int current_version;
    int built_version;
    std::mutex mutex;
};

class GLMeshData
{
public:
    enum class GLMeshDataType {Points, Lines, Triangles, Quads};
    GLMeshData(GLMeshDataType type = GLMeshDataType::Triangles);
    void setData(const std::vector<GLfloat>& vertex, const std::vector<unsigned int>& index = std::vector<unsigned int>(), const std::vector<GLfloat> &color= std::vector<GLfloat>());
    void setData(const std::vector<cv::Point3f>& vertex, const std::vector<unsigned int>& index = std::vector<unsigned int>());
    void updateNormal(bool useLock = true);
    void setAutoBuildNormal(bool autoBuildNormal);
    void render(std::shared_ptr<GLShaderProgram> shaderData);
    void compile(bool lock = true);
    bool isBuilt() const;
    bool isCurrentVersionBuilt() const;

    void clear();
    unsigned int addVertex(cv::Point3f p);
    unsigned int addVertex(cv::Point3f p, cv::Point3f color);
    void addLine(unsigned int p1, unsigned int p2);
    void addLine(cv::Point3f p1, cv::Point3f p2, bool useIndex = true);
    void addLine(cv::Point3f p1, cv::Point3f p2, cv::Point3f color, bool useIndex = true);
    void addTriangle(unsigned int p1, unsigned int p2, unsigned int p3);
    void addTriangle(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, bool useIndex = true);
    void addTriangle(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f color, bool useIndex = true);
    void addQuad(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4);
    void addQuad(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f p4, bool useIndex = true);
    void addQuad(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f p4, cv::Point3f color, bool useIndex = true);
    void addCube(cv::Point3f p1, cv::Point3f dir1, cv::Point3f dir2, cv::Point3f dir3, bool useIndex = true);
    void addCube(cv::Point3f p1, cv::Point3f dir1, cv::Point3f dir2, cv::Point3f dir3, cv::Point3f color, bool useIndex = true);

    std::vector<GLfloat> vertex;
    std::vector<GLfloat> normal;
    std::vector<GLfloat> color;
    std::vector<unsigned int> index;
    GLuint vertexbuffer;
    GLuint normalbuffer;
    GLuint colorbuffer;
    GLuint elementbuffer;
    int current_version_vertex, current_version_index, current_version_normal, current_version_color;
    int built_version_vertex, built_version_index, built_version_normal, built_version_color;
    bool autoBuildNormal;
    std::mutex mutex;
    GLMeshDataType type;
};

class GLMesh
{
public:
    enum class GLMeshRenderMode {Points, Wireframe, Solid};
    GLMesh(std::shared_ptr<GLMeshData> meshData = nullptr);
    void render(GLRenderer& renderer);
    void setMesh(std::shared_ptr<GLMeshData> meshData);
    void setShader(std::shared_ptr<GLShaderProgram> shaderData);
    void setDrawNormals(bool drawNormals);
    void updateNormalMesh();

    std::shared_ptr<GLMeshData> meshData;
    std::shared_ptr<GLShaderProgram> shaderData;
    std::shared_ptr<GLMesh> normalMesh;//mesh for drawing the normals
    cv::Point3f ambientColor;
    cv::Point3f diffuseColor;
    cv::Point3f specularColor;
    float shininess;
    GLfloat modelMat[16];
    GLMeshRenderMode renderMode;
    std::mutex mutex;
    bool picking;
};

class GLRenderer
{
public:
    GLRenderer(GLWindow *win);
    void addMesh(std::shared_ptr<GLMesh> mesh);
    void render(bool use_lock = true);

    GLWindow *win;
    std::mutex mutex;
    std::vector<std::shared_ptr<GLMesh> > listMesh;
    bool pause;
    int mostRecentRenderId;
    double viewTheta, viewPhi;
    cv::Point3f cameraPos;
    cv::Point3f lightPos;
    bool lightPosSet;
    float near, far;
    GLfloat viewMat[16];
    GLfloat projMat[16];
    GLfloat cameraMat[16];//projMat*viewMat
    void onMouseEvent(bool leftDown, bool rightDown, double x, double y, double scrollX, double scrollY);
    void onKeyboardEvent(int key, int action);
    cv::Mat genModelviewMatrix();
    cv::Size getSize() const;
    void pauseRenderer();
    void startRenderer();
    void setCameraPos(cv::Point3f pos);
    void setLightPos(cv::Point3f pos);

private:
    void do_render(int id, bool lock = true);
    void do_update(int id, bool lock = true);
};

class FuncArgPromise
{
public:
    FuncArgPromise(std::function<void (void *)> func, void *arg, std::promise<bool> *promise)
        :func(func), arg(arg), promise(promise)
    {
    }

    std::function<void (void *)> func;
    void *arg;
    std::promise<bool> *promise;
};

class GLRendererMngr
{
public:
    std::thread GLThread;
    std::vector<std::shared_ptr<GLRenderer> > listRenderer;

    void initGlew();
    void doInGLThread(std::function<void (void *)> func, void* arg, std::promise<bool>* finish);
    std::shared_ptr<GLRenderer> getRendererByVoidPtr(void *ptr);
    std::shared_ptr<GLRenderer> createRenderer(GLWindow *win = NULL);
    void loadShader(std::shared_ptr<GLShaderProgram>* shader, const char *vertShaderFilename, const char *fragShaderFilename);
    std::shared_ptr<GLShaderProgram> getSimpleShader();
    std::shared_ptr<GLShaderProgram> getSimpleShaderWithVertexColor();
    std::shared_ptr<GLShaderProgram> getPhongShader();
    ~GLRendererMngr();
    static GLRendererMngr& GetInstance();

private:
    static std::unique_ptr<GLRendererMngr> m_instance;
    static std::once_flag m_onceFlagInit;
    static std::once_flag m_onceFlagGlewInit;
    std::vector<FuncArgPromise> todoInGlThread;
    std::mutex mngrMutex, rscMutex;
    bool running;

    GLRendererMngr();
    void GLThreadFunc();
    std::shared_ptr<GLShaderProgram> simpleShader, simpleShaderWithVertexColor, phongShader;
};

#endif // GLRENDERER_H
