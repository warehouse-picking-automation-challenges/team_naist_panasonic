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

#include "VisualHull.h"
#include "UtilCvPclRs.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include "GLRenderer.h"

using MouseAction = std::function<void(int, int, int, int)>;

class OctreeVoxelElem
{
public:
    unsigned char filled;
    bool isLeaf;
    int sub[8];

    OctreeVoxelElem()
    {
        filled = 255;
        isLeaf = true;
        for(int i = 0; i < 8; i++)
            sub[i] = -1;
    }

    bool isFilled(int id) const
    {
        return (filled>>id)&1;
    }

    void setFilled(int id, bool val)
    {
        filled = (filled&(~(1<<id))) | (val<<id);
    }
};

class OctreeVoxel
{
    cv::Point3d offset;
    double size;
    int nbSubDiv;

    std::vector<OctreeVoxelElem> listVoxels;

    OctreeVoxel(cv::Point3d offset = cv::Point3d(0,0,0), double size = 10.0, int nbSubDiv = 10)
        :offset(offset), size(size), nbSubDiv(nbSubDiv)
    {
        listVoxels.push_back(OctreeVoxelElem());
    }
};

class OctreeVoxelIterator
{
public:
    OctreeVoxel *voxel;
    std::vector<int> tree;
    std::vector<unsigned char> currentBranch;
    int currentDepth;

    OctreeVoxelIterator(OctreeVoxel *voxel)
        :voxel(voxel)
    {
        currentDepth = 0;
    }

    void goToLeaf()
    {

    }

    void nextLeaf()
    {

    }
};

class VisualHullMouseHandler
{
public:
    enum class SelectionMode { selectionRect, selectionPoly, selectionSuperpixels};
    cv::Mat img,maskRGB;
    cv::Point startSelect;
    cv::Point endSelect;
    SelectionMode selectionMode;
    std::vector<cv::Point> polySelect;
    bool valid;
    cv::Ptr<cv::ximgproc::SuperpixelSEEDS> seeds;

    VisualHullMouseHandler(cv::Mat img, cv::Mat maskRGB)
    {
        this->img = img;
        this->maskRGB = maskRGB;
        startSelect = cv::Point(-1,-1);
        endSelect = cv::Point(-1,-1);
        valid = false;
    }

    void setSelectionRect()
    {
        selectionMode = SelectionMode::selectionRect;
        polySelect.clear();
    }

    void setSelectionPoly()
    {
        selectionMode = SelectionMode::selectionPoly;
        startSelect.x = -1;
    }

    void setSelectionSuperpixels()
    {
        selectionMode = SelectionMode::selectionSuperpixels;
        startSelect.x = -1;
        int num_iterations = 4;
        int prior = 2;
        bool double_step = false;
        int num_superpixels = 400;
        int num_levels = 4;
        int num_histogram_bins = 5;

        seeds = cv::ximgproc::createSuperpixelSEEDS(img.cols, img.rows, img.channels(), num_superpixels, num_levels, prior, num_histogram_bins, double_step);
        seeds->iterate(img, num_iterations);
    }

    void setValid()
    {
        valid = true;
    }

    void onMouse(int event, int x, int y, int flags)
    {
        std::cout << "mouse action!" << std::endl;
        if  ( event == cv::EVENT_LBUTTONDOWN )
        {
            if(selectionMode == SelectionMode::selectionRect)
            {
                startSelect.x = x;
                startSelect.y = y;
                endSelect.x = x;
                endSelect.y = y;
            }
            else if(selectionMode == SelectionMode::selectionPoly)
            {
                polySelect.push_back(cv::Point2f(x,y));
            }
            else if(selectionMode == SelectionMode::selectionSuperpixels)
            {
                cv::Mat labels;
                seeds->getLabels(labels);
                unsigned int label = labels.at<unsigned int>(y,x);
                for(int i = 0; i < maskRGB.rows; i++)
                {
                    for(int j = 0; j < maskRGB.cols; j++)
                    {
                        if(labels.at<unsigned int>(i,j)==label)
                            maskRGB.at<unsigned char>(i,j) = 0;
                    }
                }
            }

        }
        else if  ( event == cv::EVENT_MOUSEMOVE)
        {
            if(selectionMode == SelectionMode::selectionRect && (flags & cv::EVENT_FLAG_LBUTTON) && startSelect.x >=0 && startSelect.y >=0 )
            {
                endSelect.x = x;
                endSelect.y = y;
            }
        }
    }

    void onKeyboard(int key)
    {
        int minX = std::max(0, std::min(startSelect.x, endSelect.x));
        int minY = std::max(0, std::min(startSelect.y, endSelect.y));
        int maxX = std::min(maskRGB.cols-1, std::max(startSelect.x, endSelect.x));
        int maxY = std::min(maskRGB.rows-1, std::max(startSelect.y, endSelect.y));

        if(key == 'c')
        {
            if(selectionMode == SelectionMode::selectionRect)
            {
                if(startSelect.x >= 0)
                {
                    cv::Mat mask = cv::Mat::zeros(maskRGB.size(), CV_8UC1);
                    cv::Rect rect(minX, minY, maxX-minX+1, maxY-minY+1);
                    maskRGB(rect).copyTo(mask(rect));
                    mask.copyTo(maskRGB);
                }
                startSelect.x = -1;
            }
            else if(selectionMode == SelectionMode::selectionPoly)
            {
                cv::Mat mask = cv::Mat::zeros(maskRGB.size(), CV_8UC1);
                std::vector<std::vector<cv::Point> > list;
                list.push_back(polySelect);
                cv::drawContours(mask, list, 0, cv::Scalar(255), -1);
                cv::Mat mask2 = cv::Mat::zeros(maskRGB.size(), CV_8UC1);
                maskRGB.copyTo(mask2, mask);
                mask2.copyTo(maskRGB);
                polySelect.clear();
            }
        }
        else if(key == 'd')
        {
            if(selectionMode == SelectionMode::selectionRect)
            {
                if(startSelect.x >= 0)
                {
                    cv::Rect rect(minX, minY, maxX-minX+1, maxY-minY+1);
                    maskRGB(rect).setTo(cv::Scalar(0));
                }
                startSelect.x = -1;
            }
            else if(selectionMode == SelectionMode::selectionPoly)
            {
                std::vector<std::vector<cv::Point> > list;
                list.push_back(polySelect);
                cv::drawContours(maskRGB, list, 0, cv::Scalar(0), -1);
                polySelect.clear();
            }
        }
        else printf("key %d\n", key);
    }
};

template<typename T>
class ToolbarWinCV
{
public:
    ToolbarWinCV(std::string name, T *winMouseHandler);
    ~ToolbarWinCV()
    {
        cv::setMouseCallback(name, NULL, NULL);
    }

    void draw(cv::Mat img)
    {
        int w = std::max((int)icons.size()*iconSize, img.cols);
        cv::Mat img2 = cv::Mat::zeros(iconSize+img.rows, w, CV_8UC3);
        for(int i = 0; i < icons.size(); i++)
        {
            cv::Mat icon;
            cv::resize(icons[i], icon, cv::Size(iconSize, iconSize));
            icon.copyTo(img2(cv::Rect(i*iconSize, 0, iconSize, iconSize)));
        }
        img.copyTo(img2(cv::Rect(0,iconSize,img.cols, img.rows)));
        cv::imshow(name, img2);
    }

    void onMouse(int event, int x, int y, int flags)
    {
        if(y < iconSize)
        {
            if(x < listCallback.size()*iconSize && event == cv::EVENT_LBUTTONDOWN)
                listCallback[x/iconSize](winMouseHandler);
        }
        else winCallback(winMouseHandler, event, x, y-iconSize, flags);
    }

    void setWinCallback(std::function<void(T*,int,int,int,int)> callback)
    {
        winCallback = callback;
    }

    void addButton(cv::Mat icon, std::function<void(T*)> callback)
    {
        icons.push_back(icon);
        listCallback.push_back(callback);
    }

    std::string name;
    std::vector<cv::Mat> icons;
    T* winMouseHandler;
    std::function<void(T*,int,int,int,int)> winCallback;
    std::vector<std::function<void(T*)> > listCallback;
    int iconSize;
};

template<typename T>
void ToolbarWinCVMouseCallback(int event, int x, int y, int flags, void* userdata)
{
    ((ToolbarWinCV<T>*)userdata)->onMouse(event, x, y, flags);
}

template<typename T>
ToolbarWinCV<T>::ToolbarWinCV(std::string name, T* winMouseHandler)
    :name(name), winMouseHandler(winMouseHandler), iconSize(20)
{
    cv::namedWindow(name);
    cv::setMouseCallback(name, ToolbarWinCVMouseCallback<T>, (void*)this);
}

cv::Mat multiplyMask(cv::Mat mask1, cv::Mat mask2)
{
    cv::Mat mask(mask1.size(), mask1.type());
    for(int i = 0; i < mask.rows; i++)
    {
        unsigned char *dst = mask.ptr<unsigned char>(i);
        unsigned char *src1 = mask1.ptr<unsigned char>(i);
        unsigned char *src2 = mask2.ptr<unsigned char>(i);
        for(int j = 0; j < mask.cols; j++)
        {
            dst[j] = ((int)src1[j]*src2[j])/255;
        }
    }
    return mask;
}

std::vector<cv::Point3f> cutTriangleByZPlane(const std::vector<cv::Point3f>& tri, float minZ)
{
    std::vector<cv::Point3f> result;
    for(int i = 0; i < tri.size(); i++)
    {
        cv::Point3f p1 = tri[i];
        cv::Point3f p2 = tri[(i+1)%tri.size()];
        if(p1.z >= minZ && p2.z >= minZ)
            result.push_back(p1);
        else if(p1.z >= minZ || p2.z >= minZ)
        {
            if(p1.z >= minZ)
                result.push_back(p1);

            float d = p2.z-p1.z;
            float a = (minZ-p1.z)/d;
            result.push_back(p1+(p2-p1)*a);
        }
    }
    return result;
}

std::vector<cv::Mat> editFrameVisualHull(std::vector<RgbdFrame> &listFrame, cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, std::vector<cv::Mat> KRGB, std::vector<cv::Mat> poseRGB, std::vector<cv::Mat> firstFramePose, cv::Mat boardToWorld)
{
    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    std::shared_ptr<GLRenderer> renderer;
    renderer = renderMngr.createRenderer();
    renderer->cameraPos = cv::Point3f(2.537867,-0.426777,1.187069);
    renderer->viewTheta =  -1.887569;
    renderer->viewPhi = 0.263948;
    std::shared_ptr<GLMeshData> boardMeshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    boardMeshData->type = GLMeshData::GLMeshDataType::Points;
    std::shared_ptr<GLMesh> boardMesh = std::shared_ptr<GLMesh>(new GLMesh(boardMeshData));
    boardMesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    renderer->addMesh(boardMesh);

    std::shared_ptr<GLMeshData> cameraConeMeshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    cameraConeMeshData->type = GLMeshData::GLMeshDataType::Triangles;
    std::shared_ptr<GLMesh> cameraConeMesh = std::shared_ptr<GLMesh>(new GLMesh(cameraConeMeshData));
    cameraConeMesh->renderMode = GLMesh::GLMeshRenderMode::Wireframe;
    cameraConeMesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    renderer->addMesh(cameraConeMesh);

    cv::Point3f minBoard = getMinBoard(board);
    cv::Point3f maxBoard = getMaxBoard(board);

    for(int i = 0; i < board->chessboardCorners.size(); i++)
    {
        boardMeshData->addVertex(board->chessboardCorners[i], cv::Point3f(1,0,0));
    }
    renderer->render();
    float maxHeight = std::max(maxBoard.x-minBoard.x, maxBoard.y-minBoard.y)/2;
    cv::Point3f bbox[8];
    for(int k = 0; k < 8; k++)
        bbox[k] = cv::Point3f((k%2)>=1?maxBoard.x:minBoard.y,(k%4)>=2?maxBoard.y:minBoard.y,k>=4?maxHeight:0);

    cv::Mat worldToBoard = boardToWorld.inv();

    std::vector<std::vector<std::vector<cv::Point> > > listContours(listFrame.size());
    std::vector<cv::Mat> listMask(listFrame.size());

    std::vector<cv::Mat> toBoard(listFrame.size());
    for(int i = 0; i < listFrame.size(); i++)
        toBoard[i] = worldToBoard*firstFramePose[i]*poseRGB[i].inv();

    int currentFrame = 0;
    while(true)
    {
        cv::Mat toView = toBoard[currentFrame].inv();
        RgbdFrame& frame = listFrame[currentFrame];
        cv::Point3f bboxUV[8];
        for(int k = 0; k < 8; k++)
        {
            cv::Point3f p = multMatVec(toView, bbox[k]);
            p /= p.z;
            bboxUV[k] = multMatVec(KRGB[currentFrame], p);
        }

        cv::Mat maskRGB = 255*cv::Mat::ones(frame.img.size(), CV_8UC1);
        int idx[6][4] = {{0,1,3,2}, {4,5,7,6}, {0,1,5,4}, {1,3,7,5}, {2,3,7,6}, {0,2,6,4}};

        if(!listMask[currentFrame].empty())
            maskRGB = listMask[currentFrame].clone();
        std::vector<std::vector<cv::Point3f> > listProjContour;
        renderer->pauseRenderer();
        cameraConeMeshData->clear();
        std::vector<cv::Point3f> colors;
        colors.push_back(cv::Point3f(1,0,0));
        colors.push_back(cv::Point3f(0,1,0));
        colors.push_back(cv::Point3f(0,0,1));
        colors.push_back(cv::Point3f(1,1,0));
        colors.push_back(cv::Point3f(0,1,1));
        colors.push_back(cv::Point3f(1,0,1));
        colors.push_back(cv::Point3f(1,1,1));
        for(int i = 0; i < listContours.size(); i++)
        {
            if(/*i < 3 &&*/ /*i != currentFrame && */listContours[i].size() > 0)
            {
                char name[255];
                sprintf(name, "mask%d", i);
                cv::Mat maskRGB2 = cv::Mat::zeros(frame.img.size(), CV_8UC1);
                for(int j = 0; j < listContours[i].size(); j++)
                {
                    std::vector<cv::Point3f> projContour(listContours[i][j].size());
                    cv::Point3f orig = multMatVec(toBoard[i], cv::Point3f(0,0,0));
                    for(int k = 0; k < listContours[i][j].size(); k++)
                    {
                        cv::Point3f p(listContours[i][j][k].x, listContours[i][j][k].y, 1);
                        p = multMatVec(KRGB[i].inv(), p);
                        p *= 10;
                        p = multMatVec(toBoard[i], p);
                        projContour[k] = p;
                    }

                    cv::Point projContour2d[projContour.size()];
                    for(int k = 0; k < projContour.size(); k++)
                    {
                        int k2 = (k+1)%projContour.size();
                        cv::Point3f pts3d[3];
                        pts3d[0] = orig;
                        pts3d[1] = projContour[k];
                        pts3d[2] = projContour[k2];

                        cameraConeMeshData->addTriangle(pts3d[0], pts3d[1], pts3d[2], colors[i%colors.size()]);

                        std::vector<cv::Point3f> pts3dViewspace(3);
                        for(int l = 0; l < 3; l++)
                            pts3dViewspace[l] = multMatVec(toView, pts3d[l]);

                        std::vector<cv::Point3f> pts3dViewspaceZCorrected = cutTriangleByZPlane(pts3dViewspace, 0.01);

                        std::vector<cv::Point> pts;
                        printf("%s :", name);
                        for(int l = 0; l < pts3dViewspaceZCorrected.size(); l++)
                        {
                            cv::Point3f p = pts3dViewspaceZCorrected[l];
                            p /= p.z;
                            p = multMatVec(KRGB[currentFrame], p);
                            pts.push_back(cv::Point(p.x, p.y));
                            printf("(%d,%d)\n", pts[l].x, pts[l].y);
                        }
                        projContour2d[k] = pts[1];
                        fillConvexPoly(maskRGB2, &pts[0], pts.size(), cv::Scalar(255));
                    }
                    const cv::Point* tmp = &projContour2d[0];
                    int np = projContour.size();
                    fillPoly(maskRGB2, &tmp, &np, 1, cv::Scalar(255));
                    listProjContour.push_back(projContour);
                }
                maskRGB = multiplyMask(maskRGB, maskRGB2);
            }
        }
        if(listContours[currentFrame].size() != 0)
        {
            maskRGB = cv::Mat::zeros(frame.img.size(), CV_8UC1);
            for(int i = 0; i < listContours[currentFrame].size(); i++)
                cv::drawContours(maskRGB, listContours[currentFrame], i, cv::Scalar(255), CV_FILLED);
        }
        renderer->render();

        cv::Mat maskedImg = cv::Mat::zeros(frame.img.size(), CV_8UC3);
        frame.img.copyTo(maskedImg, maskRGB);
        cv::imshow("maskedImg", maskedImg);

        VisualHullMouseHandler mouseHandler(frame.img, maskRGB);
        ToolbarWinCV<VisualHullMouseHandler> toolbarWin("maskedImg", &mouseHandler);
        toolbarWin.setWinCallback([](VisualHullMouseHandler *handler,int event, int x, int y, int flags){handler->onMouse(event, x, y, flags);});
        toolbarWin.addButton(cv::imread("icons/selectionRect.png"), [](VisualHullMouseHandler *handler){handler->setSelectionRect();});
        toolbarWin.addButton(cv::imread("icons/selectionPoly.png"), [](VisualHullMouseHandler *handler){handler->setSelectionPoly();});
        toolbarWin.addButton(cv::imread("icons/superpixels.png"), [](VisualHullMouseHandler *handler){handler->setSelectionSuperpixels();});
        toolbarWin.addButton(cv::imread("icons/check.png"), [](VisualHullMouseHandler *handler){handler->setValid();});

        while(true)
        {
            if(mouseHandler.valid)
                return listMask;
            maskedImg = cv::Mat::zeros(frame.img.size(), CV_8UC3);
            frame.img.copyTo(maskedImg, maskRGB);
            if(mouseHandler.selectionMode == VisualHullMouseHandler::SelectionMode::selectionRect && mouseHandler.startSelect.x >= 0 && mouseHandler.startSelect.y >= 0)
                cv::rectangle(maskedImg, cv::Rect(mouseHandler.startSelect.x, mouseHandler.startSelect.y, mouseHandler.endSelect.x-mouseHandler.startSelect.x, mouseHandler.endSelect.y-mouseHandler.startSelect.y), cv::Scalar(0,255,0), 2);
            else if(mouseHandler.selectionMode == VisualHullMouseHandler::SelectionMode::selectionPoly)
            {
                for(int i = 0; i < mouseHandler.polySelect.size(); i++)
                {
                    cv::Point2f p1 = mouseHandler.polySelect[i];
                    cv::Point2f p2 = mouseHandler.polySelect[(i+1)%mouseHandler.polySelect.size()];
                    cv::line(maskedImg, p1, p2, cv::Scalar(255,0,0));
                }
            }
            else if(mouseHandler.selectionMode == VisualHullMouseHandler::SelectionMode::selectionSuperpixels)
            {
                cv::Mat mask;
                mouseHandler.seeds->getLabelContourMask(mask, false);
                maskedImg.setTo(cv::Scalar(0,0,255), multiplyMask(maskRGB, mask));
            }

            {
                std::vector<cv::Point> contour;
                for(int j = 0; j < 4; j++)
                {
                    cv::Point3f p = multMatVec(toView, bbox[j]);
                    p /= p.z;
                    p = multMatVec(KRGB[currentFrame], p);
                    contour.push_back(cv::Point(p.x, p.y));
                }

                std::vector<std::vector<cv::Point> > contours;
                contours.push_back(contour);
                cv::drawContours(maskedImg, contours, 0, cv::Scalar(0,0,255));

                for(int l = 0; l < board->chessboardCorners.size(); l++)
                {
                    cv::Point3f p = multMatVec(toView, board->chessboardCorners[l]);
                    p /= p.z;
                    p = multMatVec(KRGB[currentFrame], p);
                    cv::circle(maskedImg, cv::Point(p.x, p.y), 4, cv::Scalar(0,0,255), 2);
                }
            }

            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(maskRGB, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            for(int i = 0; i < contours.size(); i++)
                cv::drawContours(maskedImg, contours, i, cv::Scalar(255,0,0));
            listContours[currentFrame] = contours;
            listMask[currentFrame] = maskRGB.clone();
            toolbarWin.draw(maskedImg);
            int key = cv::waitKey(100);
            if(key != 255)
            {
                if(key == 'n')
                {
                    currentFrame = (currentFrame+1)%listFrame.size();
                    break;
                }
                else if(key == 'p')
                {
                    currentFrame = (currentFrame+listFrame.size()-1)%listFrame.size();
                    break;
                }
                else mouseHandler.onKeyboard(key);

                std::vector<std::vector<cv::Point> > contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(maskRGB, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                for(int i = 0; i < contours.size(); i++)
                    cv::drawContours(maskedImg, contours, i, cv::Scalar(255,0,0));
                listContours[currentFrame] = contours;
                listMask[currentFrame] = maskRGB.clone();

                break;
            }
        }
    }
    return std::vector<cv::Mat>();
}
