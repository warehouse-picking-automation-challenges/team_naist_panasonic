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

#include "RGBDFrame.h"
#include "UtilCvPclRs.h"
#include <pcl/features/integral_image_normal.h>

#ifdef USE_CERES_FOR_DISTORTIONS
#include "CeresModels.h"
#endif

RgbdFrame::RgbdFrame()
{
    rs_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB> );
    imgPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    rs_cloud_normal_ptr = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    imgNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    useDepth = false;
    useIR = false;
    useIR2 = false;
}

cv::Point2f RgbdFrame::colorCameraProject(cv::Point3f p)
{
    return rs2cv(color_intrin.project(cv2rs(p)));
}

cv::Point2f RgbdFrame::depthCameraProject(cv::Point3f p)
{
    return rs2cv(depth_intrin.project(cv2rs(p)));
}

cv::Point3f RgbdFrame::depthCameraToColorCamera(cv::Point3f p)
{
    return rs2cv(depth_to_color.transform(cv2rs(p)));
}

cv::Point3f RgbdFrame::depthCameraDeproject(cv::Point2f p, float distance_in_meter)
{
    return rs2cv(depth_intrin.deproject(cv2rs(p), distance_in_meter));
}

cv::Point2f RgbdFrame::undistortDepth(cv::Point2f p)
{
    float x = p.x;
    float y = p.y;
    if((*(rs_intrinsics*)&depth_intrin).model == RS_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + depth_intrin.coeffs[0]*r2 + depth_intrin.coeffs[1]*r2*r2 + depth_intrin.coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*depth_intrin.coeffs[2]*x*y + depth_intrin.coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*depth_intrin.coeffs[3]*x*y + depth_intrin.coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    return cv::Point2f(x, y);
}

cv::Point3f RgbdFrame::undistortDepth(cv::Point3f p)
{
    cv::Point2f p2 = undistortDepth(cv::Point2f(p.x/p.z, p.y/p.z));
    return cv::Point3f(p2.x*p.z, p2.y*p.z, p.z);
}

cv::Mat RgbdFrame::getKMat(const rs::intrinsics intrin)
{
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    K.at<double>(0,0) = intrin.fx;
    K.at<double>(1,1) = intrin.fy;
    K.at<double>(0,2) = intrin.ppx;
    K.at<double>(1,2) = intrin.ppy;
    return K;
}

void RgbdFrame::extractDataFromKMat(cv::Mat K, rs::intrinsics& intrin)
{
    intrin.fx = K.at<double>(0,0);
    intrin.fy = K.at<double>(1,1);
    intrin.ppx = K.at<double>(0,2);
    intrin.ppy = K.at<double>(1,2);
}

cv::Mat RgbdFrame::getDistMat(const rs::intrinsics intrin)
{
    cv::Mat dist = cv::Mat(1,5,CV_64F);
    for(int i = 0; i < dist.cols; i++)
        dist.at<double>(0,i) = intrin.coeffs[i];
    return dist;
}

void RgbdFrame::extractDataFromDistMat(cv::Mat dist, rs::intrinsics& intrin)
{
    for(int i = 0; i < dist.cols && i < 5; i++)
        intrin.coeffs[i] = dist.at<double>(0,i);
    for(int i = dist.cols; i < 5; i++)
        intrin.coeffs[i] = 0;
}

cv::Mat RgbdFrame::getRtMat(const rs::extrinsics extrin)
{
    cv::Mat Rt = cv::Mat::eye(4,4,CV_64F);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            Rt.at<double>(i,j) = extrin.rotation[j*3+i];//todo : check librealsense order
    for(int i = 0; i < 3; i++)
        Rt.at<double>(i,3) = extrin.translation[i];
    return Rt;
}

void RgbdFrame::extractDataFromRtMat(cv::Mat Rt, rs::extrinsics& extrin)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            extrin.rotation[i*3+j] = Rt.at<double>(i,j);
    for(int i = 0; i < 3; i++)
        extrin.translation[i] = Rt.at<double>(i,3);
    printf("rot\n");
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            printf("%lf,%c", extrin.rotation[i*3+j], j==2?'\n':' ');
    printf("trans\n");
    for(int i = 0; i < 3; i++)
        printf("%lf, ", extrin.translation[i]);
}

void RgbdFrame::storeIntrinsic(cv::FileStorage& fs, std::string base, rs::intrinsics intrin)
{
    fs << (base+"_K").c_str() << getKMat(intrin);
    fs << (base+"_dist").c_str() << getDistMat(intrin);
    fs << (base+"_width").c_str() << intrin.width;
    fs << (base+"_height").c_str() << intrin.height;
    fs << (base+"_dist_type").c_str() << (*(rs_intrinsics*)&intrin).model;
}

void RgbdFrame::readIntrinsic(cv::FileStorage& fs, std::string base, rs::intrinsics& intrin)
{
    cv::Mat K, dist;
    printf("%s\n", (base+"_K").c_str());
    fs[(base+"_K").c_str()] >> K;//getKMat(intrin);
    fs[(base+"_dist").c_str()] >> dist;// getDistMat(intrin);
    printf("%s\n", mat2str(K).c_str());
    extractDataFromKMat(K, intrin);
    extractDataFromDistMat(dist, intrin);
    fs[(base+"_width").c_str()] >> intrin.width;
    fs[(base+"_height").c_str()] >> intrin.height;
    int type;
    fs[(base+"_dist_type").c_str()] >> type;
    (*(rs_intrinsics*)&intrin).model = (rs_distortion)type;
}

void RgbdFrame::save(std::string folder, std::string name)
{
    cv::FileStorage fs((folder+"/"+name+".yml").c_str(), cv::FileStorage::WRITE);
    storeIntrinsic(fs, "color", color_intrin);
    fs << "scale" << scale;
    fs << "serial" << camSerial;
    fs << "useDepth" << useDepth;
    fs << "useIR" << useIR;
    fs << "useIR2" << useIR2;
    cv::imwrite((folder+"/"+name+"_color.png").c_str(), img);
    if(useDepth)
    {
        storeIntrinsic(fs, "depth", depth_intrin);
        fs << "depth_to_color" << getRtMat(depth_to_color);
        cv::FileStorage fsDepth((folder+"/"+name+"_depth.yml").c_str(), cv::FileStorage::WRITE);
        fsDepth << "mat" << depth;
        fsDepth.release();
    }
    if(useIR)
    {
        storeIntrinsic(fs, "ir", ir_intrin);
        fs << "ir_to_color" << getRtMat(ir_to_color);
        cv::imwrite((folder+"/"+name+"_ir.png").c_str(), imgIR);
    }
    if(useIR2)
    {
        storeIntrinsic(fs, "ir2", ir2_intrin);
        fs << "ir2_to_color" << getRtMat(ir2_to_color);
        cv::imwrite((folder+"/"+name+"_ir2.png").c_str(), imgIR2);
    }
    fs.release();
}

bool RgbdFrame::load(std::string folder, std::string name)
{
    printf("%s\n", (folder+"/"+name+".yml").c_str());
    cv::FileStorage fs((folder+"/"+name+".yml").c_str(), cv::FileStorage::READ);
    if(!fs.isOpened())
    {
      printf("can't load %s\n", (folder+"/"+name+".yml").c_str());
      return false;
    }
    readIntrinsic(fs, "color", color_intrin);
    fs["serial"] >> camSerial;
    fs["useDepth"] >> useDepth;
    fs["useIR"] >> useIR;
    fs["useIR2"] >> useIR2;
    fs["scale"] >> scale;

    if(useDepth)
    {
        cv::Mat RtDepthToCol;
        fs["depth_to_color"] >> RtDepthToCol;
        readIntrinsic(fs, "depth", depth_intrin);
        extractDataFromRtMat(RtDepthToCol, depth_to_color);
        cv::FileStorage fsDepth((folder+"/"+name+"_depth.yml").c_str(), cv::FileStorage::READ);
        if(!fsDepth.isOpened())
        {
          printf("can't load %s\n", (folder+"/"+name+"_depth.yml").c_str());
          return false;
        }
        fsDepth["mat"] >> depth;
        fsDepth.release();
    }
    if(useIR)
    {
        cv::Mat RtIrToCol;
        readIntrinsic(fs, "ir", ir_intrin);
        fs["ir_to_color"] >> RtIrToCol;
        extractDataFromRtMat(RtIrToCol, ir_to_color);
        imgIR = cv::imread((folder+"/"+name+"_ir.png").c_str());
    }
    if(useIR2)
    {
        cv::Mat RtIr2ToCol;
        readIntrinsic(fs, "ir2", ir2_intrin);
        fs["ir2_to_color"] >> RtIr2ToCol;
        extractDataFromRtMat(RtIr2ToCol, ir2_to_color);
        imgIR2 = cv::imread((folder+"/"+name+"_ir2.png").c_str());
    }

    img = cv::imread((folder+"/"+name+"_color.png").c_str());
    cvtColor(img, img, CV_BGR2RGB);
    mask = cv::imread((folder+"/"+name+"_mask.png").c_str());
    if(!mask.empty() && mask.channels() == 3)
        cvtColor(mask, mask, CV_RGB2GRAY);

    fs.release();
    return true;
}



RgbdFrame extractDataFromRs( rs::device *dev, bool useDepth, bool useIR, bool useIR2)
{
    RgbdFrame frame;
    frame.useDepth = useDepth;
    frame.useIR = useIR;
    frame.useIR2 = useIR2;
    frame.camSerial = dev->get_serial();
    // Wait for new frame data
    if( dev->is_streaming( ) )
        dev->wait_for_frames();

    rs::stream depth_stream = rs::stream::depth;
    rs::stream color_stream = rs::stream::rectified_color;
    // Retrieve camera parameters for mapping between depth and color
    if(useDepth)
    {
        frame.depth_intrin     = dev->get_stream_intrinsics( depth_stream );//todo : check more options with alignment
        frame.depth_to_color   = dev->get_extrinsics( depth_stream, color_stream );
    }
    frame.color_intrin     = dev->get_stream_intrinsics( color_stream );
    frame.scale            = dev->get_depth_scale( );

    if(useIR)
    {
        frame.ir_intrin         = dev->get_stream_intrinsics( rs::stream::infrared);
        frame.ir_to_color   = dev->get_extrinsics( rs::stream::infrared, color_stream );
        frame.color_to_ir   = dev->get_extrinsics( color_stream, rs::stream::infrared );
    }
    if(useIR2)
    {
        frame.ir2_intrin         = dev->get_stream_intrinsics( rs::stream::infrared2);
        frame.ir2_to_color   = dev->get_extrinsics( rs::stream::infrared2, color_stream );
    }
    // Retrieve our images
    if(useDepth)
        frame.depth = cv::Mat(frame.depth_intrin.height, frame.depth_intrin.width, CV_16UC1, (void*)dev->get_frame_data(depth_stream), cv::Mat::AUTO_STEP).clone();
    frame.img   = cv::Mat(frame.color_intrin.height, frame.color_intrin.width, CV_8UC3, (void*)dev->get_frame_data(color_stream), cv::Mat::AUTO_STEP).clone();

    if(useIR)
        frame.imgIR = cv::Mat(frame.ir_intrin.height, frame.ir_intrin.width, CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared), cv::Mat::AUTO_STEP).clone();
    if(useIR2)
        frame.imgIR2   = cv::Mat(frame.ir2_intrin.height, frame.ir2_intrin.width, CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared2), cv::Mat::AUTO_STEP).clone();

    cvtColor(frame.img, frame.img, CV_BGR2RGB);

    return frame;
}

void generatePointCloudOrganized2(RgbdFrame& frame, bool computeDepthAligned, bool computeNormal, bool computeNormalAligned)
{
    int t0 = clock();
    frame.imgDepth = cv::Mat(frame.img.rows, frame.img.cols, CV_32F);
    frame.imgDepth.setTo(cv::Scalar(0.0));
    frame.imgPoints->clear();
    frame.imgPoints->resize(frame.img.rows * frame.img.cols);
    frame.imgPoints->is_dense = false;
    frame.imgPoints->width = frame.img.cols;
    frame.imgPoints->height = frame.img.rows;
    int t1 = clock();
    for(int i = 0; i < frame.imgPoints->height; i++)
        for(int j = 0; j < frame.imgPoints->width; j++)
        {
            int id = i*frame.imgPoints->width+j;
            frame.imgPoints->points[ id ].x = 0.0;
            frame.imgPoints->points[ id ].y = 0.0;
            frame.imgPoints->points[ id ].z = 0.0;
        }
    int t2 = clock();

    // Depth dimension helpers
    int dw  = 0;
    int dh  = 0;
    int dwh = 0;

    dw = frame.depth.cols;
    dh = frame.depth.rows;

    dwh = dw * dh;

    // Set the cloud up to be used
    frame.rs_cloud_ptr->clear( );
    frame.rs_cloud_ptr->resize( dwh );
    frame.rs_cloud_ptr->is_dense = false;
    frame.rs_cloud_ptr->width = dw;
    frame.rs_cloud_ptr->height = dh;

    static const float nan = std::numeric_limits<float>::quiet_NaN( );
    // Iterate the data space
    // First, iterate across columns
    int t3 = clock();
    for( int dy = 0; dy < dh; dy++ )
    {
        // Second, iterate across rows
        for( int dx = 0; dx < dw; dx++ )
        {
            uint i = dy * dw + dx;
            uint16_t depth_value = frame.depth.at<uint16_t>(dy, dx);

            if( depth_value == 0 )
            {
                frame.rs_cloud_ptr->points[ i ].x = (float) nan;
                frame.rs_cloud_ptr->points[ i ].y = (float) nan;
                frame.rs_cloud_ptr->points[ i ].z = (float) nan;
                continue;
            }

            cv::Point2f depth_pixel((float)dx, (float)dy);
            float depth_in_meters = depth_value * frame.scale;

            cv::Point3f depth_point = frame.depthCameraDeproject(depth_pixel, depth_in_meters);
            cv::Point3f color_point = frame.depthCameraToColorCamera(depth_point);
            cv::Point2f color_pixel = frame.colorCameraProject(color_point);

            const int cx = ( int )std::round( color_pixel.x );
            const int cy = ( int )std::round( color_pixel.y );

            // Set up logic to remove bad points
            bool depth_fail = true;
            bool color_fail = true;

            depth_fail = ( depth_point.z > NOISY );
            color_fail = ( cx < 0 || cy < 0 || cx > frame.img.cols || cy > frame.img.rows );

            // ==== Cloud Input Pointers ====

            // XYZ input access to cloud
            float *dp_x;
            float *dp_y;
            float *dp_z;

            dp_x = &( frame.rs_cloud_ptr->points[ i ].x );
            dp_y = &( frame.rs_cloud_ptr->points[ i ].y );
            dp_z = &( frame.rs_cloud_ptr->points[ i ].z );

            // RGB input access to cloud
            uint8_t *cp_r;
            uint8_t *cp_g;
            uint8_t *cp_b;

            cp_r = &( frame.rs_cloud_ptr->points[ i ].r );
            cp_g = &( frame.rs_cloud_ptr->points[ i ].g );
            cp_b = &( frame.rs_cloud_ptr->points[ i ].b );

            if( depth_fail || color_fail )
            {
                *dp_x = *dp_y = *dp_z = (float) nan;
                *cp_r = *cp_g = *cp_b = 255;
                continue;
            }

            // ==== Cloud Input Data ====
            // Set up depth point data
            float real_x        = 0;
            float real_y        = 0;
            float real_z        = 0;
            float adjusted_x    = 0;
            float adjusted_y    = 0;
            float adjusted_z    = 0;

            real_x = depth_point.x;
            real_y = depth_point.y;
            real_z = depth_point.z;

            // Adjust point to coordinates
            adjusted_x = /*-1 * */real_x;
            adjusted_y = /*-1 * */real_y;
            adjusted_z = real_z;

            // Set up color point data
            uint8_t *offset = frame.img.ptr<uint8_t>(cy) + cx * 3 ;

            uint8_t raw_r       = 0;
            uint8_t raw_g       = 0;
            uint8_t raw_b       = 0;
            uint8_t adjusted_r  = 0;
            uint8_t adjusted_g  = 0;
            uint8_t adjusted_b  = 0;

            raw_r = raw_r = *( offset );
            raw_g = raw_g = *( offset + 1 );
            raw_b = raw_b = *( offset + 2 );

            // Adjust color arbitrarily
            adjusted_r = raw_r;
            adjusted_g = raw_g;
            adjusted_b = raw_b;

            // ==== Cloud Point Evaluation ====
            // If bad point, remove & skip

            // If valid point, add data to cloud
            {
                // Fill in cloud depth
                *dp_x = adjusted_x;
                *dp_y = adjusted_y;
                *dp_z = adjusted_z;

                // Fill in cloud color
                *cp_r = adjusted_r;
                *cp_g = adjusted_g;
                *cp_b = adjusted_b;

                //frame.imgDepth.at<float>(cy,cx) = sqrt(adjusted_x*adjusted_x+adjusted_y*adjusted_y+adjusted_z*adjusted_z);
            }
        }
    }
    int t4 = clock();

    if(computeDepthAligned)
    {
        const int dx[4] = {-1,-1,0,0}, dy[4] = {-1,0,0,-1};
        std::vector<cv::Point3f> listColorPoints(frame.depth.cols);
        std::vector<cv::Point2f> listColorPixels(frame.depth.cols);
        std::vector<bool> listColorValid(frame.depth.cols);

        //first line
        for(int j = 0; j < frame.depth.cols; j++)
        {
            pcl::PointXYZRGB p = frame.rs_cloud_ptr->points[j];
            if(p.x == nan)
                listColorValid[j] = false;
            else
            {
                listColorPoints[j] = frame.depthCameraToColorCamera(pcl2cv(p));
                cv::Point2f uv = frame.colorCameraProject(listColorPoints[j]);
                if(uv.x < 0 || uv.y < 0 || uv.x > frame.img.cols || uv.y > frame.img.rows)
                    listColorValid[j] = false;
                else
                {
                    listColorPixels[j] = uv;
                    listColorValid[j] = true;
                }
            }
        }
        for(int i = 1; i < frame.depth.rows; i++)
        {
            cv::Point3f color_point[4];
            cv::Point2f color_pixel[4];
            bool listValid[4];

            //first col
            pcl::PointXYZRGB p = frame.rs_cloud_ptr->points[i*dw];
            if(p.x == nan)
                listValid[2] = false;
            else
            {
                color_point[2] = frame.depthCameraToColorCamera(pcl2cv(p));
                cv::Point2f uv = frame.colorCameraProject(color_point[2]);
                if(uv.x < 0 || uv.y < 0 || uv.x > frame.img.cols || uv.y > frame.img.rows)
                    listValid[2] = false;
                else
                {
                    color_pixel[2] = uv;
                    listValid[2] = true;
                }
            }

            for(int j = 1; j < frame.depth.cols; j++)
            {
                color_point[0] = listColorPoints[j-1];
                color_point[1] = color_point[2];
                color_point[3] = listColorPoints[j];

                color_pixel[0] = listColorPixels[j-1];
                color_pixel[1] = color_pixel[2];
                color_pixel[3] = listColorPixels[j];

                listValid[0] = listColorValid[j-1];
                listValid[1] = listValid[2];
                listValid[3] = listColorValid[j];

                listColorPoints[j-1] = color_point[1];
                listColorPixels[j-1] = color_pixel[1];
                listColorValid[j-1] = listValid[1];



                pcl::PointXYZRGB p = frame.rs_cloud_ptr->points[(i+dy[2]) * dw + j+dx[2]];
                if(p.x == nan)
                    listValid[2] = false;
                else
                {
                    color_point[2] = frame.depthCameraToColorCamera(pcl2cv(p));
                    cv::Point2f uv = frame.colorCameraProject(color_point[2]);
                    if(uv.x < 0 || uv.y < 0 || uv.x > frame.img.cols || uv.y > frame.img.rows)
                        listValid[2] = false;
                    else
                    {
                        color_pixel[2] = uv;
                        listValid[2] = true;
                    }
                }

                if(!listValid[0] || !listValid[1] || !listValid[2] || !listValid[3])
                    continue;

                for(int k = 0; k < 2; k++)
                {
                    cv::Point3f color_pointTri[3] = {color_point[0], color_point[2], (k==0?color_point[1]:color_point[3])};
                    cv::Point2f color_pixelTri[3] = {color_pixel[0], color_pixel[2], (k==0?color_pixel[1]:color_pixel[3])};
                    cv::Point2f pixMin(std::min(color_pixelTri[0].x, std::min(color_pixelTri[1].x, color_pixelTri[2].x)), std::min(color_pixelTri[0].y, std::min(color_pixelTri[1].y, color_pixelTri[2].y)));
                    cv::Point2f pixMax(std::max(color_pixelTri[0].x, std::max(color_pixelTri[1].x, color_pixelTri[2].x)), std::max(color_pixelTri[0].y, std::max(color_pixelTri[1].y, color_pixelTri[2].y)));

                    cv::Point2f v0, v1;
                    float d00, d01, d11, invDenom;
                    BarycentricPrecompute(color_pixelTri[0], color_pixelTri[1], color_pixelTri[2], v0, v1, d00, d01, d11, invDenom);

                    int minY = std::max(0, (int)floor(pixMin.y));
                    int maxY = std::min(frame.img.rows, (int)ceil(pixMax.y));
                    int minX = std::max(0, (int)floor(pixMin.x));
                    int maxX = std::min(frame.img.cols, (int)ceil(pixMax.x));
                    for(int y = minY; y < maxY; y++)
                    {
                        float *imgDepthPtr = frame.imgDepth.ptr<float>(y);
                        pcl::PointXYZ *pointsPtr = &frame.imgPoints->points[y*frame.imgPoints->width];
                        for(int x = minX; x < maxX; x++)
                        {
                            float u,v,w;
                            BarycentricWithPrecomputedVal(cv::Point2f(x,y), color_pixelTri[0], v0, v1, d00, d01, d11, invDenom, u, v, w);
                            if(u >= 0 && u <= 1 && v >= 0 && v <= 1)
                            {
                                cv::Point3f res = u*color_pointTri[0]+v*color_pointTri[1]+w*color_pointTri[2];
                                imgDepthPtr[x] = sqrt(res.dot(res));
                                pointsPtr[x] = pcl::PointXYZ(res.x, res.y, res.z);
                            }
                        }
                    }
                }
            }
        }
    }
    int t5 = clock();

    if(computeNormal)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(frame.rs_cloud_ptr);
        ne.compute(*(frame.rs_cloud_normal_ptr));
    }

    if(computeNormalAligned)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(frame.imgPoints);
        ne.compute(*(frame.imgNormals));
    }

    int t6 = clock();
    printf("generatePointCloudOrganized2 time : %d %d %d %d %d %d\n", t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5);
}

RgbdFrame generatePointCloudOrganized2( rs::device *dev)
{
    RgbdFrame frame = extractDataFromRs(dev);
    generatePointCloudOrganized2(frame);
    return frame;
}

/**
 * Creates a complete RGBD frame from one camera
 */
RgbdFrame RgbdFrame::extractDataFromRos(cv::Mat colorK, cv::Mat colorD, cv::Mat colorImg, cv::Mat depthK, cv::Mat depthD, cv::Mat depthImg,
                                                cv::Mat Extrinsic)
{
  RgbdFrame frame;

  if(depthK.empty() || depthImg.empty())
    frame.useDepth = false;
  else frame.useDepth = true;

  frame.useIR = false;
  frame.useIR2 = false;

  if(frame.useDepth)
  {
    frame.depth_intrin = rs::intrinsics();
    frame.depth_intrin.width = depthImg.cols;
    frame.depth_intrin.height = depthImg.rows;
    frame.depth_intrin.ppx = depthK.at<double>(0, 2);
    frame.depth_intrin.ppy = depthK.at<double>(1, 2);
    frame.depth_intrin.fx = depthK.at<double>(0, 0);
    frame.depth_intrin.fy = depthK.at<double>(1, 1);
    ((rs_intrinsics *)&frame.depth_intrin)->model = (rs_distortion)rs::distortion::inverse_brown_conrady;
    for (int i = 0; i < 5; i++)
      frame.depth_intrin.coeffs[i] = depthD.at<double>(i); // Approximated to 0 for now.
  }

  frame.color_intrin = rs::intrinsics();
  frame.color_intrin.width = colorImg.cols;
  frame.color_intrin.height = colorImg.rows;
  frame.color_intrin.ppx = colorK.at<double>(0, 2);
  frame.color_intrin.ppy = colorK.at<double>(1, 2);
  frame.color_intrin.fx = colorK.at<double>(0, 0);
  frame.color_intrin.fy = colorK.at<double>(1, 1);
  ((rs_intrinsics *)&frame.color_intrin)->model = (rs_distortion)rs::distortion::modified_brown_conrady;
  for (int i = 0; i < 5; i++)
    frame.color_intrin.coeffs[i] = colorD.at<double>(i); // Approximated to 0 for now.

  if(frame.useDepth)
  {
    for (int i = 0; i < 3; i++)
    {
      frame.depth_to_color.translation[i] = Extrinsic.at<double>(i, 3);
      for (int j = 0; j < 3; j++)
        frame.depth_to_color.rotation[j * 3 + i] = Extrinsic.at<double>(i, j);
    }
    frame.depth = depthImg.clone();
  }
  frame.scale = 0.001;

  frame.img = colorImg.clone();

  return frame;
}

//stream = color:0, ir1:1, ir2:2
cv::Mat getCameraMatrix(const RgbdFrame& frame, int stream)
{
    if(stream == 0)
        return frame.getKMat(frame.color_intrin);
    else if(stream == 1)
    {
        if(frame.useDepth)
            return frame.getKMat(frame.depth_intrin);
        else return frame.getKMat(frame.ir_intrin);
    }
    else if(stream == 2)
        return frame.getKMat(frame.ir2_intrin);
    return cv::Mat();
}

void sortBySerial(std::vector<RgbdFrame>& frames)
{
    std::sort(frames.begin(), frames.end(), [](const RgbdFrame& a, const RgbdFrame& b){ return a.camSerial.compare(b.camSerial) < 0;});
}

#ifdef USE_CERES_FOR_DISTORTIONS

//stream = color:0, ir1:1, ir2:2
cv::Mat getCameraDistMatrix(const RgbdFrame& frame, cv::Size size, int stream)
{
    if(stream == 0)
        return modifiedToNormalBrownCoeffs(frame.getDistMat(frame.color_intrin), size);
    else if(stream == 1)
    {
        if(frame.useDepth)
            return inversedToNormalBrownCoeffs(frame.getDistMat(frame.depth_intrin), size);
        else return inversedToNormalBrownCoeffs(frame.getDistMat(frame.ir_intrin), size);
    }
    else if(stream == 2)
        return inversedToNormalBrownCoeffs(frame.getDistMat(frame.ir2_intrin), size);
    return cv::Mat();
}
#endif
