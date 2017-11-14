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

#include "tnp_yolo_manager.h"

using namespace std;

/*
 * Converts ros images to openCV images, copyies the images
 * type as sensor_msgs::image_encodings::BGR8|TYPE_16UC1.
 */
cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &img, const std::string type)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, type);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("Image converter, cv_bridge exception: %s", e.what());
  }
  return cv_ptr;
}

YoloManager::YoloManager(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
  sub_detections_ = nh_.subscribe("/out_detection", 1, &YoloManager::detectionsCallback, this); // TODO: Rename topic.
  sub_target_item_ = nh_.subscribe("/tnp_task_manager/item_to_be_picked", 1, &YoloManager::targetItemCallback, this);
  sub_camera_info_ = nh_.subscribe("/camera/depth_registered/sw_registered/camera_info", 1, &YoloManager::cameraInfoCallback, this);
  sub_color_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &YoloManager::colorCallback, this);
  sub_depth_ = it_.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1, &YoloManager::depthCallback, this);

  srv_classify_with_dl = nh_.advertiseService("/tnp_deep_vision/recognize_items", &YoloManager::recognize_items, this);

  pub_color_ = it_.advertise("/tnp_deep_vision/color_img", 1);
  pub_depth_ = it_.advertise("/tnp_deep_vision/depth_img", 1);
}

void YoloManager::allThere()
{
  if (yolo_det_ishere_ and color_img_ishere_ and depth_img_ishere_ and cam_info_ishere_) {
    yolo_det_ishere_ = false;
    color_img_ishere_ = false;
    depth_img_ishere_ = false;
    cam_info_ishere_ = false;

    float center_x, center_y;
    float box_left, box_top, box_right, box_bottom;
    float box_width, box_height;
    float grasp_x, grasp_y, grasp_theta, grasp_width;
    int roi_left, roi_top, roi_right, roi_bottom;
    int roi_width, roi_height;
    cv::Scalar roi_sum;
    int roi_nonzero;

    char txt[255];
    int found_nan;
    string txt_str;

    float color_to_depth_cols = (float)depth_img_.cols / (float)color_img_.cols;
    float color_to_depth_rows = (float)depth_img_.rows / (float)color_img_.rows;

    // Get min/max value in depth image (ignore zero value).
    cv::Mat depth_8U, depth_nonzero_mask;
    depth_img_.convertTo(depth_8U, CV_8UC1);
    cv::threshold(depth_8U, depth_nonzero_mask, 1, 255, cv::THRESH_BINARY);
    double min, max;
    cv::minMaxLoc(depth_img_, &min, &max, NULL, NULL, depth_nonzero_mask);
    // Draw min/max in depth image.
    cv::Mat depth_show;
    depth_img_.convertTo(depth_show, CV_8UC1, 255 / max);
    cv::cvtColor(depth_show, depth_show, CV_GRAY2BGR);
    /*stringstream ss;
    ss << "Min: " << min * DEPTH_SCALE << "m, Max: " << max * DEPTH_SCALE << "m";*/

    vector<geometry_msgs::PoseStamped> poses_tmp;

    ROS_INFO_STREAM(yolo_det_.detections.size() << " item(s) detected");
    if (yolo_det_.detections.size() != 0) {
      for (size_t i = 0; i < yolo_det_.detections.size(); i++) {
        // Define dynamic colors for the bounding boxes.
        cv::Scalar box_color = cv::Scalar(0, 255 * yolo_det_.detections[i].confidence, 255 - 255 * yolo_det_.detections[i].confidence);
        if (yolo_det_.detections[i].class_name == target_item_) {
          box_color = cv::Scalar(255, 0, 0);
        }

        // Define dynamic colors for the grasp points.
        cv::Scalar grasp_color = cv::Scalar(255 * yolo_det_.detections[i].confidence, 255 * yolo_det_.detections[i].confidence, 255 * yolo_det_.detections[i].confidence);
        if (yolo_det_.detections[i].class_name == target_item_) {
          grasp_color = cv::Scalar(255, 0, 0);
        }

        string ss_id = yolo_det_.detections[i].class_name;

        // Get bounding boxes (OpenCV convention: y-axis positive down).
        center_x = yolo_det_.detections[i].x * (float)color_img_.cols;
        center_y = yolo_det_.detections[i].y * (float)color_img_.rows;

        box_width = yolo_det_.detections[i].width * (float)color_img_.cols;
        box_height = yolo_det_.detections[i].height * (float)color_img_.rows;

        box_left = center_x - box_width / 2;
        box_top = center_y - box_height / 2;
        box_right = center_x + box_width / 2;
        box_bottom = center_y + box_height / 2;
        if (box_left < 0) {
          box_left = 0;
        }
        if (box_top < 0) {
          box_top = 0;
        }
        if (box_right > color_img_.cols) {
          box_right = color_img_.cols;
        }
        if (box_bottom > color_img_.rows) {
          box_bottom = color_img_.rows;
        }

        poses_tmp.push_back(calculatePose(yolo_det_.detections[i], 0));

        // Get grasp information (OpenCV convention: y-axis positive down).
        grasp_x = yolo_det_.detections[i].x_grasp;
        grasp_y = yolo_det_.detections[i].y_grasp;
        grasp_theta = yolo_det_.detections[i].theta_grasp;
        grasp_width = yolo_det_.detections[i].width_grasp;

        cv::Point grasp_line_pt1((grasp_x + grasp_width * cos(grasp_theta) / 2) * (float)color_img_.cols, (grasp_y + grasp_width * sin(grasp_theta) / 2) * (float)color_img_.rows);
        cv::Point grasp_line_pt2((grasp_x - grasp_width * cos(grasp_theta) / 2) * (float)color_img_.cols, (grasp_y - grasp_width * sin(grasp_theta) / 2) * (float)color_img_.rows);

        grasp_x = grasp_x * (float)color_img_.cols;
        grasp_y = grasp_y * (float)color_img_.rows;

        // Draw bounding box in color_image.
        cv::rectangle(color_img_, cv::Point(box_left, box_top), cv::Point(box_right, box_bottom),
                      box_color, 2);

        sprintf(txt, "%s (%d%%)", ss_id.c_str(), (int)(yolo_det_.detections[i].confidence * 100));

        // Draw grasp point/line in color image.
        cv::circle(color_img_, cv::Point(grasp_x, grasp_y), 4, grasp_color, CV_FILLED);
        cv::line(color_img_, grasp_line_pt1, grasp_line_pt2, grasp_color, 2);

        // Frame label placement in color image.
        int text_top_pos;
        box_top < TXT_HEIGHT + 5.0 ? text_top_pos = box_top + TXT_HEIGHT : text_top_pos = box_top - 5.0;

        cv::putText(color_img_, txt, cv::Point(box_left, text_top_pos), font_face, font_scale, box_color,
                    font_thick, font_ltype);

        sprintf(txt, "[%.3fm, %.3fm, %.3fm]", poses_tmp[i].pose.position.x, poses_tmp[i].pose.position.y, poses_tmp[i].pose.position.z);

        // TODO: Somehow this might not have an effect since above the label was already rendered. Merge conflict bug??
        txt_str = string(txt);
        found_nan = txt_str.find("nan");
        if (found_nan == string::npos) {
          cv::putText(color_img_, txt, cv::Point(box_left, box_bottom - 5.0), font_face, font_scale, box_color, font_thick, font_ltype);
        }

        // Draw bounding box in depth image.
        cv::rectangle(depth_show, cv::Point(box_left * color_to_depth_cols, box_top * color_to_depth_rows),
                      cv::Point(box_right * color_to_depth_cols, box_bottom * color_to_depth_rows),
                      box_color, 2);

        sprintf(txt, "%.3fm", poses_tmp[i].pose.position.z);

        // Draw grasp point/line in depth image.
        cv::circle(depth_show, cv::Point(grasp_x * color_to_depth_cols, grasp_y * color_to_depth_rows), 4, grasp_color, CV_FILLED);
        cv::line(depth_show, grasp_line_pt1, grasp_line_pt2, grasp_color, 2); //TODO: Scale to depth image size if different resolution at color camera.

        // Frame label placement in depth image.
        txt_str = string(txt);
        found_nan = txt_str.find("nan");
        if (found_nan == string::npos) {
          box_top *color_to_depth_rows < TXT_HEIGHT + 5.0 ? text_top_pos = box_top *color_to_depth_rows + TXT_HEIGHT : text_top_pos = box_top * color_to_depth_rows - 5.0;

          cv::putText(depth_show, txt, cv::Point(box_left * color_to_depth_cols, text_top_pos), font_face, font_scale,
                      box_color, font_thick, font_ltype);
        }
      }

      for (size_t j = 0; j < poses_tmp.size(); ++j) {
        ROS_INFO_STREAM(
            "Item " << j << ": " << yolo_det_.detections[j].class_name.c_str() << " (" << fixed << setprecision(2) << yolo_det_.detections[j].confidence * 100 << "%)" << setprecision(3) << ", pos: [" << poses_tmp[j].pose.position.x << ", " << poses_tmp[j].pose.position.y << ", " << poses_tmp[j].pose.position.z << "]");
      }
    }

    // Display depth information.
    sprintf(txt, "Min: %.3fm, Max: %.3fm", min * DEPTH_SCALE, max * DEPTH_SCALE);
    cv::rectangle(depth_show, cv::Point(0, depth_show.rows - 30), cv::Point(depth_show.cols / 2, depth_show.rows), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(depth_show, txt, cv::Point(8, depth_show.rows - 10), font_face, font_scale, cv::Scalar(255, 255, 255), font_thick, font_ltype);

    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img_).toImageMsg();
    pub_color_.publish(msg);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_show).toImageMsg();
    pub_depth_.publish(msg);
  }
}

void YoloManager::detectionsCallback(const yolo_light::ImageDetections &msg)
{
  yolo_det_ = msg;
  yolo_det_ishere_ = true;
  allThere();
}

void YoloManager::colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  color_img_ishere_ = true;
  allThere();
}

void YoloManager::depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  depth_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  depth_img_ishere_ = true;
  allThere();
}

void YoloManager::targetItemCallback(const std_msgs::StringConstPtr &msg)
{
  target_item_ = msg->data;
  target_item_ishere_ = true;
}

void YoloManager::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  cam_info_ = msg;
  cam_info_ishere_ = true;
  allThere();
}

bool YoloManager::recognize_items(tnp_deep_vision::recognize_items::Request &req,
                                  tnp_deep_vision::recognize_items::Response &res)
{
  ROS_INFO("recognize_items was called");
  is_recognizing_ = true;

  for (int i = 0; i < yolo_det_.detections.size(); i++) {
    tnp_deep_vision::BoundingBox bbx;
    bbx.item_id = yolo_det_.detections[i].class_name;
    bbx.confidence = yolo_det_.detections[i].confidence;
    bbx.center_x = yolo_det_.detections[i].x * (float)color_img_.cols;
    bbx.center_y = yolo_det_.detections[i].y * (float)color_img_.rows;
    bbx.width = yolo_det_.detections[i].width * (float)color_img_.cols;
    bbx.height = yolo_det_.detections[i].height * (float)color_img_.rows;
    bbx.poseCenter = calculatePose(yolo_det_.detections[i], 0);
    bbx.poseGrasp = calculatePose(yolo_det_.detections[i], 1);
    bbx.graspLinePt1 = calculatePose(yolo_det_.detections[i], 2);
    bbx.graspLinePt2 = calculatePose(yolo_det_.detections[i], 3);
    res.boundingBoxes.push_back(bbx);
  }

  return true;
}

/**
* @param poseType:
*     0 - calculate pose of center of the bbx
*     1 - calculate pose of the grasp center point
*     2 - calculate pose of the first grasp point in the plane of the grasp center
*     3 - calculate pose of the second grasp point in the plane of the grasp center
*/
geometry_msgs::PoseStamped YoloManager::calculatePose(yolo_light::Detection det, int poseType)
{
  float color_to_depth_cols = (float)depth_img_.cols / (float)color_img_.cols;
  float color_to_depth_rows = (float)depth_img_.rows / (float)color_img_.rows;

  // Use image_geometry package to transform from pixel to spatial coordinates.
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info_);

  // Get point of interest: bounding box center or grasp point.
  float center_x, center_y;

  if (poseType == 0) { // calculate pose / depth of the center of the bbx
    center_x = det.x * (float)color_img_.cols;
    center_y = det.y * (float)color_img_.rows;
  }
  else if (poseType == 1 || poseType == 2 || poseType == 3) { // calculate pose / depth of the center of grasp point
    center_x = det.x_grasp * (float)color_img_.cols;
    center_y = det.y_grasp * (float)color_img_.rows;
  }
  else {
    ROS_ERROR("Incorrect poseType selected.");
    throw 42;
  }
  // Get average depth in ROI.
  int roi_left = (int)((center_x - ROI_SIZE / 2) * color_to_depth_cols);
  int roi_top = (int)((center_y - ROI_SIZE / 2) * color_to_depth_rows);
  int roi_right = (int)((center_x + ROI_SIZE / 2) * color_to_depth_cols);
  int roi_bottom = (int)((center_y + ROI_SIZE / 2) * color_to_depth_rows);
  if (roi_left < 0) {
    roi_left = 0;
  }
  if (roi_top < 0) {
    roi_top = 0;
  }
  if (roi_right > depth_img_.cols) {
    roi_right = depth_img_.cols;
  }
  if (roi_bottom > depth_img_.rows) {
    roi_bottom = depth_img_.rows;
  }
  int roi_width = roi_right - roi_left;
  int roi_height = roi_bottom - roi_top;
  cv::Mat roi = depth_img_(cv::Rect(roi_left, roi_top, roi_width, roi_height));

  // Threshold filtering dropping depth values between 0.3m and 1.5m.
  cv::Mat roi_threshed = cv::Mat(roi.rows, roi.cols, roi.type());
  for (int i = 0; i < roi.rows; i++) {
    unsigned short *src = roi.ptr<unsigned short>(i);
    unsigned short *dst = roi_threshed.ptr<unsigned short>(i);
    for (int j = 0; j < roi.cols; j++) {
      if (src[j] >= THRES_MIN_DEPTH && src[j] <= THRES_MAX_DEPTH)
        dst[j] = src[j];
      else {
        dst[j] = 0;
      }
    }
  }

  cv::Scalar roi_sum = cv::sum(roi_threshed);
  int roi_nonzero = cv::countNonZero(roi_threshed);

  geometry_msgs::PoseStamped pose_tmp;

  if (roi_nonzero != 0) {
    cv::Scalar mean = roi_sum / roi_nonzero * DEPTH_SCALE;

    float box_width = det.width * (float)color_img_.cols;
    float box_height = det.height * (float)color_img_.rows;

    // Convert pixel to 3d coordinates.
    cv::Point3d point3d;
    if (poseType == 0 || poseType == 1) { // calculate pose / depth of the center of the bbx or grasp point
      point3d = cam_model.projectPixelTo3dRay(cv::Point2d(center_x, center_y)) * mean[0];
    }
    else if (poseType == 2) { // calculate pose of the first grasp line extremity using depth of the grasp point plane
      cv::Point grasp_line_pt((det.x_grasp + det.width_grasp * cos(det.theta_grasp) / 2) * (float)color_img_.cols,
                              (det.y_grasp + det.width_grasp * sin(det.theta_grasp) / 2) * (float)color_img_.rows);
      point3d = cam_model.projectPixelTo3dRay(grasp_line_pt) * mean[0];
    }
    else if (poseType == 3) { // calculate pose of the second grasp line extremity using depth of the grasp point plane
      cv::Point grasp_line_pt((det.x_grasp - det.width_grasp * cos(det.theta_grasp) / 2) * (float)color_img_.cols,
                              (det.y_grasp - det.width_grasp * sin(det.theta_grasp) / 2) * (float)color_img_.rows);
      point3d = cam_model.projectPixelTo3dRay(grasp_line_pt) * mean[0];
    }

    pose_tmp.header.stamp = ros::Time::now();
    pose_tmp.header.frame_id = str_frame_id;
    pose_tmp.pose.position.x = point3d.x;
    pose_tmp.pose.position.y = point3d.y;
    pose_tmp.pose.position.z = point3d.z;
    pose_tmp.pose.orientation.w = 1.0;
  }
  return pose_tmp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_yolo_manager");
  ros::NodeHandle nh;

  YoloManager ym = YoloManager(nh);

  ros::spin();

  return 0;
}
