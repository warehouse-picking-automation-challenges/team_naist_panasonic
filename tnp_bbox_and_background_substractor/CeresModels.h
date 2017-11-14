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

#ifndef CERESMODELS_H
#define CERESMODELS_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "UtilCvPclRs.h"

struct Reprojection3DErrorMultiPlane {
  Reprojection3DErrorMultiPlane(double observed_x, double observed_y, double observed_z, double model_x, double model_y, double model_z)
      : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z), model_x(model_x), model_y(model_y), model_z(model_z) {}

  template <typename T>
  bool operator()(const T* const cameraRt,
                  const T* const objectRt,
                  const T* const planeRt,
                  T* residuals) const {

    T predicted[3];
    T point[3] = {T(model_x), T(model_y), T(model_z)};
    transform(cameraRt, objectRt, planeRt, point, predicted);
    // The error is the difference between the predicted and observed position.
    residuals[0] = T(100)*(predicted[0] - T(observed_x));
    residuals[1] = T(100)*(predicted[1] - T(observed_y));
    residuals[2] = T(100)*(predicted[2] - T(observed_z));
    return true;
  }

  template<typename T>
  static void transform(const T* const cameraRt,
                      const T* const objectRt,
                      const T* const planeRt,
                      const T* const point,
                      T* result)
  {
      // camera[0,1,2] are the angle-axis rotation.
      T p0[3], p1[3];

      ceres::AngleAxisRotatePoint(planeRt, point, p0);
      p0[0] += planeRt[3]; p0[1] += planeRt[4]; p0[2] += planeRt[5];

      ceres::AngleAxisRotatePoint(objectRt, p0, p1);
      p1[0] += objectRt[3]; p1[1] += objectRt[4]; p1[2] += objectRt[5];

      ceres::AngleAxisRotatePoint(cameraRt, p1, result);
      result[0] += cameraRt[3]; result[1] += cameraRt[4]; result[2] += cameraRt[5];
  }

  static cv::Point3f transform(const double* const cameraRt,
                     const double* const objectRt,
                     const double* const planeRt,
                     cv::Point3f point)
  {
      double result[2];
      double p[3] = {point.x, point.y, point.z};
      transform(cameraRt, objectRt, planeRt, p, result);
      return cv::Point3f(result[0], result[1], result[2]);
  }

   // the client code.
   static ceres::CostFunction* Create(double observed_x, double observed_y, double observed_z, double model_x, double model_y, double model_z) {
     return (new ceres::AutoDiffCostFunction<Reprojection3DErrorMultiPlane, 3, 6, 6, 6>(
                 new Reprojection3DErrorMultiPlane(observed_x, observed_y, observed_z, model_x, model_y, model_z)));
   }

  double observed_x;
  double observed_y;
  double observed_z;
  double model_x;
  double model_y;
  double model_z;
};


template<int nbDistVal>
struct ReprojectionErrorMultiPlane {
  ReprojectionErrorMultiPlane(double observed_x, double observed_y, bool modifiedBrownDist)
      : observed_x(observed_x), observed_y(observed_y), modifiedBrownDist(modifiedBrownDist) {}

  template <typename T>
  bool operator()(const T* const cameraK,//fx,fy,cx,cy
                  const T* const cameraDist,//k1,k2,p1,p2,k3
                  const T* const cameraRt,
                  const T* const objectRt,
                  const T* const planeRt,
                  const T* const point,
                  T* residuals) const {

    T predicted[2];
    project(cameraK, cameraDist, cameraRt, objectRt, planeRt, point, modifiedBrownDist, predicted);
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted[0] - T(observed_x);
    residuals[1] = predicted[1] - T(observed_y);
    return true;
  }

  template<typename T>
  static void project(const T* const cameraK,//fx,fy,cx,cy
                      const T* const cameraDist,//k1,k2,p1,p2,k3
                      const T* const cameraRt,
                      const T* const objectRt,
                      const T* const planeRt,
                      const T* const point,
                      bool modifiedBrownDist,
                      T* result)
  {
      // camera[0,1,2] are the angle-axis rotation.
      T p0[3], p1[3], p2[3];

      ceres::AngleAxisRotatePoint(planeRt, point, p0);
      p0[0] += planeRt[3]; p0[1] += planeRt[4]; p0[2] += planeRt[5];

      ceres::AngleAxisRotatePoint(objectRt, p0, p1);
      p1[0] += objectRt[3]; p1[1] += objectRt[4]; p1[2] += objectRt[5];

      ceres::AngleAxisRotatePoint(cameraRt, p1, p2);
      p2[0] += cameraRt[3]; p2[1] += cameraRt[4]; p2[2] += cameraRt[5];

      p2[0] /= p2[2];
      p2[1] /= p2[2];

      T p_dist[2];
      applyDistortion<nbDistVal>(p2, cameraDist, p_dist, modifiedBrownDist);

      result[0] = cameraK[0] * p_dist[0] + cameraK[2];
      result[1] = cameraK[1] * p_dist[1] + cameraK[3];
  }

  static cv::Point2f project(const double* const cameraK,//fx,fy,cx,cy
                     const double* const cameraDist,//k1,k2,p1,p2,k3
                     const double* const cameraRt,
                     const double* const objectRt,
                     const double* const planeRt,
                     cv::Point3f point, bool modifiedBrownDist)
  {
      double result[2];
      double p[3] = {point.x, point.y, point.z};
      project(cameraK, cameraDist, cameraRt, objectRt, planeRt, p, modifiedBrownDist, result);
      return cv::Point2f(result[0], result[1]);
  }

   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y, bool modifiedBrownDist) {
     return (new ceres::AutoDiffCostFunction<ReprojectionErrorMultiPlane<nbDistVal>, 2, 4, nbDistVal, 6, 6, 6, 3>(
                 new ReprojectionErrorMultiPlane<nbDistVal>(observed_x, observed_y, modifiedBrownDist)));
   }

  double observed_x;
  double observed_y;
  bool modifiedBrownDist;
};

template<int nbDistVal>
struct ReprojectionError2 {
  ReprojectionError2(double observed_x, double observed_y, bool modifiedBrownDist)
      : observed_x(observed_x), observed_y(observed_y), modifiedBrownDist(modifiedBrownDist) {}

  template <typename T>
  bool operator()(const T* const cameraK,//fx,fy,cx,cy
                  const T* const cameraDist,//k1,k2,p1,p2,k3
                  const T* const cameraRt,
                  const T* const objectRt,
                  const T* const point,
                  T* residuals) const {

    T predicted[2];
    project(cameraK, cameraDist, cameraRt, objectRt, point, modifiedBrownDist, predicted);
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted[0] - T(observed_x);
    residuals[1] = predicted[1] - T(observed_y);
    return true;
  }

  template<typename T>
  static void project(const T* const cameraK,//fx,fy,cx,cy
                      const T* const cameraDist,//k1,k2,p1,p2,k3
                      const T* const cameraRt,
                      const T* const objectRt,
                      const T* const point,
                      bool modifiedBrownDist,
                      T* result)
  {
      // camera[0,1,2] are the angle-axis rotation.
      T p0[3], p1[3], p2[3];
      /*p0[0] = point[0]-objectRt[3]; p0[1] = point[1]-objectRt[4]; p0[2] = point[2]-objectRt[5];
      T rot[3];
      rot[0] = -objectRt[0]; rot[1] = -objectRt[1]; rot[2] = -objectRt[2];
      ceres::AngleAxisRotatePoint(rot, p0, p1);*/
      ceres::AngleAxisRotatePoint(objectRt, point, p1);
      p1[0] += objectRt[3]; p1[1] += objectRt[4]; p1[2] += objectRt[5];

      ceres::AngleAxisRotatePoint(cameraRt, p1, p2);
      p2[0] += cameraRt[3]; p2[1] += cameraRt[4]; p2[2] += cameraRt[5];

      p2[0] /= p2[2];
      p2[1] /= p2[2];

      T p_dist[2];
      applyDistortion<nbDistVal>(p2, cameraDist, p_dist, modifiedBrownDist);

      result[0] = cameraK[0] * p_dist[0] + cameraK[2];
      result[1] = cameraK[1] * p_dist[1] + cameraK[3];
  }

  static cv::Point2f project(const double* const cameraK,//fx,fy,cx,cy
                     const double* const cameraDist,//k1,k2,p1,p2,k3
                     const double* const cameraRt,
                     const double* const objectRt,
                     cv::Point3f point, bool modifiedBrownDist)
  {
      double result[2];
      double p[3] = {point.x, point.y, point.z};
      project(cameraK, cameraDist, cameraRt, objectRt, p, modifiedBrownDist, result);
      return cv::Point2f(result[0], result[1]);
  }

   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y, bool modifiedBrownDist) {
     return (new ceres::AutoDiffCostFunction<ReprojectionError2<nbDistVal>, 2, 4, nbDistVal, 6, 6, 3>(
                 new ReprojectionError2<nbDistVal>(observed_x, observed_y, modifiedBrownDist)));
   }

  double observed_x;
  double observed_y;
  bool modifiedBrownDist;
};

template<int nbDistVal>
struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y, bool modifiedBrownDist)
      : observed_x(observed_x), observed_y(observed_y), modifiedBrownDist(modifiedBrownDist) {}

  template <typename T>
  bool operator()(const T* const cameraK,//fx,fy,cx,cy
                  const T* const cameraDist,//k1,k2,p1,p2,k3
                  const T* const cameraRt,
                  const T* const point,
                  T* residuals) const {

    T predicted[2];
    project(cameraK, cameraDist, cameraRt, point, modifiedBrownDist, predicted);
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted[0] - T(observed_x);
    residuals[1] = predicted[1] - T(observed_y);
    return true;
  }

  template<typename T>
  static void project(const T* const cameraK,//fx,fy,cx,cy
                      const T* const cameraDist,//k1,k2,p1,p2,k3
                      const T* const cameraRt,
                      const T* const point,
                      bool modifiedBrownDist,
                      T* result)
  {
      // camera[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(cameraRt, point, p);
      // camera[3,4,5] are the translation.
      p[0] += cameraRt[3]; p[1] += cameraRt[4]; p[2] += cameraRt[5];

      p[0] /= p[2];
      p[1] /= p[2];

      T p_dist[2];
      applyDistortion<nbDistVal>(p, cameraDist, p_dist, modifiedBrownDist);

      result[0] = cameraK[0] * p_dist[0] + cameraK[2];
      result[1] = cameraK[1] * p_dist[1] + cameraK[3];
  }

  static cv::Point2f project(const double* const cameraK,//fx,fy,cx,cy
                     const double* const cameraDist,//k1,k2,p1,p2,k3
                     const double* const cameraRt,
                     cv::Point3f point, bool modifiedBrownDist)
  {
      double result[2];
      double p[3] = {point.x, point.y, point.z};
      project(cameraK, cameraDist, cameraRt, p, modifiedBrownDist, result);
      return cv::Point2f(result[0], result[1]);
  }

   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y, bool modifiedBrownDist) {
     return (new ceres::AutoDiffCostFunction<ReprojectionError<nbDistVal>, 2, 4, nbDistVal, 6, 3>(
                 new ReprojectionError<nbDistVal>(observed_x, observed_y, modifiedBrownDist)));
   }

  double observed_x;
  double observed_y;
  bool modifiedBrownDist;
};

template<int nbDistVal>
struct DistortionError {
  DistortionError(double x, double y, double distorted_x, double distorted_y, bool modifiedBrownDist)
      : x(x), y(y), distorted_x(distorted_x), distorted_y(distorted_y), modifiedBrownDist(modifiedBrownDist)
  {}

  template <typename T>
  bool operator()(const T* const cameraDist,//k1,k2,p1,p2,k3
                  T* residuals) const {

    T predicted[2];
    T point[2] = {T(x), T(y)};
    applyDistortion<nbDistVal>(point, cameraDist, predicted, modifiedBrownDist);
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted[0] - T(distorted_x);
    residuals[1] = predicted[1] - T(distorted_y);
    return true;
  }

  static cv::Point2f project(const double* const cameraDist,//k1,k2,p1,p2,k3
                     cv::Point2f point, bool modifiedBrownDist)
  {
      double result[2];
      double p[2] = {point.x, point.y};
      applyDistortion<nbDistVal>(p, cameraDist, result, modifiedBrownDist);
      return cv::Point2f(result[0], result[1]);
  }

   // the client code.
   static ceres::CostFunction* Create(const double x, const double y,
                                      const double distorted_x, const double distorted_y,
                                      bool modifiedBrownDist) {
     return (new ceres::AutoDiffCostFunction<DistortionError<nbDistVal>, 2, nbDistVal>(
                 new DistortionError<nbDistVal>(x, y, distorted_x, distorted_y, modifiedBrownDist)));
   }

  double x, y;
  double distorted_x, distorted_y;
  bool modifiedBrownDist;
};

struct RigidTransformationError {
  RigidTransformationError()
  {}

  template <typename T>
  bool operator()(const T* const Rt,
                  const T* const boardPoint,
                  const T* const observedPoint,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(Rt, observedPoint, p);
    // camera[3,4,5] are the translation.
    p[0] += Rt[3]; p[1] += Rt[4]; p[2] += Rt[5];

    residuals[0] = p[0] - boardPoint[0];
    residuals[1] = p[1] - boardPoint[1];
    residuals[2] = p[2] - boardPoint[2];
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create() {
     return (new ceres::AutoDiffCostFunction<RigidTransformationError, 3, 6, 3, 3>(
                 new RigidTransformationError()));
   }
};

std::vector<double> modifiedToNormalBrownCoeffs(const std::vector<double>& modifiedBrownCoeffs, cv::Size imgSize);
cv::Mat modifiedToNormalBrownCoeffs(cv::Mat modifiedBrownCoeffs, cv::Size imgSize);
std::vector<double> inversedToNormalBrownCoeffs(const std::vector<double>& inverseBrownCoeffs, cv::Size imgSize);
cv::Mat inversedToNormalBrownCoeffs(cv::Mat inverseBrownCoeffs, cv::Size imgSize);

#endif // CERESMODELS_H
