/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

//#include <pcl/registration/pbmap/pbmap.h>
#include <pcl/registration/pbmap/planar_patch.h>

//#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>

//#include <boost/serialization/base_object.hpp>
//#include <boost/serialization/utility.hpp>
//#include <boost/serialization/list.hpp>
//#include <boost/serialization/assume_abstract.hpp>

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/surface/convex_hull.h>

//using pcl::Region3D<PointT>::centroid_;
//using Region3D<PointT>::covariance_;
//using Region3D<PointT>::count_;
//using PlanarPolygon<PointT>::contour_;
//using PlanarPolygon<PointT>::coefficients_;

/** \brief Normalize plane's coefficients, that is, make the plane coefficients {A,B,C} in ( Ax+By+Cz+D=0 )coincide with the normal vector
 */
//void
//NormalizePlaneCoefs ()
//{
//    //float normABC = centroid_[0];
//    float normABC = sqrt(coefficients_[0]*coefficients_[0] + coefficients_[1]*coefficients_[1] + coefficients_[2]*coefficients_[2]);
//    coefficients_[0] /= normABC;
//    coefficients_[1] /= normABC;
//    coefficients_[2] /= normABC;
//}

/** \brief Force the 3D points of the plane to actually lay on the plane
 */
//void
//Plane::forcePtsLayOnPlane()
//{
//  // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter
//  const double D = -(v3normal.dot(v3center));
//  for(unsigned i = 0; i < planePointCloudPtr->size(); i++)
//  {
//    double dist = v3normal[0]*planePointCloudPtr->points[i].x + v3normal[1]*planePointCloudPtr->points[i].y + v3normal[2]*planePointCloudPtr->points[i].z + D;
//    planePointCloudPtr->points[i].x -= v3normal[0] * dist;
//    planePointCloudPtr->points[i].y -= v3normal[1] * dist;
//    planePointCloudPtr->points[i].z -= v3normal[2] * dist;
//  }
//  // Do the same with the points defining the convex hull
//  for(unsigned i = 0; i < polygonContourPtr->size(); i++)
//  {
//    double dist = v3normal[0]*polygonContourPtr->points[i].x + v3normal[1]*polygonContourPtr->points[i].y + v3normal[2]*polygonContourPtr->points[i].z + D;
//    polygonContourPtr->points[i].x -= v3normal[0] * dist;
//    polygonContourPtr->points[i].y -= v3normal[1] * dist;
//    polygonContourPtr->points[i].z -= v3normal[2] * dist;
//  }
//}

//void
//Plane::computeConvexHull (pcl::PointCloud<pcl::PointXYZRGBA> points, std::vector<size_t> &indices)
//{
//    assert (!points->points.empty ());

//    pcl::PointCloud<pcl::PointXYZRGBA> point_cloud;
//    pcl::ConvexHull<pcl::PointXYZRGBA> convex_hull;
//    convex_hull.setDimension (2);
//    std::vector<pcl::Vertices> polygons;
//    convex_hull.reconstruct (points, polygons);
//    getTotalArea ();

//}

///** \brief Compute the patch's gravity center
//  */
//void
//Plane::computeCenter ()
//{
//  assert (!polygonContourPtr->points.empty ());

//  int k0, k1, k2;

//  // Find axis with largest normal component and project onto perpendicular plane
//  k0 = (fabs (v3normal[0] ) > fabs (v3normal[1])) ? 0  : 1;
//  k0 = (fabs (v3normal[k0]) > fabs (v3normal[2])) ? k0 : 2;
//  k1 = (k0 + 1) % 3;
//  k2 = (k0 + 2) % 3;

//  // cos(theta), where theta is the angle between the polygon and the projected plane
//  float ct = fabs ( v3normal[k0] );
//  float AreaX2 = 0.0;
//  Eigen::Vector3f massCenter = Eigen::Vector3f::Zero();
//  float p_i[3], p_j[3];

//  for (unsigned int i = 0; i < polygonContourPtr->points.size (); i++)
//  {
//    p_i[0] = polygonContourPtr->points[i].x; p_i[1] = polygonContourPtr->points[i].y; p_i[2] = polygonContourPtr->points[i].z;
//    int j = (i + 1) % polygonContourPtr->points.size ();
//    p_j[0] = polygonContourPtr->points[j].x; p_j[1] = polygonContourPtr->points[j].y; p_j[2] = polygonContourPtr->points[j].z;
//    double cross_segment = p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];

//    AreaX2 += cross_segment;
//    massCenter[k1] += (p_i[k1] + p_j[k1]) * cross_segment;
//    massCenter[k2] += (p_i[k2] + p_j[k2]) * cross_segment;
//  }
//  areaHull = fabs (AreaX2) / (2 * ct);

//  massCenter[k1] /= (3*AreaX2);
//  massCenter[k2] /= (3*AreaX2);
//  massCenter[k0] = (v3normal.dot(v3center) - v3normal[k1]*massCenter[k1] - v3normal[k2]*massCenter[k2]) / v3normal[k0];

//  v3center = massCenter;

//  d = -(v3normal. dot (v3center));
//}

//void
//Plane::calcElongationAndPpalDir ()
//{
//  pcl::PCA< PointT > pca;
//  pca.setInputCloud(planePointCloudPtr);
//  Eigen::VectorXf eigenVal = pca.getEigenValues();
////  if( eigenVal[0] > 2 * eigenVal[1] )
//  {
//    elongation = sqrt(eigenVal[0] / eigenVal[1]);
//    Eigen::MatrixXf eigenVect = pca.getEigenVectors();
////    v3PpalDir = makeVector(eigenVect(0,0), eigenVect(1,0), eigenVect(2,0));
//    v3PpalDir[0] = eigenVect(0,0);
//    v3PpalDir[1] = eigenVect(1,0);
//    v3PpalDir[2] = eigenVect(2,0);
//  }
//}
