/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_PBMAP_HPP_
#define PCL_PBMAP_HPP_

//#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Archive> void
pcl::pbmap::PbMap<PointT>::serialize (Archive & ar, const unsigned int version)
{
  size_t pbmap_size = patches_.size ();
  ar & pbmap_size;
  if( pbmap_size != patches_.size () )
     patches_.resize ( pbmap_size );

  ar & connected_patches_;
  for (size_t i=0; i< patches_.size (); i++)
    ar & patches_[i];
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PbMap<PointT>::Merge (pcl::pbmap::PbMap &pbm, Eigen::Matrix4f &Rt)
{
  // Rotate and translate PbMap
  for (size_t i = 0; i < pbm.patches_.size(); i++)
  {
    pcl::pbmap::PlanarPatch patch = pbm.patches_[i];
    patch.transformAffine (Rt);

//    // Transform centroid, normal and main direction
//    patch.setCentroid ( Rt.block (0,0,3,3) * patch.getCentroid () + Rt.block (0,3,3,1) );
//    pcl::ModelCoefficients new_coeffs;
//    new_coeffs.values.resize (4);
//    new_coeffs.values[0] = Rt.block (0,0,1,3) * patch.getCoefficients (). block (0,0,3,1);
//    new_coeffs.values[1] = Rt.block (1,0,1,3) * patch.getCoefficients (). block (0,0,3,1);
//    new_coeffs.values[2] = Rt.block (2,0,1,3) * patch.getCoefficients (). block (0,0,3,1);
//    new_coeffs.values[3] = Rt.block (0,3,3,1).transpose () * patch.getCoefficients (). block (0,0,3,1) + patch.getCoefficients ()[3];
//    patch.setCoefficients ( new_coeffs );
//    patch.setMainDirection ( Rt.block (0,0,3,3) * patch.getMainDirection () );

//    // Transform convex hull points
//    pcl::transformPointCloud(*patch.polygonContourPtr, *patch.polygonContourPtr, Rt);

//    pcl::transformPointCloud(*patch.patchPointCloudPtr, *patch.patchPointCloudPtr, Rt);

    patch.id = patches_.size();

    patches_.push_back ( patch );
  }

  // Rotate and translate the point cloud
  pcl::PointCloud<PointT>::Ptr aligned_point_cloud (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud ( *pbm.point_cloud_, *aligned_point_cloud, Rt);

  *point_cloud_ += *aligned_point_cloud;
}


///////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> bool
//pcl::pbmap::PbMap<PointT>::arePatchesNearby(pcl::pbmap::PlanarPatch &patch1, pcl::pbmap::PlanarPatch &patch2, const float distThreshold)
//{
//  float distThres2 = distThreshold * distThreshold;

//  // First we check distances between centroids and vertex to accelerate this check
//  if( (patch1.v3center - patch2.v3center).squaredNorm() < distThres2 )
//    return true;

//  for(unsigned i=1; i < patch1.polygonContourPtr->size(); i++)
//    if( (getVector3fromPointXYZ(patch1.polygonContourPtr->points[i]) - patch2.v3center).squaredNorm() < distThres2 )
//      return true;

//  for(unsigned j=1; j < patch2.polygonContourPtr->size(); j++)
//    if( (patch1.v3center - getVector3fromPointXYZ(patch2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
//      return true;

//  for(unsigned i=1; i < patch1.polygonContourPtr->size(); i++)
//    for(unsigned j=1; j < patch2.polygonContourPtr->size(); j++)
//      if( (diffPoints(patch1.polygonContourPtr->points[i], patch2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
//        return true;

//  //If not found yet, search properly by checking distances:
//  // a) Between an edge and a vertex
//  // b) Between two edges (imagine two polygons on perpendicular patchs)
//  // c) Between a vertex and the inside of the polygon
//  // d) Or the polygons intersect

//  // a) & b)
//  for(unsigned i=1; i < patch1.polygonContourPtr->size(); i++)
//    for(unsigned j=1; j < patch2.polygonContourPtr->size(); j++)
//      if(dist3D_Segment_to_Segment2(Segment(patch1.polygonContourPtr->points[i],patch1.polygonContourPtr->points[i-1]), Segment(patch2.polygonContourPtr->points[j],patch2.polygonContourPtr->points[j-1])) < distThres2)
//        return true;

//  // c)
//  for(unsigned i=1; i < patch1.polygonContourPtr->size(); i++)
//    if( patch2.v3normal.dot(getVector3fromPointXYZ(patch1.polygonContourPtr->points[i]) - patch2.v3center) < distThreshold )
//      if(isInHull(patch1.polygonContourPtr->points[i], patch2.polygonContourPtr) )
//        return true;

//  for(unsigned j=1; j < patch2.polygonContourPtr->size(); j++)
//    if( patch1.v3normal.dot(getVector3fromPointXYZ(patch2.polygonContourPtr->points[j]) - patch1.v3center) < distThreshold )
//      if(isInHull(patch2.polygonContourPtr->points[j], patch1.polygonContourPtr) )
//        return true;

//  return false;
//}

#endif //#ifndef PCL_PBMAP_HPP_
