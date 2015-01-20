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

#ifndef PCL_PBMAP_UTILS_H_
#define PCL_PBMAP_UTILS_H_

#define SMALL_NUM  0.00000001 // anything that avoids division overflow

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/common/common.h>

namespace pcl
{
  namespace pbmap
  {

  //    /** A set of parameters to specify thresholds for the registration.*/
  //    struct config_pbmap
  //    {
  //        // [global]
  //        float color_threshold;
  //        float intensity_threshold;
  //        float hue_threshold;

  //        // [plane_segmentation]
  //        float dist_threshold; // Maximum distance to the plane between neighbor 3D-points
  //        float angle_threshold; //  = 0.017453 * 4.0 // Maximum angle between contiguous 3D-points
  //        float minInliersRate; // Minimum ratio of inliers/image points required

  //        // [map_construction]
  //        bool use_color;                   // Add color information to the planes
  //        float proximity_neighbor_planes;  // Two planar patches are considered neighbors when the closest distance between them is under proximity_neighbor_planes
  //        //  float max_angle_normals; // (10º) Two planar patches that represent the same surface must have similar normals // QUITAR
  //        float max_cos_normal;
  //        float max_dist_center_plane; // Two planar patches that represent the same surface must have their center in the same plane
  //        float proximity_threshold;  // Two planar patches that represent the same surface must overlap or be nearby
  //        int   graph_mode;  // This var selects the condition to create edges in the graph, either proximity of planar patches or co-visibility in a single frame

  //        // [semantics]
  //        bool inferStructure;    // Infer if the planes correspond to the floor, ceiling or walls
  //        bool makeClusters; // Should the PbMapMaker cluster the planes according to their co-visibility

  //        // [localisation]
  //        bool detect_loopClosure;             // Run PbMapLocaliser in a different threads to detect loop closures or preloaded PbMaps
  //    };

    // Define some colours (RGB): red, green, blue, yellow, pink, turquoise, orange, purple, dark green, beige
    const unsigned char R[10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
    const unsigned char G[10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
    const unsigned char B[10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

    const double Rf[10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
    const double Gf[10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
    const double Bf[10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

  /** \brief Calculate Bhattacharyya distance between the given input histograms
    * \param[in] hist1 input histogram
    * \param[in] hist2 input histogram
    */
    double
    calcBhattacharyyaDist (std::vector<float> &hist1, std::vector<float> &hist2);

    /** \brief Transform the (x,y,z) coordinates of a PCL point into a Eigen::Vector3f
      * \param[in] pt input point
      */
    template<class PointT> inline Eigen::Vector3f
    getVector3FromPointXYZ (const PointT & pt)
    {
      return Eigen::Vector3f (pt.x,pt.y,pt.z);
    }

    /** \brief Compute the difference of two PCL points into a Eigen::Vector3f
      * \param[in] pt1 input point
      * \param[in] pt2 input point
      */
    template<class PointT> inline Eigen::Vector3f
    diffPoints (const PointT &pt1, const PointT &pt2)
    {
      Eigen::Vector3f diff;
      diff[0] = pt1.x - pt2.x;
      diff[1] = pt1.y - pt2.y;
      diff[2] = pt1.z - pt2.z;
      return diff;
    }

    /** @b Segment stores a pair of PCL points defining a line segment in 3D.
      * \author Eduardo Fernandez-Moral
      * \ingroup registration
      */
    template <typename PointT>
    struct Segment
    {
    /** \brief Constructor from two points
      * \param[in] p0 input point
      * \param[in] p1 input point
      */
      Segment (PointT p0, PointT p1) :
        pt0_ (p0), pt1_ (p1)
      {};

      /** \brief Points that define the end of the segment. */
      PointT pt0_, pt1_;
    };

    /** \brief Compute the Euclidean distance between two line segments in 3D
      * \param[in] pt1 input point
      * \param[in] pt2 input point
      */
    template <typename PointT> float
    dist3D_Segment_to_Segment2 (Segment<PointT> S1, Segment<PointT> S2);

    /** \brief Get the UNIMODAL histogram mean-shift from variable bandwidth mean-shift.
      * \param[in] data input histogram
      * \param[in] max_range is the maximum value of the elements
      * \param[in] range_in is the range within which the values are considered inliers for the given mean
      */
    double
    getHistMeanShift (std::vector<float> &data, double max_range );

  }
}

#endif //#ifndef PCL_PBMAP_UTILS_H_
