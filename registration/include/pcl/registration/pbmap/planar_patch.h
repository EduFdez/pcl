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

#ifndef PCL_PLANAR_PATCH_H_
#define PCL_PLANAR_PATCH_H_

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <pcl/segmentation/planar_region.h>
//#include <pcl/registration/registration.h>

namespace pcl
{
  namespace registration
  {

  /** @b PlanarPatch stores a pcl::segmentation::PlanarRegion extended by some geometric and radiometric characteristics.
    *
    * \author Eduardo Fernandez-Moral
    * \ingroup registration
    */
    template <typename PointT>
    class PlanarPatch : public pcl::segmentation::PlanarRegion<PointT>
    {
      private:

        // Parameters to allow the plane-based representation of the map by a graph
        /** \brief Patch's index. */
        unsigned id_;
        /** \brief Nuber of times the plane has been observed. */
        unsigned num_obs_;
        /** \brief Semantic property. */
        unsigned semantic_group_;
        /** \brief Set of connected planes (nearby and co-visible planes). */
        std::set<unsigned> connected_planes_;
        /** \brief Set of connected planes (nearby and co-visible planes). */
        std::map<unsigned,unsigned> neighborPlanes_;

        // Labels to store semantic attributes
        /** \brief Patch's tag. */
        std::string label_;
        /** \brief Object tag. */
        std::string label_object_;
        /** \brief Context tag. */
        std::string label_context_;

        // Geometric description
//        /** \brief Patch's center. */
//        Eigen::Vector3f v3_center;
//        /** \brief Plane tag. */
//        Eigen::Vector3f v3_normal;
//        /** \brief Distance to the world reference system. In the plane equation: v3_normal*point + dist = 0 */
//        float dist;
//        /** \brief Information matrix of the plane parameters: (v3_normal, dist) */
//        Eigen::Matrix4f information_matrix; // Fisher information matrix (the inverse of the plane covariance)
//        /** \brief Plane tag. */
//        float curvature;
        /** \brief Eigenvector corresponding to the largest eigenvalue of the patch's points. */
        Eigen::Vector3f v_main_direction_;
        /** \brief Relation between the largest and the second largest eigenvalues. */
        float elongation_; // This is the reatio between the lengths of the plane in the two principal directions
//        /** \brief Plane tag. */
//        float area;
        /** \brief This boolean tells whether the patch covers completely the physical planar surface (observed without occlussions). */
        bool b_real_boundary_;
        /** \brief This boolean tells whether the patch corresponds to the scene structure (i.e. floor, walls, ceiling). */
        bool b_scene_structure_;
//        /** \brief Plane tag. */
//        unsigned nFramesAreaIsStable;

        // Radiometric description
        /** \brief Normalized rgb value corresponding to the dominant colour. */
        Eigen::Vector3f v_normalized_rgb_;
        /** \brief Dominant intensity. */
        float dominant_intensity_;
        /** \brief This boolean tells whether a dominant single colour exists or not. */
        bool b_dominant_color_;
        /** \brief Standard deviation of the dominant colour. */
        Eigen::Vector3f v_normalized_rgb_dev_;
        /** \brief Histogram of the Hue values, implemented following the paper:
         * "Utilizing color information in 3d scan registration using planar-patches matching" K. Pathak et a. 2012.
         */
        std::vector<float> hue_histogram_;

        pcl::PointCloud<PointT>::Ptr patch_points_;
        //std::vector<int32_t> inliers;

//        /**!
//         *  Convex Hull
//        */
//        std::vector<int32_t> inliers;
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr polygonContourPtr;
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outerPolygonPtr; // This is going to be deprecated
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloudPtr; // This is going to be deprecated

      protected:
        using Region3D<PointT>::centroid_;
        using Region3D<PointT>::covariance_;
        using Region3D<PointT>::count_;
        using PlanarPolygon<PointT>::contour_;
        using PlanarPolygon<PointT>::coefficients_;

      public:

        /** \brief Empty constructor. */
        Plane () :
            elongation (1.0),
            b_real_boundary (false),
            b_scene_structure (false)
            //convex_hull (new PointCloud<PointXYZRGBA>),
//            points (new PointCloud<PointXYZRGBA>)
        {
        }

        /** \brief Constructor for PlanarPatch region from a PlanarRegion
          * \param[in] planar_region a PlanarRegion for the input data
          */
        Plane (const PlanarRegion<PointT> &planar_region) :
            elongation (1.0),
            b_real_boundary (false),
            b_scene_structure (false)
        {
            centroid_ = planar_region.centroid;
            covariance_ = planar_region.covariance;
            count_ = planar_region.count;
            contour_ = planar_region.contour;
            coefficients_ = planar_region.coefficients;
        }

        /** \brief Normalize plane's coefficients, that is, make the plane coefficients {A,B,C} in ( Ax+By+Cz+D=0 )coincide with the normal vector
         */
        void
        NormalizePlaneCoefs ();

        /** \brief Force the 3D points of the plane to actually lay on the plane
         */
        void
        forcePtsLayOnPlane ();

//        /** \brief Add point simultaneously to octree and input point cloud. A corresponding index will be added to the indices vector.
//         * \param[in] point_arg point to be added
//         * \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
//         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
//         */


//        /** \brief Calculate the plane's convex hull with the monotone chain algorithm.
//         * \param[in] cloud_arg pointer to input point cloud obtained from the planar segmentation
//         * \param[in] indices_arg indices vector of the convex hull
//         */
//        void
//        calcConvexHull (PointCloud<PointXYZRGBA>::Ptr &cloud_arg, std::vector<size_t> &indices_arg = DEFAULT_VECTOR );

        /** \brief Compute the patch's gravity center
          */
        void
        computeCenter ();

        /** \brief Calculate plane's elongation and principal direction
          */
        void
        calcElongationAndPpalDir ();


        /** \brief Calculate the plane's geometric parameters (i.e. Area, ...).
         */
        void computeParameters ();

//        /*!
//         * Calculate plane's main color using "MeanShift" method
//         */
//        void calcMainColor();
//        void calcMainColor2();
//        void calcPlaneHistH();

//       private:
//        /*!
//         * Calculate plane's main color in normalized rgb space
//         */
//        void getPlaneNrgb();
//        std::vector<float> r;
//        std::vector<float> g;
//        std::vector<float> b;
//        std::vector<float> intensity;


//      /** \brief Compute the area of a 2D planar polygon patch - using a given normal
//  //      * \param polygonContourPtr the point cloud (planar)
//  //      * \param normal the plane normal
//        */
//      float compute2DPolygonalArea (/*PointCloud<PointXYZRGBA>::Ptr &polygonContourPtr, Vector<3> &normal*/);

//      /** \brief Compute the patch's convex-hull area and mass center
//        */
//      void computeMassCenterAndArea();


//      /*!Returns true when the closest distance between the patches "this" and "plane" is under distThreshold.*/
//      bool isPlaneNearby(Plane &plane, const float distThreshold); // pbmap

//      /*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
//       * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
//      bool isSamePlane(Plane &plane, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold); // pbmap

//      bool isSamePlane(Eigen::Matrix4f &Rt, Plane &plane_, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

//      bool hasSimilarDominantColor(Plane &plane, const float colorThreshold);// pbmap

//      /*! Merge the two input patches into "updatePlane".
//       *  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
//       */
//      void mergePlane(Plane &plane);// pbmap
//      void mergePlane2(Plane &plane);// Adaptation for RGBD360

      void transformAffine(Eigen::Matrix4f &Rt);

    };
  }
}

#endif //#ifndef PCL_PLANAR_PATCH_H_
