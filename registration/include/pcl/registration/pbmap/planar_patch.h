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

#include <Eigen/SVD>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/access.hpp>

#include <pcl/segmentation/planar_region.h>
#include <pcl/registration/pbmap/utils.h>
#include <pcl/registration/pbmap/boost_serialization.h>

namespace pcl
{
  namespace pbmap
  {

  /** @b PlanarPatch stores a pcl::segmentation::PlanarRegion extended by some geometric and radiometric characteristics.
    *
    * \author Eduardo Fernandez-Moral
    * \ingroup registration
    */
    template <typename PointT>
    class PlanarPatch : public pcl::PlanarRegion<PointT>
    {
      private:

        // Parameters to allow the plane-based representation of the map by a graph
        /** \brief Patch's index. */
        unsigned id_;
        /** \brief Nuber of times the plane has been observed. */
        unsigned num_obs_;
        /** \brief Semantic property. */
        unsigned semantic_group_;

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

        /** \brief Area of the patch computed from the convex hull. */
        float area_;
        /** \brief Relation between the largest and the second largest eigenvalues of the patch's points. */
        float elongation_;
        /** \brief Eigenvector corresponding to the largest eigenvalue of the patch's points. */
        Eigen::Vector3f v_main_direction_;
//        /** \brief Plane tag. */
//        float area;
        /** \brief This boolean tells whether the patch covers completely the physical planar surface (observed without occlussions). */
        bool b_real_boundary_;
        /** \brief This boolean tells whether the patch corresponds to the scene structure (i.e. floor, walls, ceiling). */
        bool b_scene_structure_;
//        /** \brief Plane tag. */
//        unsigned nFramesAreaIsStable;

        // Radiometric description
        /** \brief Normalized rgb value corresponding to the dominant colour plus the dominant Intensity. */
        Eigen::Vector4f v_dominant_rgb_int;
        /** \brief This boolean tells whether a dominant single colour exists or not. */
        bool b_dominant_color_;
        /** \brief Standard deviation of the dominant colour. */
        Eigen::Vector3f v_dominant_rgb_dev_;
        /** \brief Histogram of the Hue values, implemented following the paper:
         * "Utilizing color information in 3d scan registration using planar-patches matching" K. Pathak et a. 2012. */
        std::vector<float> hue_histogram_;

        /** \brief The segmented points on the planar patch. */
//        pcl::PointCloud<PointT> patch_points_;
        typename pcl::PointCloud<PointT>::Ptr patch_points_;
        //std::vector<int32_t> inliers;

      protected:
        using Region3D<PointT>::centroid_;
        using Region3D<PointT>::covariance_;
        using Region3D<PointT>::count_;
        using PlanarPolygon<PointT>::contour_;
        using PlanarPolygon<PointT>::coefficients_;

        /** \brief PlanarPatch serialization
          * \param[in] ar destination archive
          * \param[in] version number
          */
        friend class boost::serialization::access;

        template<class Archive> void
        serialize (Archive & ar, const unsigned int version);
//        {
//          ar & id_;
//          ar & num_obs_;
//          ar & semantic_group_;

//          ar & label_;
//          ar & label_object_;
//          ar & label_context_;

//          ar & elongation_;
//          ar & v_main_direction_;

//          ar & centroid_;
//          ar & covariance_;
//          ar & count_;
//          ar & contour_;
//          ar & coefficients_;
//        }

      public:

        /** \brief Empty constructor. */
        PlanarPatch () :
            elongation_ (1.0),
            b_real_boundary_ (false),
            b_scene_structure_ (false),
            patch_points_ (new PointCloud<PointT>)
        {
        }

        /** \brief Constructor for PlanarPatch region from a PlanarRegion
          * \param[in] planar_region a PlanarRegion for the input data
          */
        PlanarPatch (const PlanarRegion<PointT> &planar_region) :
            elongation_ (1.0),
            b_real_boundary_ (false),
            b_scene_structure_ (false),
            patch_points_ (new PointCloud<PointT>)
        {
            centroid_   = planar_region.getCentroid ();
            covariance_ = planar_region.getCovariance ();
            count_      = planar_region.getCount ();
            contour_    = planar_region.getContour ();
            coefficients_ = planar_region.getCoefficients ();
        }

        /** \brief Set the centroid of the patch. */
        inline void
        setCentroid (const Eigen::Vector3f &centroid)
        {
          centroid_ = centroid;
        }

        /** \brief Get the centroid of the region. */
        inline Eigen::Vector3f
        getNormal () const
        {
          return this->getCoefficients ().block (0,0,3,1);
        }

        /** \brief Get the centroid of the region. */
        inline float
        getArea () const
        {
          return area_;
        }

        /** \brief Get the centroid of the region. */
        inline Eigen::Vector3f
        getMainDirection () const
        {
          return (v_main_direction_);
        }

        /** \brief Get the centroid of the region. */
        inline void
        setMainDirection (const Eigen::Vector3f &v_main_direction)
        {
          v_main_direction_ = v_main_direction;
        }

        /** \brief Get the number of observations. */
        inline unsigned &
        getObservations ()
        {
          return (num_obs_);
        }

        /** \brief Get the number of observations. */
        inline std::string &
        getLabel ()
        {
          return (label_);
        }

        /** \brief Normalize plane's coefficients, that is, make the plane coefficients {A,B,C} in ( Ax+By+Cz+D=0 )coincide with the normal vector
         */
        void
        NormalizePlaneCoefs ();

        /** \brief Refine the patch's convex hull
          */
        void
        refineConvexHull ();

        /** \brief Compute the patch's convex-hull area and mass center
        */
        void computeCentroidAndArea();

        /** \brief Calculate plane's elongation and principal direction
          */
        void
        calcElongationAndPpalDir ();

        /** \brief Compute the patch's dominant colour using "MeanShift" method */
        void
        computeDominantColour ();

        /** \brief Compute the patch's saturated Hue histogram following the paper:
         * "Utilizing color information in 3d scan registration using planar-patches matching" K. Pathak et a. 2012. */
        void
        computeHueHistogram ();

        /** \brief Tell whether the current patch is closer to the input patch than a given threshold
          * \param[in] patch the patch to compare
          * \param[in] threshold_dist
          */
        bool
        isPatchNearby(const PlanarPatch<PointT> & patch, const float threshold_dist);

        /** \brief Compute the patch's saturated Hue histogram following the paper:
         * "Utilizing color information in 3d scan registration using planar-patches matching" K. Pathak et a. 2012. */
        void
        mergePlanes(const PlanarPatch<PointT> & new_observation);

        /** \brief Transform the patch coordinates according to the
          * \param[in] Rt affine transformation */
        void
        transformAffine (Eigen::Matrix4f &Rt);

        /** \brief Force the 3D points of the plane to actually lay on the plane
         */
        void
        forcePtsLayOnPlane ();

//        /** \brief Calculate the plane's geometric parameters (i.e. Area, ...).
//         */
//        void computeParameters ();

//        /** \brief Calculate the plane's convex hull with the monotone chain algorithm.
//         * \param[in] cloud_arg pointer to input point cloud obtained from the planar segmentation
//         * \param[in] indices_arg indices vector of the convex hull
//         */
//        void
//        calcConvexHull (PointCloud<PointXYZRGBA>::Ptr &cloud_arg, std::vector<size_t> &indices_arg = DEFAULT_VECTOR );

//        /** \brief Compute the patch's center (the gravity center of its convex hull)
//          */
//        void
//        computeCenter ()
//        {

//        };

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

    };
  }
}

#endif //#ifndef PCL_PLANAR_PATCH_H_
