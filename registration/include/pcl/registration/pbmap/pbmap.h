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

#ifndef PCL_PBMAP_H_
#define PCL_PBMAP_H_

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>

//#include <pcl/registration/registration.h>

namespace pcl
{
  namespace registration
  {

    /** @b PbMap stores a Plane-based Map, which is used to represent and register the planar structure of the scene.
      * It has uses for localization, mapping, place recognition, structure-odometry and loop closure.
      * It can be used with a variety of sensors that retrieve 3D-geometry like LIDAR, depth cameras or stereo vision.
      *
      * \author Eduardo Fernandez-Moral
      * \ingroup registration
      */
    template <typename PointT>
    class PbMap
    {

    };

//          typedef PointCloud<PointSource> PointCloudSource;
//          typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
//          typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

//          typedef PointCloud<PointTarget> PointCloudTarget;

//          typedef PointIndices::Ptr PointIndicesPtr;
//          typedef PointIndices::ConstPtr PointIndicesConstPtr;

//      public:
//          typedef boost::shared_ptr<TransformationEstimationLM<PointSource, PointTarget, MatScalar> > Ptr;
//          typedef boost::shared_ptr<const TransformationEstimationLM<PointSource, PointTarget, MatScalar> > ConstPtr;

//          typedef Eigen::Matrix<MatScalar, Eigen::Dynamic, 1> VectorX;
//          typedef Eigen::Matrix<MatScalar, 4, 1> Vector4;
//          typedef typename TransformationEstimation<PointSource, PointTarget, MatScalar>::Matrix4 Matrix4;

//          /** \brief Constructor. */
//          TransformationEstimationLM ();


//#include <mrpt/config.h>
//#if MRPT_HAS_PCL

//#include <mrpt/utils/utils_defs.h>

//#include <mrpt/utils/CSerializable.h>
//#include <mrpt/pbmap/link_pragmas.h>

//#include <mrpt/pbmap/Plane.h>
//#include <mrpt/pbmap/Miscellaneous.h>  // For typedef PointT;

////#include <boost/thread/thread.hpp>

//namespace mrpt {
//namespace pbmap {
//	using namespace mrpt::utils;

//	// This must be added to any CSerializable derived class:
//	DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( PbMap, PBMAP_IMPEXP)

//	/** A class used to store a Plane-based Map (PbMap). A PbMap consists of a set of planar patches
//	* described by geometric features (shape, relative position, etc.) and/or radiometric features
//	* (dominant color). It is organized as an annotated, undirected graph, where nodes stand for planar
//	* patches and edges connect neighbor planes when the distance between their closest points is under
//	* a threshold. This graph structure permits to find efficiently the closest neighbors of a plane,
//	* or to select groups of nearby planes representing part of the scene.
//   *
//   * \ingroup mrpt_pbmap_grp
//   */
//  class PBMAP_IMPEXP PbMap : public mrpt::utils::CSerializable
//  {
//    // This must be added to any CSerializable derived class:
//    DEFINE_SERIALIZABLE( PbMap )

//   public:
//  /*!Constructor.*/
//    PbMap();

//  /*!Vector to store the 3D-planes which are the basic characteristic of our map.*/
//    std::vector<Plane> vPlanes_;

//  /*!Label to store a semantic attribute*/
//    std::string label_;

//  /*!Floor plane id*/
//    int FloorPlane;

//  /*!Registered point cloud from the RGB-D or Depth frames and visual odometry.*/
//    PointCloud<PointT>::Ptr globalMapPtr;

//    PointCloud<PointXYZRGBA>::Ptr edgeCloudPtr;
//    PointCloud<PointXYZRGBA>::Ptr outEdgeCloudPtr;
//    unsigned background, foreground, groundplane;

//    /*!Save PbMap in the given filePath*/
//    void savePbMap(std::string filePath);

//    /*!Load a PbMap from the given filePath*/
//    void loadPbMap(std::string PbMapFile);

//    /*!Merge two pbmaps*/
//    void MergeWith(PbMap &pbm, Eigen::Matrix4f &T);

//    /*! Print PbMap content to a text file*/
//    void printPbMap(std::string txtFilePbm);

////    boost::mutex mtx_pbmap_busy;

//  };
//  DEFINE_SERIALIZABLE_POST_CUSTOM_LINKAGE( PbMap, PBMAP_IMPEXP)

  }
}

#endif //#ifndef PCL_PBMAP_H_
