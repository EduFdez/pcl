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
#include <map>

//#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

//#include <boost/serialization/base_object.hpp>
//#include <boost/serialization/utility.hpp>
//#include <boost/serialization/list.hpp>
//#include <boost/serialization/assume_abstract.hpp>

#include <pcl/registration/pbmap/planar_patch.h>

namespace pcl
{
  namespace pbmap
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
      protected:
        /** Vector to store the 3D-planes which are the basic characteristic of our map. */
        std::vector< PlanarPatch<PointT> > patches_;

//        /** \brief Set of connected planes (nearby and co-visible planes). */
//        std::set<unsigned> connected_patches_;
        /** \brief Set of connected planes (nearby and co-visible planes). */
        std::map<unsigned, std::set<unsigned> > connected_patches_;

        /** \brief Point cloud constructed from the input images from where the patches have been extracted.*/
        typename pcl::PointCloud<PointT>::Ptr point_cloud_;

        friend class boost::serialization::access;

        /** \brief PlanarPatch serialization
          * \param[in] ar destination archive
          * \param[in] version number
          */
        template<class Archive> void
        serialize (Archive & ar, const unsigned int version);

      public:
        /** \brief Merge this PbMap with
          * \param[in] pbmap input PbMap, which has the coordinate system given by
          * \param[in] Rt affine transformation
          */
        void
        Merge(PbMap &pbm, Eigen::Matrix4f &Rt);
    };

  }
}

#endif //#ifndef PCL_PBMAP_H_
