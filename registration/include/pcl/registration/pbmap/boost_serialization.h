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

#ifndef BOOST_SERIALIZATION_H_
#define	BOOST_SERIALIZATION_H_

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace boost
{
  namespace serialization
  {
    /** \brief Eigen::Matrix serialization
      * \param[in] ar destination archive
      * \param[in] point_cloud data to serialize
      * \param[in] file_version version number
      */
    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline void
    serialize ( Archive & ar,
                Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & point_cloud,
                const unsigned int file_version )
    {
      size_t rows = point_cloud.rows ();
      size_t cols = point_cloud.cols ();
      ar & rows;
      ar & cols;
      if( rows * cols !=  point_cloud.size () )
         point_cloud.resize ( rows, cols );

      for (size_t i=0; i< point_cloud.size (); i++)
        ar & point_cloud.data ()[i];
    }

  /** \brief pcl::PointCloud serialization
    * \param[in] ar destination archive
    * \param[in] point_cloud data to serialize
    * \param[in] file_version version number
    */
    template<class Archive, typename PointT> inline void
    serialize ( Archive & ar,
                pcl::PointCloud<PointT> & point_cloud,
                const unsigned int file_version )
    {
      ar & point_cloud.height;
      ar & point_cloud.width;
      ar & point_cloud.is_dense;
      size_t cloud_size = point_cloud.height * point_cloud.width;
      if( cloud_size != point_cloud.size () )
         point_cloud.points.resize ( cloud_size );

      for (size_t i=0; i< point_cloud.size (); i++)
      {
        ar & point_cloud.points[i].x;
        ar & point_cloud.points[i].y;
        ar & point_cloud.points[i].z;
      }
    }

  /** \brief pcl::PointCloud serialization
    * \param[in] ar destination archive
    * \param[in] point_cloud data to serialize
    * \param[in] file_version version number
    */
    template<class Archive, typename PointT> inline void
    serialize ( Archive & ar,
                typename pcl::PointCloud<PointT>::VectorType & point_cloud,
                const unsigned int file_version )
    {
      size_t cloud_size = point_cloud.size ();
      ar & cloud_size;
      if( cloud_size != point_cloud.size () )
         point_cloud.resize ( cloud_size );

      for (size_t i=0; i< point_cloud.size (); i++)
      {
        ar & point_cloud[i].x;
        ar & point_cloud[i].y;
        ar & point_cloud[i].z;
      }
    }
  }
}

#endif //#ifndef BOOST_SERIALIZATION_H_
