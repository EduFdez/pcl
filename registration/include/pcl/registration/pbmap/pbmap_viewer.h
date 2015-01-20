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

#ifndef PCL_PBMAP_VIEWER_H_
#define PCL_PBMAP_VIEWER_H_

#include <boost/shared_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/pbmap/pbmap_ptr_->h>

namespace pcl
{
  namespace pbmap
  {

    /** @b PbMapViewer displays a pbmap_ptr_->
      *
      * \author Eduardo Fernandez-Moral
      * \ingroup registration
      */
    template <typename PointT>
    class PbMapViewer
    {
      protected:

        boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

        //bool show_graph_ = false;
        bool show_cloud_;
        bool show_pbmap_ = true;

        bool show_pbmap_;
        bool show_cloud_;
        bool show_graph_;
      public:

        /** The PbMap to show. */
        pcl::pbmap::PbMap<PointT>::Ptr pbmap_ptr_;

        /** \brief Empty constructor. */
        PbMapViewer() :
            cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PbMap")),
            pbmap_ptr_ ( new pcl::pbmap::PbMap<PointT> ),
            show_cloud_ (true),
            show_cloud_ (false),
            show_cloud_ (false)
        {
        }

        /** \brief Constructor. */
        PbMapViewer(const pcl::pbmap::PbMap<PointT>::Ptr & pbm) :
            cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PbMap")),
            pbmap_ptr_ (pbm),
            show_cloud_ (true),
            show_cloud_ (false),
            show_cloud_ (false)
        {
        }

        void
        printHelp ()
        {
            std::cout <<"-------------------------------------------------------------------------------------" << std::endl;
            std::cout <<"./pbmap_visualizer <pointCloud.pcd> <planes.pbmap>"                                    << std::endl;
            std::cout <<"       options: "                                                                      << std::endl;
            std::cout <<"         -p | P: Show/hide point cloud"                                                << std::endl;
            std::cout <<"         -l | L: Show/hide PbMap"                                                      << std::endl;
            std::cout <<"         -r | R: Switch between point cloud and graph representation"                  << std::endl;
        }

        void
        keyboard_callback (const pcl::visualization::KeyboardEvent &event, void*)
        {
          if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
          {
            std::cout << "Switch between regular/graph representation" << std::endl;
            show_graph_ = !show_graph_;
          }
          else if ( (event.getKeySym () == "p" || event.getKeySym () == "P") && event.keyDown ())
          {
            std::cout << "Show/hide point cloud" << std::endl;
            show_cloud_ = !show_cloud_;
          }
          else if ( (event.getKeySym () == "l" || event.getKeySym () == "L") && event.keyDown ())
          {
              std::cout << "Show/hide PbMap" << std::endl;
            show_pbmap_ = !show_pbmap_;
          }
        }

        void
        mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
        {
          if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
          {
            cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
          }
        }

//        void
//        viz_cb (pcl::visualization::PCLVisualizer& viz)
//        {
//          if (pbmap_ptr_->point_cloud_->empty ())
//          {
//            boost::this_thread::sleep (boost::posix_time::milliseconds (10));
//            return;
//          }

//          // Render the data
//          {
//            viz.removeAllShapes ();
//            viz.removeAllPointClouds ();

//            char name[1024];

//            if (show_graph_)
//            {
//                //      cout << "show show_graph_\n";
//              for(size_t i=0; i<pbmap_ptr_->patches_.size (); i++)
//              {
//                pcl::PointXYZ center (2*pbmap_ptr_->patches_[i].getCentroid ()[0], 2*pbmap_ptr_->patches_[i].getCentroid ()[1], 2*pbmap_ptr_->patches_[i].getCentroid ()[2]);
//                double radius = 0.1 * sqrt (pbmap_ptr_->patches_[i].areaVoxels);
//                //        cout << "radius " << radius << std::endl;
//                sprintf (name, "sphere%u", static_cast<unsigned>(i));
//                viz.addSphere (center, radius, Rf[i%10], Gf[i%10], Bf[i%10], name);

//                if( !pbmap_ptr_->patches_[i].getLabel ().empty () )
//                    viz.addText3D (pbmap_ptr_->patches_[i].getLabel (), center, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], pbmap_ptr_->patches_[i].getLabel ());
//                else
//                {
//                  sprintf (name, "P%u", static_cast<unsigned>(i));
//                  viz.addText3D (name, center, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], name);
//                }

//                for (map<unsigned,unsigned>::iterator it = pbmap_ptr_->patches_[i].neighborPlanes.begin(); it != pbmap_ptr_->patches_[i].neighborPlanes.end(); it++)
//                {
//                  if (it->first > pbmap_ptr_->patches_[i].id)
//                    break;

//                  sprintf (name, "common_observations%u_%u", static_cast<unsigned>(i), static_cast<unsigned> (it->first));
//                  pcl::PointXYZ center_it (2*pbmap_ptr_->patches_[it->first].getCentroid ()[0], 2*pbmap_ptr_->patches_[it->first].getCentroid ()[1], 2*pbmap_ptr_->patches_[it->first].getCentroid ()[2]);
//                  viz.addLine (center, center_it, Rf[i%10], Gf[i%10], Bf[i%10], name);

//                  sprintf (name, "edge%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
//                  char common_observations[8];
//                  sprintf (common_observations, "%u", it->second);
//                  pcl::PointXYZ half_edge ( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
//                  viz.addText3D (common_observations, half_edge, 0.05, 1.0, 1.0, 1.0, name);
//                }
//              }
//            }
//            else
//            { // Regular representation
//              if (!viz.updatePointCloud (pbmap_ptr_->point_cloud_, "cloud"))
//                viz.addPointCloud (pbmap_ptr_->point_cloud_, "cloud");

//              sprintf (name, "PointCloud size %u", static_cast<unsigned>( pbmap_ptr_->point_cloud_->size() ) );
//              viz.addText(name, 10, 20);

//              for (size_t i=0; i<pbmap_ptr_->patches_.size(); i++)
//              {
//                Plane &plane_i = pbmap_ptr_->patches_[i];
//        //        sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                name[0] = *(mrpt::format("normal_%u", static_cast<unsigned>(i)).c_str());
//                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                pt1 = pcl::PointXYZ(plane_i.getCentroid ()[0], plane_i.getCentroid ()[1], plane_i.getCentroid ()[2]);
//                pt2 = pcl::PointXYZ(plane_i.getCentroid ()[0] + (0.5f * plane_i.getNormal ()[0]),
//                                    plane_i.getCentroid ()[1] + (0.5f * plane_i.getNormal ()[1]),
//                                    plane_i.getCentroid ()[2] + (0.5f * plane_i.getNormal ()[2]));
//                viz.addArrow (pt2, pt1, Rf[i%10], Gf[i%10], Bf[i%10], false, name);

//                if ( !plane_i.getLabel ().empty () )
//                  viz.addText3D (plane_i.getLabel (), pt2, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], plane_i.getLabel ());
//                else
//                {
//                  sprintf (name, "n%u", static_cast<unsigned>(i));
//                  viz.addText3D (name, pt2, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], name);
//                }

//                sprintf (name, "approx_plane_%02d", int (i));
//                viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * R[i%10], 0.5 * G[i%10], 0.5 * B[i%10], name);
//              }
//            }
//          }
//        }

        void
        Visualize ()
        {
          cloud_viewer_->registerMouseCallback (&PbMapViewer::mouse_callback, *this);
          cloud_viewer_->registerKeyboardCallback (&PbMapViewer::keyboard_callback, *this);
          cloud_viewer_->setCameraFieldOfView (1.02259994f);
//          cloud_viewer_->runOnVisualizationThread (boost::bind (&PbMapViewer::viz_cb, this, _1), "viz_cb");

          while (!cloud_viewer_->wasStopped () && !pbmap_ptr_->patches_->empty ())
          {
            cloud_viewer_->spinOnce (100);

            if (pbmap_ptr_->pbmap_mutex_.try_lock ())// Render the data
            {     boost::mutex::scoped_lock lock (pbmap_ptr_->pbmap_mutex_);
              viz.removeAllShapes ();
              viz.removeAllPointClouds ();

              char name[1024];

              if (show_graph_)
              {
                  //      cout << "show show_graph_\n";
                for(size_t i=0; i<pbmap_ptr_->patches_.size (); i++)
                {
                  pcl::PointXYZ center (2*pbmap_ptr_->patches_[i].getCentroid ()[0], 2*pbmap_ptr_->patches_[i].getCentroid ()[1], 2*pbmap_ptr_->patches_[i].getCentroid ()[2]);
                  double radius = 0.1 * sqrt (pbmap_ptr_->patches_[i].areaVoxels);
                  //        cout << "radius " << radius << std::endl;
                  sprintf (name, "sphere%u", static_cast<unsigned>(i));
                  viz.addSphere (center, radius, Rf[i%10], Gf[i%10], Bf[i%10], name);

                  if( !pbmap_ptr_->patches_[i].getLabel ().empty () )
                      viz.addText3D (pbmap_ptr_->patches_[i].getLabel (), center, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], pbmap_ptr_->patches_[i].getLabel ());
                  else
                  {
                    sprintf (name, "P%u", static_cast<unsigned>(i));
                    viz.addText3D (name, center, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], name);
                  }

                  for (map<unsigned,unsigned>::iterator it = pbmap_ptr_->patches_[i].neighborPlanes.begin(); it != pbmap_ptr_->patches_[i].neighborPlanes.end(); it++)
                  {
                    if (it->first > pbmap_ptr_->patches_[i].id)
                      break;

                    sprintf (name, "common_observations%u_%u", static_cast<unsigned>(i), static_cast<unsigned> (it->first));
                    pcl::PointXYZ center_it (2*pbmap_ptr_->patches_[it->first].getCentroid ()[0], 2*pbmap_ptr_->patches_[it->first].getCentroid ()[1], 2*pbmap_ptr_->patches_[it->first].getCentroid ()[2]);
                    viz.addLine (center, center_it, Rf[i%10], Gf[i%10], Bf[i%10], name);

                    sprintf (name, "edge%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
                    char common_observations[8];
                    sprintf (common_observations, "%u", it->second);
                    pcl::PointXYZ half_edge ( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
                    viz.addText3D (common_observations, half_edge, 0.05, 1.0, 1.0, 1.0, name);
                  }
                }
              }
              else
              { // Regular representation
                if (!viz.updatePointCloud (pbmap_ptr_->point_cloud_, "cloud"))
                  viz.addPointCloud (pbmap_ptr_->point_cloud_, "cloud");

                sprintf (name, "PointCloud size %u", static_cast<unsigned>( pbmap_ptr_->point_cloud_->size() ) );
                viz.addText(name, 10, 20);

                for (size_t i=0; i<pbmap_ptr_->patches_.size(); i++)
                {
                  Plane &plane_i = pbmap_ptr_->patches_[i];
          //        sprintf (name, "normal_%u", static_cast<unsigned>(i));
                  name[0] = *(mrpt::format("normal_%u", static_cast<unsigned>(i)).c_str());
                  pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
                  pt1 = pcl::PointXYZ(plane_i.getCentroid ()[0], plane_i.getCentroid ()[1], plane_i.getCentroid ()[2]);
                  pt2 = pcl::PointXYZ(plane_i.getCentroid ()[0] + (0.5f * plane_i.getNormal ()[0]),
                                      plane_i.getCentroid ()[1] + (0.5f * plane_i.getNormal ()[1]),
                                      plane_i.getCentroid ()[2] + (0.5f * plane_i.getNormal ()[2]));
                  viz.addArrow (pt2, pt1, Rf[i%10], Gf[i%10], Bf[i%10], false, name);

                  if ( !plane_i.getLabel ().empty () )
                    viz.addText3D (plane_i.getLabel (), pt2, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], plane_i.getLabel ());
                  else
                  {
                    sprintf (name, "n%u", static_cast<unsigned>(i));
                    viz.addText3D (name, pt2, 0.1, Rf[i%10], Gf[i%10], Bf[i%10], name);
                  }

                  sprintf (name, "approx_plane_%02d", int (i));
                  viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * R[i%10], 0.5 * G[i%10], 0.5 * B[i%10], name);
                }
              }
            }
            else
              boost::this_thread::sleep (boost::posix_time::milliseconds (1));
          }
        }
    };

  }
}

#endif //#ifndef PCL_PBMAP_VIEWER_H_
