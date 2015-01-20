/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/io/openni2/openni2_device.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointT;

class PbMapConstruction
{
  private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    //boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

    boost::mutex cloud_mutex_;
    //boost::mutex image_mutex_;

    pcl::PointCloud<PointT>::ConstPtr cloud_;
    //boost::shared_ptr<pcl::io::openni2::Image> image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;

  public:
    PbMapConstruction ()
    {
    }
    ~PbMapConstruction ()
    {
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
    {
      boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("CloudViewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
      viewer->addPointCloud<PointT> (cloud, single_color, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
      viewer->addCoordinateSystem (1.0, "global");
      viewer->initCameraParameters ();
      return (viewer);
    }

    void
    cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!cloud_viewer_->wasStopped ())
      {
        cloud_mutex_.lock ();
        cloud_ = cloud;
        cloud_mutex_.unlock ();
      }
      //FPS_CALC ("cloud callback");
      //boost::mutex::scoped_lock lock (cloud_mutex_);
      //cloud_ = cloud;
    }

//    void
//    removePreviousDataFromScreen (size_t prev_models_size)
//    {
//      char name[1024];
//      for (size_t i = 0; i < prev_models_size; i++)
//      {
//        sprintf (name, "normal_%lu", i);
//        viewer->removeShape (name);

//        sprintf (name, "plane_%02zu", i);
//        viewer->removePointCloud (name);
//      }
//    }

//    void
//    run ()
//    {

//    }
};

int
main ()
{
  std::cout << "PbMapC-onstruction Application\n";

  pcl::io::OpenNI2Grabber grabber_;

//  pcl::io::OpenNI2Grabber grabber;
//  boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&PbMapViewer::cloud_callback, this, _1);
//  boost::signals2::connection cloud_connection = grabber.registerCallback (cloud_cb);
//  grabber.start ();

//  pcl::pbmap::PbMap pbmap;
//  PbMapViewer<PointT> pbmap_viewer (pbmap);



//  cloud_viewer_->registerMouseCallback (&PbMapViewer::mouse_callback, *this);
//  cloud_viewer_->registerKeyboardCallback (&PbMapViewer::keyboard_callback, *this);
//  cloud_viewer_->setCameraFieldOfView (1.02259994f);
//  boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&PbMapViewer::cloud_callback, this, _1);
//  boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

//  boost::signals2::connection image_connection;
//  if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
//  {
//    image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
//    image_viewer_->registerMouseCallback (&PbMapViewer::mouse_callback, *this);
//    image_viewer_->registerKeyboardCallback (&PbMapViewer::keyboard_callback, *this);
//    boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&PbMapViewer::image_callback, this, _1);
//    image_connection = grabber_.registerCallback (image_cb);
//  }

//  bool image_init = false, cloud_init = false;

//  grabber_.start ();



//  pcl::Grabber* interface = new pcl::OpenNI2Grabber ();

//  boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&PbMapConstruction::cloud_cb_, this, _1);

//  //make a viewer
//  pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);
//  viewer = cloudViewer (init_cloud_ptr);
//  boost::signals2::connection c = interface->registerCallback (f);

//  interface->start ();

//  while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//  }

//  interface->stop ();


  return (0);
}
