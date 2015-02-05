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

#include <boost/thread.hpp>

#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/io/openni2/openni2_device.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
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

#include <pcl/registration/pbmap/pbmap.h>
#include <pcl/registration/pbmap/pbmap_viewer.h>

typedef pcl::PointXYZRGBA PointT;

//pcl::PointCloud<PointT>::ConstPtr _cloud_;
//boost::mutex _cloud_mutex_;

//void
//_cloud_callback (const pcl::PointCloud<PointT>::ConstPtr& cloud)
//{
////  if (!cloud_viewer_->wasStopped ())
////  {
////    _cloud_mutex_.lock ();
////    _cloud_ = cloud;
////    _cloud_mutex_.unlock ();
////  }
//  //FPS_CALC ("cloud callback");
//  boost::mutex::scoped_lock lock (_cloud_mutex_);
//  _cloud_ = cloud;
//}

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class OpenNI2Viewer
{
public:
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  OpenNI2Viewer ()
    : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI2 cloud"))
    , image_viewer_ ()
    , rgb_data_ (0), rgb_data_size_ (0)
  {
  }

  void
  cloud_callback (const CloudConstPtr& cloud)
  {
    FPS_CALC ("cloud callback");
    boost::mutex::scoped_lock lock (cloud_mutex_);
    cloud_ = cloud;
  }

  void
  image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
  {
    FPS_CALC ("image callback");
    boost::mutex::scoped_lock lock (image_mutex_);
    image_ = image;

    if (image->getEncoding () != pcl::io::openni2::Image::RGB)
    {
      if (rgb_data_size_ < image->getWidth () * image->getHeight ())
      {
        if (rgb_data_)
          delete [] rgb_data_;
        rgb_data_size_ = image->getWidth () * image->getHeight ();
        rgb_data_ = new unsigned char [rgb_data_size_ * 3];
      }
      image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
    }
  }

  void
  keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.getKeyCode ())
      cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
    else
      cout << "the special key \'" << event.getKeySym () << "\' was";
    if (event.keyDown ())
      cout << " pressed" << endl;
    else
      cout << " released" << endl;
  }

  void
  mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
  {
    if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
    {
      cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
  }

  /**
  * @brief starts the main loop
  */
  void
  run ()
  {
    cloud_viewer_->registerMouseCallback (&OpenNI2Viewer::mouse_callback, *this);
    cloud_viewer_->registerKeyboardCallback (&OpenNI2Viewer::keyboard_callback, *this);
    cloud_viewer_->setCameraFieldOfView (1.02259994f);
    boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNI2Viewer::cloud_callback, this, _1);
    boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

//    boost::signals2::connection image_connection;
//    if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
//    {
//      image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
//      image_viewer_->registerMouseCallback (&OpenNI2Viewer::mouse_callback, *this);
//      image_viewer_->registerKeyboardCallback (&OpenNI2Viewer::keyboard_callback, *this);
//      boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&OpenNI2Viewer::image_callback, this, _1);
//      image_connection = grabber_.registerCallback (image_cb);
//    }

    bool image_init = false, cloud_init = false;

    grabber_.start ();

    while (!cloud_viewer_->wasStopped ())// && (image_viewer_ && !image_viewer_->wasStopped ()))
    {
      boost::shared_ptr<pcl::io::openni2::Image> image;
      CloudConstPtr cloud;

      cloud_viewer_->spinOnce ();

      // See if we can get a cloud
      if (cloud_mutex_.try_lock ())
      {
        cloud_.swap (cloud);
        cloud_mutex_.unlock ();
      }

      if (cloud)
      {
        FPS_CALC("drawing cloud");

        if (!cloud_init)
        {
          cloud_viewer_->setPosition (0, 0);
          cloud_viewer_->setSize (cloud->width, cloud->height);
          cloud_init = !cloud_init;
        }

        if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
        {
          cloud_viewer_->addPointCloud (cloud, "OpenNICloud");
          cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
          cloud_viewer_->setCameraPosition (
            0,0,0,		// Position
            0,0,1,		// Viewpoint
            0,-1,0);	// Up
        }
      }

//      // See if we can get an image
//      if (image_mutex_.try_lock ())
//      {
//        image_.swap (image);
//        image_mutex_.unlock ();
//      }


//      if (image)
//      {
//        if (!image_init && cloud && cloud->width != 0)
//        {
//          image_viewer_->setPosition (cloud->width, 0);
//          image_viewer_->setSize (cloud->width, cloud->height);
//          image_init = !image_init;
//        }

//        if (image->getEncoding () == pcl::io::openni2::Image::RGB)
//          image_viewer_->addRGBImage ( (const unsigned char*)image->getData (), image->getWidth (), image->getHeight ());
//        else
//          image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
//        image_viewer_->spinOnce ();

//      }
    }

    grabber_.stop ();

    cloud_connection.disconnect ();
//    image_connection.disconnect ();
//    if (rgb_data_)
//      delete[] rgb_data_;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

  pcl::io::OpenNI2Grabber grabber_;
  boost::mutex cloud_mutex_;
  boost::mutex image_mutex_;

  CloudConstPtr cloud_;
  boost::shared_ptr<pcl::io::openni2::Image> image_;
  unsigned char* rgb_data_;
  unsigned rgb_data_size_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PbMapConstruction
{
  //private:
  public:
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    //boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

    //boost::mutex image_mutex_;

    //boost::shared_ptr<pcl::io::openni2::Image> image_;
    //unsigned char* rgb_data_;
    //unsigned rgb_data_size_;

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



};

int
main ()
{
  std::cout << "PbMapConstruction Application\n";

  OpenNI2Viewer openni2_viewer;
  boost::thread openni2_grabber_viewer ( &OpenNI2Viewer::run, &openni2_viewer );
  std::cout << "OpenNI2 grabber and viewer started...\n";

  //pcl::PointCloud<PointT>::Ptr cloud;

  pcl::pbmap::PbMap<PointT> pbmap;

  while (!openni2_viewer.cloud_viewer_->wasStopped ())
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));

  //PbMapConstruction pbmap_maker;


  openni2_grabber_viewer.join ();

  return (0);
}
