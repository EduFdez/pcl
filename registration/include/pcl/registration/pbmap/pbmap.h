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

//#define _VERBOSE

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
#include <boost/thread/mutex.hpp>

#include <pcl/registration/pbmap/planar_patch.h>

namespace pcl
{
  namespace pbmap
  {

    /** A set of parameters to specify thresholds for the registration.*/
    struct configPbMap
    {
        // [global]
        float color_threshold;
        float intensity_threshold;
        float hue_threshold;

        // [plane_segmentation]
        float threshold_dist; // Maximum distance to the plane between neighbor 3D-points
        float angle_threshold; //  = 0.017453 * 4.0 // Maximum angle between contiguous 3D-points
        float min_inliersRate; // Minimum ratio of inliers/image points required

        // [map_construction]
        bool use_color;                   // Add color information to the planes
        float proximity_neighbor_planes_;  // Two planar patches are considered neighbors when the closest distance between them is under proximity_neighbor_planes_
        //  float max_angle_normals; // (10º) Two planar patches that represent the same surface must have similar normals // QUITAR
        float max_cos_normal_;
        float max_dist_center_plane_; // Two planar patches that represent the same surface must have their center in the same plane
        float proximity_threshold_;  // Two planar patches that represent the same surface must overlap or be nearby
        int   graph_mode;  // This var selects the condition to create edges in the graph, either proximity of planar patches or co-visibility in a single frame

        // [semantics]
        bool inferStructure;    // Infer if the planes correspond to the floor, ceiling or walls
        bool makeClusters; // Should the PbMapMaker cluster the planes according to their co-visibility

        // [localisation]
        bool detect_loopClosure;             // Run PbMapLocaliser in a different threads to detect loop closures or preloaded PbMaps
        std::string config_localiser;
    };

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
        std::map<unsigned, std::map<unsigned,unsigned> > connected_patches_;

        /** \brief Point cloud constructed from the input images from where the patches have been extracted.*/
        typename pcl::PointCloud<PointT>::Ptr point_cloud_;

        /** Label to store a semantic attribute of the PbMap (e.g. the kind of room: office, kitchen, etc.) */
        std::string label_;

        /** Floor plane index. This is especially usefull in robotic applications where we can guess where is the floor, */
        int floor_plane_;

        /** \brief Set of parameters to control the PbMap construction and registration. */
        configPbMap config_pbmap_;

        friend class boost::serialization::access;

        /** \brief PlanarPatch serialization
          * \param[in] ar destination archive
          * \param[in] version number
          */
        template<class Archive> void
        serialize (Archive & ar, const unsigned int version);

      public:

        typedef boost::shared_ptr<PbMap> Ptr;
        typedef boost::shared_ptr<const PbMap> ConstPtr;

        /** \brief Mutual exception. */
        boost::mutex pbmap_mutex_;

        /** \brief Empty constructor. */
        PbMap() :
            point_cloud_ ( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
            floor_plane_(-1)
        {
        }

        /** \brief Merge this PbMap with
          * \param[in] pbmap input PbMap, which has the coordinate system given by
          * \param[in] Rt affine transformation
          */
        void
        Merge(PbMap &pbm, const Eigen::Matrix4f &Rt);

/*
        void
        updateProximityGraph(Plane &plane, float proximity)
        {
          for(unsigned i=0; i < patches_.size(); i++ )
          {
            if(patch.id == patches_[i].id)
              continue;

            if(patch.nearbyPlanes.count(patches_[i].id))
              continue;

            if(arePlanesNearby(plane, patches_[i], proximity) ) // If the planes are closer than proximity (in meters), then mark them as neighbors
            {
              patch.nearbyPlanes.insert(patches_[i].id);
              patches_[i].nearbyPlanes.insert(patch.id);
            }
          }
        }
*/

//        /** \brief Segment planar patches from the input point cloud and add them to the current PbMap according to their relative pose
//          * \param[in] point_cloud_arg input point cloud
//          * \param[in] pose of the input cloud with respect to the current PbMap
//          * \param[in] threshold_dist for segmentation
//          * \param[in] threshold_angle for segmentation
//          * \param[in] threshold_inliers for segmentation
//          */
//        void
//        getNewPatches ( const pcl::PointCloud<PointT>::Ptr &point_cloud_arg,
//                        const Eigen::Matrix4f & pose,
//                        const double threshold_dist,
//                        const double threshold_angle,
//                        const double threshold_inliers )
//        {
//          boost::mutex::scoped_lock updateLock(mtx_pbmap_busy);

//          unsigned min_inliers = threshold_inliers * point_cloud_arg->size ();

//          #ifdef _VERBOSE
//            std::cout << "detectPlanes in a cloud with " << point_cloud_arg->size () << " points " << min_inliers << " min_inliers\n";
//          #endif

//          pcl::PointCloud<PointT>::Ptr point_cloud_aux (new pcl::PointCloud<PointT>);
//          pcl::copyPointCloud (*point_cloud_arg, *point_cloud_aux);

//          pcl::PointCloud<PointT>::Ptr aligned_cloud (new pcl::PointCloud<PointT>);
//          pcl::transformPointCloud (*point_cloud_arg, *aligned_cloud, pose);

//          { // Should we have a critical section here?
//            *point_cloud_ += *aligned_cloud;
//            // Downsample voxel map's point cloud
//            static pcl::VoxelGrid<PointT> grid;
//            grid.setLeafSize (0.02,0.02,0.02);
//            pcl::PointCloud<PointT> cloud_merge;
//            grid.setInputCloud (point_cloud_);
//            grid.filter (cloud_merge);
//            point_cloud_->clear ();
//            *point_cloud_ = cloud_merge;
//          } // End CS

//          pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
//          ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
//          ne.setMaxDepthChangeFactor (0.02f); // For VGA: 0.02f, 10.0f are recommended
//          ne.setNormalSmoothingSize (10.0f);
//          ne.setDepthDependentSmoothing (true);

//          pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
//          mps.setmin_inliers (min_inliers); // std::cout << "Params " << min_inliers << " " << threshold_angle << " " << threshold_dist << std::endl;
//          mps.setAngularThreshold (threshold_angle); // (0.017453 * 2.0) // 3 degrees
//          mps.setDistanceThreshold (threshold_dist); //2cm

//          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
//          ne.setInputCloud (point_cloud_aux);
//          ne.compute (*normal_cloud);

//        #ifdef _VERBOSE
//          double plane_extract_start = pcl::getTime ();
//        #endif

//          mps.setInputNormals (normal_cloud);
//          mps.setInputCloud (point_cloud_aux);

//          std::vector<pcl::PlanarRegion<PointT>, aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//          std::vector<pcl::ModelCoefficients> model_coefficients;
//          std::vector<pcl::PointIndices> inlier_indices;
//          pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
//          std::vector<pcl::PointIndices> label_indices;
//          std::vector<pcl::PointIndices> boundary_indices;
//          mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

//          #ifdef _VERBOSE
//            double plane_extract_end = pcl::getTime ();
//            std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
//            std::cout << regions.size () << " planes detected\n";
//          #endif

//          // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
//          // in the global reference
//          std::vector<PlanarPatch> new_patches;
//          for (size_t i = 0; i < regions.size (); i++)
//          {
//            PlanarPatch patch (regions[i]);

//            // Extract the planar inliers from the input cloud
//            pcl::ExtractIndices<PointT> extract;
//            extract.setInputCloud (point_cloud_aux);
//            extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
//            extract.setNegative (false);
//            extract.filter (*patch.patch_points_);    // Write the planar point cloud

//            static pcl::VoxelGrid<PointT> plane_grid;
//            plane_grid.setLeafSize (0.05,0.05,0.05);
//            pcl::PointCloud<PointT> patch_cloud;
//            plane_grid.setInputCloud (patch.patch_points_);
//            plane_grid.filter (patch_cloud);
//            patch.patch_points_->clear ();
//            pcl::transformPointCloud (patch_cloud, *patch.patch_points_, pose);

////            pcl::PointCloud<PointT>::Ptr contourPtr(new pcl::PointCloud<PointT>);
////            contourPtr->points = regions[i].getContour();
////            plane_grid.setLeafSize(0.1,0.1,0.1);
////            plane_grid.setInputCloud (contourPtr);
////            plane_grid.filter (*patch.polygonContourPtr);
////        //    patch.contourPtr->points = regions[i].getContour();
////        //    pcl::transformPointCloud(*patch.contourPtr,*patch.polygonContourPtr,pose);
////            pcl::transformPointCloud(*patch.polygonContourPtr,*contourPtr,pose);
////            patch.calcConvexHull(contourPtr);
////            patch.computeMassCenterAndArea();
////            patch.getArea ()= patch.patch_points_->size() * 0.0025;

//            patch.refineConvexHull ();
//            patch.computeCentroidAndArea ();

//            #ifdef _VERBOSE
////              std::cout << "Area " << patch.getArea () << " of polygon " << patch.compute2DPolygonalArea() << std::endl;
//              std::cout << "Area " << patch.getArea () << std::endl;
//            #endif

//            // "Double" Check whether this patch corresponds to a previous patch in the PbMap (this situation may happen when there exists a small discontinuity in the observation)
//            bool is_same_patch = false;
//            for (size_t j = 0; j < new_patches.size (); j++)
//              if( is_same_surface(new_patches[j], plane, config_pbmap_.max_cos_normal_, config_pbmap_.max_dist_center_plane_, config_pbmap_.proximity_threshold_) ) // The planes are merged if they are the same
//              {
//                is_same_patch = true;

//                mergePlanes (new_patches[j], plane);

//                #ifdef _VERBOSE
//                  std::cout << "\tTwo regions support the same plane in the same KeyFrame\n";
//                #endif

//                break;
//              }
//            if(!is_same_patch)
//              new_patches.push_back(patch);
//          }

//          #ifdef _VERBOSE
//            std::cout << new_patches.size () << " Planes detected\n";
//          #endif

//          // Merge detected planes with previous ones if they are the same
//          size_t num_prev_patches = patches_.size ();
//          observed_patches.clear ();
//          { //boost::mutex::scoped_lock updateLock(mtx_pbmap_visualization);
//          for (size_t i = 0; i < new_patches.size (); i++)
//          {
//            // Check similarity with previous planes detected
//            bool is_same_patch = false;
//            vector<Plane>::iterator it_patch = patches_.begin ();
//            for (size_t j = 0; j < num_prev_patches; j++, it_patch++) // num_prev_patches
//            {
//              if ( is_same_surface (patches_[j], new_patches[i], config_pbmap_.max_cos_normal_, config_pbmap_.max_dist_center_plane_, config_pbmap_.proximity_threshold_) ) // The planes are merged if they are the same
//              {
//                is_same_patch = true;

//                mergePlanes (patches_[j], new_patches[i]);

//                updateProximityGraph (patches_[j], config_pbmap_.proximity_neighbor_planes_); // Detect neighbors

//                #ifdef _VERBOSE
//                  std::cout << "Previous plane " << j << " area " << patches_[j].getArea () /*<< " of polygon " << patches_[j].compute2DPolygonalArea()*/ << std::endl;
//                #endif

//                if( observed_patches.count (patches_[j].id) == 0 ) // If this plane has already been observed through a previous partial plane in this same keyframe, then we must not account twice in the observation count
//                {
//                  patches_[j].numObservations++;

//                  // Update co-visibility graph
//                  for (std::set<unsigned>::iterator it_patch = observed_patches.begin (); it_patch != observed_patches.end (); it_patch++)
//                    if (connected_patches_[j].count (*it_patch) )
//                    {
//                      connected_patches_[j][*it_patch]++;
//                      connected_patches_[*it_patch][i]++;
//                    }
//                    else
//                    {
//                      connected_patches_[j][*it_patch] = 1;
//                      connected_patches_[*it_patch][i] = 1;
//                    }

//                  //observed_patches.insert(patches_[j].id);
//                  observed_patches.insert (j);
//                }

//                #ifdef _VERBOSE
//                  std::cout << "Same plane\n";
//                #endif

//                ++it_patch;
//                for (size_t k = j+1; k < num_prev_patches; k++, it_patch++) // num_prev_patches
//                  if ( is_same_surface (patches_[j], patches_[k], config_pbmap_.max_cos_normal_, config_pbmap_.max_dist_center_plane_, config_pbmap_.proximity_threshold_) ) // The planes are merged if they are the same
//                  {
//                    mergePlanes (patches_[j], patches_[k]);

//                    patches_[j].getObservations () += patches_[k].getObservations ();

////                    for (std::set<unsigned>::iterator it = patches_[k].nearbyPlanes.begin(); it != patches_[k].nearbyPlanes.end(); it++)
////                      patches_[*it].nearbyPlanes.erase(patches_[k].id);

//                    connected_patches_.erase (k);
//                    for ( std::map<unsigned,std::map<unsigned,unsigned> >::iterator it_connection = connected_patches_.begin(); it_connection != connected_patches_.end(); it_connection++)
//                      it_connection->second.erase (k);

////                    // Update plane index
////                    for(size_t h = k+1; h < num_prev_patches; h++)
////                      --patches_[h].id;

//                    for(size_t h = 0; h < num_prev_patches; h++)
//                    {
//                      if(k==h)
//                        continue;

////                      for(set<unsigned>::iterator it = patches_[h].nearbyPlanes.begin(); it != patches_[h].nearbyPlanes.end(); it++)
////                        if(*it > patches_[k].id)
////                        {
////                          patches_[h].nearbyPlanes.insert(*it-1);
////                          patches_[h].nearbyPlanes.erase(*it);
////                        }

//                      for ( std::map<unsigned,std::map<unsigned,unsigned> >::iterator it_connection = connected_patches_.begin(); it_connection != connected_patches_.end(); it_connection++)
//                          for (std::map<unsigned,unsigned>::iterator it_connection2 = it_connection->begin(); it_connection2 != it_connection->end(); it_connection2++)
//                            if (it_connection2->first > k)
//                            {
//                              it_connection->second[it_connection2->first-1] = it_connection2->second;
//                              it_connection->second.erase(it_connection2);
//                            }
//                    }

//                    patches_.erase(it_patch);
//                    --num_prev_patches;

//                    #ifdef _VERBOSE
//                      std::cout << "MERGE TWO PREVIOUS PLANES WHEREBY THE INCORPORATION OF A NEW REGION \n";
//                    #endif
//                  }

//                break;
//              }
//            }
//            if (!is_same_patch)
//            {
////              new_patches[i].id = patches_.size();
//              new_patches[i].getObservations () = 1;
////              new_patches[i].bFullExtent = false;
////              new_patches[i].nFramesAreaIsStable = 0;
//        //      new_patches[i].calcMainColor(calcMainColor();

//              #ifdef _VERBOSE
//                std::cout << "New plane " << i << " area " << new_patches[i].getArea () << std::endl;
//              #endif

//              // Update proximity graph
//              updateProximityGraph (new_patches[i], config_pbmap_.proximity_neighbor_planes_);  // Detect neighbors with max separation of 1.0 meters

//              // Update co-visibility graph
//              for(std::set<unsigned>::iterator it_patch = observed_patches.begin(); it_patch != observed_patches.end(); it_patch++)
//              {
//                connected_patches_[i][*it_patch] = 1;
//                connected_patches_[*it_patch][i] = 1;
//              }

////              observed_patches.insert (new_patches[i].id);
//              observed_patches.insert (patches_.size ());

//              patches_.push_back (new_patches[i]);
//            }
//          }
//         }

//        //  if(frameQueue.size() == 12)
//        //   cout << "Same plane? " << is_same_surface(patches_[2], patches_[9], config_pbmap_.max_cos_normal_, config_pbmap_.max_dist_center_plane_, config_pbmap_.proximity_threshold_) << endl;

//          #ifdef _VERBOSE
//            std::cout << "\n\tobserved_patches: ";
//            std::cout << observed_patches.size () << " Planes observed\n";
//            for(std::set<unsigned>::iterator it = observed_patches.begin(); it != observed_patches.end(); it++)
//              std::cout << *it << " ";
//            std::cout << std::endl;
//          #endif

////            // For all observed planes
////            for(std::set<unsigned>::iterator it = observed_patches.begin(); it != observed_patches.end(); it++)
////            {
////              Plane &observedPlane = patches_[*it];

////              // Calculate principal direction
////              observedpatch.calcElongationAndPpalDir();

////        ////cout << "Update color\n";
////              // Update color
////              observedpatch.calcMainColor();

////            #ifdef _VERBOSE
////              cout << "Plane " << observedpatch.id << " color\n" << observedpatch.v3colorNrgb << endl;
////            #endif

////              // Infer knowledge from the planes (e.g. do these planes represent the floor, walls, etc.)
////              if(config_pbmap_.inferStructure)
////                mpPlaneInferInfo->searchTheFloor(pose, observedPlane);
////            } // End for obsevedPlanes
////        //cout << "Updated planes\n";

////        //    for(set<unsigned>::iterator it = observed_patches.begin(); it != observed_patches.end(); it++)
////        //    {
////        //      Plane &observedPlane = patches_[*it];
////        //      watchProperties(observed_patches, observedPlane); // Color paper
////        //    }

////            // Search the floor plane
////            if (floor_plane_ != -1) // Verify that the observed planes centers are above the floor
////            {
////              #ifdef _VERBOSE
////                std::cout << "Verify that the observed planes centers are above the floor\n";
////              #endif

////              for (std::set<unsigned>::reverse_iterator it = observed_patches.rbegin(); it != observed_patches.rend(); it++)
////              {
////                if(static_cast<int>(*it) == floor_plane_)
////                  continue;
////                if( patches_[floor_plane_].getNormal ().dot(patches_[*it].getCentroid () - patches_[floor_plane_].getCentroid ()) < -0.1 )
////                {
////                  if(patches_[floor_plane_].getNormal ().dot(patches_[*it].getNormal ()) > 0.99) //(cos 8.1º = 0.99)
////                  {
////                    patches_[*it].label = "Floor";
////                    patches_[floor_plane_].label = "";
////                    floor_plane_ = *it;
////                  }
////                  else
////                  {
////        //            assert(false);
////                    patches_[floor_plane_].label = "";
////                    floor_plane_ = -1;
////                    break;
////                  }
////                }
////              }
////            }

////          if (config_pbmap_.detect_loopClosure)
////            for (std::set<unsigned>::iterator it = observed_patches.begin(); it != observed_patches.end(); it++)
////            {
////              if(mpPbMapLocaliser->vQueueobserved_patches.size() < 10)
////                mpPbMapLocaliser->vQueueobserved_patches.push_back(*it);
////            }

//            #ifdef _VERBOSE
//              std::cout << "new_patchesCloud finished\n";
//            #endif

//        } // End getNewPatches ()

//        /** \brief Check if the two input patches correspond to the same surface according to the input thresholds on orientation and proximity
//          * \param[in] point_cloud_arg input point cloud
//          * \param[in] pose of the input cloud with respect to the current PbMap
//          * \param[in] threshold_dist for segmentation
//          * \param[in] threshold_angle for segmentation
//          * \param[in] threshold_inliers for segmentation
//          */
//        /*!Check if the the input plane is the same than this plane for some given angle and distance thresholds.
//         * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
//        static bool
//        is_same_surface (const pcl::pbmap::PlanarPatch & patch1,
//                         const pcl::pbmap::PlanarPatch & patch2,
//                         const float & threshold_angle_cos,
//                         const float & threshold_dist,
//                         const float & threshold_proximity )
//        {
//          // Check that both planes have similar orientation
//          if ( patch1.getNormal ().dot (patch2.getNormal ()) < threshold_angle_cos )
//            return false;

//          // Check the normal distance of the planes centers using their average normal`
//          Eigen::Vector3f dist_centroids = patch2.getCentroid () - patch1.getCentroid ();
////          if(fabs(dist_centroids) > 10 ) // Avoid matching planes which are fare away
////            return false;
//          float dist_normal = patch1.getNormal ().dot( dist_centroids / dist_centroids.norm () );
//        //  if(fabs(dist_normal) > threshold_dist ) // Avoid matching different parallel planes
//        //    return false;
//          float thres_max_dist = max(threshold_dist, threshold_dist*2*dist_centroids.norm ());
//          if(fabs(dist_normal) > thres_max_dist ) // Avoid matching different parallel planes
//            return false;

//          // Once we know that the planes are almost coincident (parallelism and position)
//          // we check that the distance between the planes is not too big
//          return arePlanesNearby(patch1, patch2, threshold_proximity);
//        }

    };

  }
}

#endif //#ifndef PCL_PBMAP_H_
