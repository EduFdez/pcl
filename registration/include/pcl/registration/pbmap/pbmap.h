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

#include <pcl/registration/pbmap/planar_patch.h>

namespace pcl
{
  namespace pbmap
  {

  /** \brief Calculate Bhattacharyya distance between the input histograms
    * \param[in] hist1
    * \param[in] hist2
    */
    double calcBhattacharyyaDist(std::vector<float> &hist1, std::vector<float> &hist2)
    {
        assert(hist1.size() == hist2.size());
        double BhattachDist;
        double BhattachDist_aux = 0.0;
        for(unsigned i=0; i < hist1.size(); i++)
          BhattachDist_aux += sqrt(hist1[i]*hist2[i]);

        BhattachDist = sqrt(1 - BhattachDist_aux);

        return BhattachDist;
    }

    /** A set of parameters to specify thresholds for the registration.*/
    struct config_pbmap
    {
        // [global]
        float color_threshold;
        float intensity_threshold;
        float hue_threshold;

        // [plane_segmentation]
        float dist_threshold; // Maximum distance to the plane between neighbor 3D-points
        float angle_threshold; //  = 0.017453 * 4.0 // Maximum angle between contiguous 3D-points
        float minInliersRate; // Minimum ratio of inliers/image points required

        // [map_construction]
        bool use_color;                   // Add color information to the planes
        float proximity_neighbor_planes;  // Two planar patches are considered neighbors when the closest distance between them is under proximity_neighbor_planes
        //  float max_angle_normals; // (10º) Two planar patches that represent the same surface must have similar normals // QUITAR
        float max_cos_normal;
        float max_dist_center_plane; // Two planar patches that represent the same surface must have their center in the same plane
        float proximity_threshold;  // Two planar patches that represent the same surface must overlap or be nearby
        int   graph_mode;  // This var selects the condition to create edges in the graph, either proximity of planar patches or co-visibility in a single frame

        // [semantics]
        bool inferStructure;    // Infer if the planes correspond to the floor, ceiling or walls
        bool makeClusters; // Should the PbMapMaker cluster the planes according to their co-visibility

        // [localisation]
        bool detect_loopClosure;             // Run PbMapLocaliser in a different threads to detect loop closures or preloaded PbMaps
        string config_localiser;
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
        std::map<unsigned, std::set<unsigned> > connected_patches_;

        /** \brief Point cloud constructed from the input images from where the patches have been extracted.*/
        typename pcl::PointCloud<PointT>::Ptr point_cloud_;

        /** \brief Set of parameters to control the PbMap construction and registration. */
        config_pbmap configPbMap;

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

        /** \brief Tell whether two patches are closer than a given threshold
          * \param[in] patch1
          * \param[in] patch1
          * \param[in] dist_threshold
          */
        bool
        arePatchesNearby(pcl::pbmap::PlanarPatch &patch1, pcl::pbmap::PlanarPatch &patch2, const float dist_threshold);
/*
        void PbMapMaker::updateProximityGraph(Plane &plane, float proximity)
        {
          for(unsigned i=0; i < mPbMap.vPlanes.size(); i++ )
          {
            if(plane.id == mPbMap.vPlanes[i].id)
              continue;

            if(plane.nearbyPlanes.count(mPbMap.vPlanes[i].id))
              continue;

            if(arePlanesNearby(plane, mPbMap.vPlanes[i], proximity) ) // If the planes are closer than proximity (in meters), then mark them as neighbors
            {
              plane.nearbyPlanes.insert(mPbMap.vPlanes[i].id);
              mPbMap.vPlanes[i].nearbyPlanes.insert(plane.id);
            }
          }
        }
*/

        /** \brief Segment the planar patches from the input point cloud
          * \param[in] point_cloud_arg
          * \param[in] pose
          * \param[in] threshold_dist
          * \param[in] threshold_angle
          * \param[in] threshold_inliers
          */
        void
        segmentPatches (const pcl::PointCloud<PointT>::Ptr &point_cloud_arg,
                        const Eigen::Matrix4f & pose,
                        const double threshold_dist,
                        const double threshold_angle,
                        const double threshold_inliers )
        {
          boost::mutex::scoped_lock updateLock(mtx_pbmap_busy);

          unsigned minInliers = threshold_inliers * point_cloud_arg->size();

          #ifdef _VERBOSE
            cout << "detectPlanes in a cloud with " << point_cloud_arg->size() << " points " << minInliers << " minInliers\n";
          #endif

          pcl::PointCloud<PointT>::Ptr point_cloud_arg2(new pcl::PointCloud<PointT>);
          pcl::copyPointCloud(*point_cloud_arg,*point_cloud_arg2);

          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::transformPointCloud(*point_cloud_arg,*alignedCloudPtr,pose);

          { mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
            *mPbMap.globalMapPtr += *alignedCloudPtr;
            // Downsample voxel map's point cloud
            static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
            grid.setLeafSize(0.02,0.02,0.02);
            pcl::PointCloud<pcl::PointXYZRGBA> globalMap;
            grid.setInputCloud (mPbMap.globalMapPtr);
            grid.filter (globalMap);
            mPbMap.globalMapPtr->clear();
            *mPbMap.globalMapPtr = globalMap;
          } // End CS

          pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
          ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
          ne.setMaxDepthChangeFactor (0.02f); // For VGA: 0.02f, 10.0f
          ne.setNormalSmoothingSize (10.0f);
          ne.setDepthDependentSmoothing (true);

          pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
          mps.setMinInliers (minInliers); cout << "Params " << minInliers << " " << threshold_angle << " " << threshold_dist << endl;
          mps.setAngularThreshold (threshold_angle); // (0.017453 * 2.0) // 3 degrees
          mps.setDistanceThreshold (threshold_dist); //2cm

          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
          ne.setInputCloud (point_cloud_arg2);
          ne.compute (*normal_cloud);

        #ifdef _VERBOSE
          double plane_extract_start = pcl::getTime ();
        #endif
          mps.setInputNormals (normal_cloud);
          mps.setInputCloud (point_cloud_arg2);

          std::vector<pcl::PlanarRegion<PointT>, aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
          std::vector<pcl::ModelCoefficients> model_coefficients;
          std::vector<pcl::PointIndices> inlier_indices;
          pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
          std::vector<pcl::PointIndices> label_indices;
          std::vector<pcl::PointIndices> boundary_indices;
          mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

          #ifdef _VERBOSE
            double plane_extract_end = pcl::getTime();
            std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
        //    std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;
            cout << regions.size() << " planes detected\n";
          #endif

          // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
          // in the global reference
          vector<Plane> detectedPlanes;
          for (size_t i = 0; i < regions.size (); i++)
          {
            Plane plane;

            Vector3f centroid = regions[i].getCentroid ();
            plane.v3center = compose(pose, centroid);
            plane.v3normal = pose.block(0,0,3,3) * Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
        //    plane.curvature = regions[i].getCurvature();
        //  assert(plane.v3normal*plane.v3center.transpose() <= 0);
        //    if(plane.v3normal*plane.v3center.transpose() <= 0)
        //      plane.v3normal *= -1;

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
            extract.setInputCloud (point_cloud_arg2);
            extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
            extract.setNegative (false);
            extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud

            static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
            plane_grid.setLeafSize(0.05,0.05,0.05);
            pcl::PointCloud<pcl::PointXYZRGBA> planeCloud;
            plane_grid.setInputCloud (plane.planePointCloudPtr);
            plane_grid.filter (planeCloud);
            plane.planePointCloudPtr->clear();
            pcl::transformPointCloud(planeCloud,*plane.planePointCloudPtr,pose);

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
            contourPtr->points = regions[i].getContour();
            plane_grid.setLeafSize(0.1,0.1,0.1);
            plane_grid.setInputCloud (contourPtr);
            plane_grid.filter (*plane.polygonContourPtr);
        //    plane.contourPtr->points = regions[i].getContour();
        //    pcl::transformPointCloud(*plane.contourPtr,*plane.polygonContourPtr,pose);
            pcl::transformPointCloud(*plane.polygonContourPtr,*contourPtr,pose);
            plane.calcConvexHull(contourPtr);
            plane.computeMassCenterAndArea();
            plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

            #ifdef _VERBOSE
              cout << "Area plane region " << plane.areaVoxels<< " of Chull " << plane.areaHull << " of polygon " << plane.compute2DPolygonalArea() << endl;
            #endif

            // Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)
            bool isSamePlane = false;
            for (size_t j = 0; j < detectedPlanes.size(); j++)
              if( areSamePlane(detectedPlanes[j], plane, configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
              {
                isSamePlane = true;

                mergePlanes(detectedPlanes[j], plane);

                #ifdef _VERBOSE
                  cout << "\tTwo regions support the same plane in the same KeyFrame\n";
                #endif

                break;
              }
            if(!isSamePlane)
              detectedPlanes.push_back(plane);
          }

          #ifdef _VERBOSE
            cout << detectedPlanes.size () << " Planes detected\n";
          #endif

          // Merge detected planes with previous ones if they are the same
          size_t numPrevPlanes = mPbMap.vPlanes.size();
        //  set<unsigned> observedPlanes;
          observedPlanes.clear();
         { mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
          for (size_t i = 0; i < detectedPlanes.size (); i++)
          {
            // Check similarity with previous planes detected
            bool isSamePlane = false;
            vector<Plane>::iterator itPlane = mPbMap.vPlanes.begin();
        //  if(frameQueue.size() != 12)
            for(size_t j = 0; j < numPrevPlanes; j++, itPlane++) // numPrevPlanes
            {
              if( areSamePlane(mPbMap.vPlanes[j], detectedPlanes[i], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
              {
        //        if (j==2 && frameQueue.size() == 12)
        //        {
        //          cout << "Same plane\n";
        //
        ////          ofstream pbm;
        ////          pbm.open("comparePlanes.txt");
        //          {
        //            cout << " ID " << mPbMap.vPlanes[j].id << " obs " << mPbMap.vPlanes[j].numObservations;
        //            cout << " areaVoxels " << mPbMap.vPlanes[j].areaVoxels << " areaVoxels " << mPbMap.vPlanes[j].areaHull;
        //            cout << " ratioXY " << mPbMap.vPlanes[j].elongation << " structure " << mPbMap.vPlanes[j].bFromStructure << " label " << mPbMap.vPlanes[j].label;
        //            cout << "\n normal\n" << mPbMap.vPlanes[j].v3normal << "\n center\n" << mPbMap.vPlanes[j].v3center;
        //            cout << "\n PpalComp\n" << mPbMap.vPlanes[j].v3PpalDir << "\n RGB\n" << mPbMap.vPlanes[j].v3colorNrgb;
        //            cout << "\n Neighbors (" << mPbMap.vPlanes[j].neighborPlanes.size() << "): ";
        //            for(map<unsigned,unsigned>::iterator it=mPbMap.vPlanes[j].neighborPlanes.begin(); it != mPbMap.vPlanes[j].neighborPlanes.end(); it++)
        //              cout << it->first << " ";
        //            cout << "\n CommonObservations: ";
        //            for(map<unsigned,unsigned>::iterator it=mPbMap.vPlanes[j].neighborPlanes.begin(); it != mPbMap.vPlanes[j].neighborPlanes.end(); it++)
        //              cout << it->second << " ";
        //            cout << "\n ConvexHull (" << mPbMap.vPlanes[j].polygonContourPtr->size() << "): \n";
        //            for(unsigned jj=0; jj < mPbMap.vPlanes[j].polygonContourPtr->size(); jj++)
        //              cout << "\t" << mPbMap.vPlanes[j].polygonContourPtr->points[jj].x << " " << mPbMap.vPlanes[j].polygonContourPtr->points[jj].y << " " << mPbMap.vPlanes[j].polygonContourPtr->points[jj].z << endl;
        //            cout << endl;
        //          }
        //          {
        ////            cout << " ID " << detectedPlanes[i].id << " obs " << detectedPlanes[i].numObservations;
        ////            cout << " areaVoxels " << detectedPlanes[i].areaVoxels << " areaVoxels " << detectedPlanes[i].areaHull;
        ////            cout << " ratioXY " << detectedPlanes[i].elongation << " structure " << detectedPlanes[i].bFromStructure << " label " << detectedPlanes[i].label;
        //            cout << "\n normal\n" << detectedPlanes[i].v3normal << "\n center\n" << detectedPlanes[i].v3center;
        ////            cout << "\n PpalComp\n" << detectedPlanes[i].v3PpalDir << "\n RGB\n" << detectedPlanes[i].v3colorNrgb;
        ////            cout << "\n Neighbors (" << detectedPlanes[i].neighborPlanes.size() << "): ";
        ////            for(map<unsigned,unsigned>::iterator it=detectedPlanes[i].neighborPlanes.begin(); it != detectedPlanes[i].neighborPlanes.end(); it++)
        ////              cout << it->first << " ";
        ////            cout << "\n CommonObservations: ";
        ////            for(map<unsigned,unsigned>::iterator it=detectedPlanes[i].neighborPlanes.begin(); it != detectedPlanes[i].neighborPlanes.end(); it++)
        ////              cout << it->second << " ";
        //            cout << "\n ConvexHull (" << detectedPlanes[i].polygonContourPtr->size() << "): \n";
        //            for(unsigned jj=0; jj < detectedPlanes[i].polygonContourPtr->size(); jj++)
        //              cout << "\t" << detectedPlanes[i].polygonContourPtr->points[jj].x << " " << detectedPlanes[i].polygonContourPtr->points[jj].y << " " << detectedPlanes[i].polygonContourPtr->points[jj].z << endl;
        //            cout << endl;
        //          }
        ////          pbm.close();
        //        }

                isSamePlane = true;

                mergePlanes(mPbMap.vPlanes[j], detectedPlanes[i]);

                // Update proximity graph
                checkProximity(mPbMap.vPlanes[j], configPbMap.proximity_neighbor_planes); // Detect neighbors

                #ifdef _VERBOSE
                  cout << "Previous plane " << mPbMap.vPlanes[j].id << " area " << mPbMap.vPlanes[j].areaVoxels<< " of polygon " << mPbMap.vPlanes[j].compute2DPolygonalArea() << endl;
                #endif

                if( observedPlanes.count(mPbMap.vPlanes[j].id) == 0 ) // If this plane has already been observed through a previous partial plane in this same keyframe, then we must not account twice in the observation count
                {
                  mPbMap.vPlanes[j].numObservations++;

                  // Update co-visibility graph
                  for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
                    if(mPbMap.vPlanes[j].neighborPlanes.count(*it))
                    {
                      mPbMap.vPlanes[j].neighborPlanes[*it]++;
                      mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id]++; // j = mPbMap.vPlanes[j]
                    }
                    else
                    {
                      mPbMap.vPlanes[j].neighborPlanes[*it] = 1;
                      mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id] = 1;
                    }

                  observedPlanes.insert(mPbMap.vPlanes[j].id);
                }

                #ifdef _VERBOSE
                  cout << "Same plane\n";
                #endif

                itPlane++;
                for(size_t k = j+1; k < numPrevPlanes; k++, itPlane++) // numPrevPlanes
                  if( areSamePlane(mPbMap.vPlanes[j], mPbMap.vPlanes[k], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
                  {
                    mergePlanes(mPbMap.vPlanes[j], mPbMap.vPlanes[k]);

                    mPbMap.vPlanes[j].numObservations += mPbMap.vPlanes[k].numObservations;

                    for(set<unsigned>::iterator it = mPbMap.vPlanes[k].nearbyPlanes.begin(); it != mPbMap.vPlanes[k].nearbyPlanes.end(); it++)
                      mPbMap.vPlanes[*it].nearbyPlanes.erase(mPbMap.vPlanes[k].id);

                    for(map<unsigned,unsigned>::iterator it = mPbMap.vPlanes[k].neighborPlanes.begin(); it != mPbMap.vPlanes[k].neighborPlanes.end(); it++)
                      mPbMap.vPlanes[it->first].neighborPlanes.erase(mPbMap.vPlanes[k].id);

                    // Update plane index
                    for(size_t h = k+1; h < numPrevPlanes; h++)
                      --mPbMap.vPlanes[h].id;

                    for(size_t h = 0; h < numPrevPlanes; h++)
                    {
                      if(k==h)
                        continue;

                      for(set<unsigned>::iterator it = mPbMap.vPlanes[h].nearbyPlanes.begin(); it != mPbMap.vPlanes[h].nearbyPlanes.end(); it++)
                        if(*it > mPbMap.vPlanes[k].id)
                        {
                          mPbMap.vPlanes[h].nearbyPlanes.insert(*it-1);
                          mPbMap.vPlanes[h].nearbyPlanes.erase(*it);
                        }

                      for(map<unsigned,unsigned>::iterator it = mPbMap.vPlanes[h].neighborPlanes.begin(); it != mPbMap.vPlanes[h].neighborPlanes.end(); it++)
                        if(it->first > mPbMap.vPlanes[k].id)
                        {
                          mPbMap.vPlanes[h].neighborPlanes[it->first-1] = it->second;
                          mPbMap.vPlanes[h].neighborPlanes.erase(it);
                        }
                    }

                    mPbMap.vPlanes.erase(itPlane);
                    --numPrevPlanes;

                    #ifdef _VERBOSE
                      cout << "MERGE TWO PREVIOUS PLANES WHEREBY THE INCORPORATION OF A NEW REGION \n";
                    #endif
                  }

                break;
              }
            }
            if(!isSamePlane)
            {
              detectedPlanes[i].id = mPbMap.vPlanes.size();
              detectedPlanes[i].numObservations = 1;
              detectedPlanes[i].bFullExtent = false;
              detectedPlanes[i].nFramesAreaIsStable = 0;
        //      detectedPlanes[i].calcMainColor(calcMainColor();
              if(configPbMap.makeClusters)
              {
                detectedPlanes[i].semanticGroup = clusterize->currentSemanticGroup;
                clusterize->groups[clusterize->currentSemanticGroup].push_back(detectedPlanes[i].id);
              }

              #ifdef _VERBOSE
                cout << "New plane " << detectedPlanes[i].id << " area " << detectedPlanes[i].areaVoxels<< " of polygon " << detectedPlanes[i].areaHull << endl;
              #endif

              // Update proximity graph
              checkProximity(detectedPlanes[i], configPbMap.proximity_neighbor_planes);  // Detect neighbors with max separation of 1.0 meters

              // Update co-visibility graph
              for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
              {
                detectedPlanes[i].neighborPlanes[*it] = 1;
                mPbMap.vPlanes[*it].neighborPlanes[detectedPlanes[i].id] = 1;
              }

              observedPlanes.insert(detectedPlanes[i].id);

              mPbMap.vPlanes.push_back(detectedPlanes[i]);
            }
          }
         }

        //  if(frameQueue.size() == 12)
        //   cout << "Same plane? " << areSamePlane(mPbMap.vPlanes[2], mPbMap.vPlanes[9], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) << endl;

          #ifdef _VERBOSE
            cout << "\n\tobservedPlanes: ";
            cout << observedPlanes.size () << " Planes observed\n";
            for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
              cout << *it << " ";
            cout << endl;
          #endif

            // For all observed planes
            for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
            {
              Plane &observedPlane = mPbMap.vPlanes[*it];

              // Calculate principal direction
              observedPlane.calcElongationAndPpalDir();

        ////cout << "Update color\n";
              // Update color
              observedPlane.calcMainColor();

            #ifdef _VERBOSE
              cout << "Plane " << observedPlane.id << " color\n" << observedPlane.v3colorNrgb << endl;
            #endif

              // Infer knowledge from the planes (e.g. do these planes represent the floor, walls, etc.)
              if(configPbMap.inferStructure)
                mpPlaneInferInfo->searchTheFloor(pose, observedPlane);
            } // End for obsevedPlanes
        //cout << "Updated planes\n";

        //    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
        //    {
        //      Plane &observedPlane = mPbMap.vPlanes[*it];
        //      watchProperties(observedPlanes, observedPlane); // Color paper
        //    }

            // Search the floor plane
            if(mPbMap.FloorPlane != -1) // Verify that the observed planes centers are above the floor
            {
              #ifdef _VERBOSE
                cout << "Verify that the observed planes centers are above the floor\n";
              #endif

              for(set<unsigned>::reverse_iterator it = observedPlanes.rbegin(); it != observedPlanes.rend(); it++)
              {
                if(static_cast<int>(*it) == mPbMap.FloorPlane)
                  continue;
                if( mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(mPbMap.vPlanes[*it].v3center - mPbMap.vPlanes[mPbMap.FloorPlane].v3center) < -0.1 )
                {
                  if(mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(mPbMap.vPlanes[*it].v3normal) > 0.99) //(cos 8.1º = 0.99)
                  {
                    mPbMap.vPlanes[*it].label = "Floor";
                    mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
                    mPbMap.FloorPlane = *it;
                  }
                  else
                  {
        //            assert(false);
                    mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
                    mPbMap.FloorPlane = -1;
                    break;
                  }
                }
              }
            }

          if(configPbMap.detect_loopClosure)
            for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
            {
        //    cout << "insert planes\n";
              if(mpPbMapLocaliser->vQueueObservedPlanes.size() < 10)
                mpPbMapLocaliser->vQueueObservedPlanes.push_back(*it);
            }

            #ifdef _VERBOSE
              cout << "DetectedPlanesCloud finished\n";
            #endif

          updateLock.unlock();
        }

        /*!Check if the the input plane is the same than this plane for some given angle and distance thresholds.
         * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
        bool PbMapMaker::areSamePlane(Plane &plane1, Plane &plane2, const float &costhreshold_angle, const float &threshold_dist, const float &proxThreshold)
        {
          // Check that both planes have similar orientation
          if( plane1.v3normal.dot(plane2.v3normal) < costhreshold_angle )
            return false;
        //  if(plane1.id == 2)
        //    cout << "normal " << plane1.v3normal.dot(plane2.v3normal) << " " << costhreshold_angle << endl;

          // Check the normal distance of the planes centers using their average normal
          float dist_normal = plane1.v3normal.dot(plane2.v3center - plane1.v3center);
        //  if(fabs(dist_normal) > threshold_dist ) // Avoid matching different parallel planes
        //    return false;
          float thres_max_dist = max(threshold_dist, threshold_dist*2*(plane2.v3center - plane1.v3center).norm());
          if(fabs(dist_normal) > thres_max_dist ) // Avoid matching different parallel planes
            return false;
        //  if(plane1.id == 2)
        //  {
        //    cout << "dist_normal " << dist_normal << " " << thres_max_dist << endl;
        //    if(arePlanesNearby(plane1, plane2, proxThreshold))
        //      cout << "planes rearby" << endl;
        //  }

          // Once we know that the planes are almost coincident (parallelism and position)
          // we check that the distance between the planes is not too big
          return arePlanesNearby(plane1, plane2, proxThreshold);
        }

        void PbMapMaker::mergePlanes(Plane &updatePlane, Plane &discardPlane)
        {
          // Update normal and center
          updatePlane.v3normal = updatePlane.areaVoxels*updatePlane.v3normal + discardPlane.areaVoxels*discardPlane.v3normal;
          updatePlane.v3normal = updatePlane.v3normal / (updatePlane.v3normal).norm();
          // Update point inliers
        //  *updatePlane.polygonContourPtr += *discardPlane.polygonContourPtr; // Merge polygon points
          *updatePlane.planePointCloudPtr += *discardPlane.planePointCloudPtr; // Add the points of the new detection and perform a voxel grid

          // Filter the points of the patch with a voxel-grid. This points are used only for visualization
          static pcl::VoxelGrid<pcl::PointXYZRGBA> merge_grid;
          merge_grid.setLeafSize(0.05,0.05,0.05);
          pcl::PointCloud<pcl::PointXYZRGBA> mergeCloud;
          merge_grid.setInputCloud (updatePlane.planePointCloudPtr);
          merge_grid.filter (mergeCloud);
          updatePlane.planePointCloudPtr->clear();
          *updatePlane.planePointCloudPtr = mergeCloud;

        //  if(configPbMap.use_color)
        //    updatePlane.calcMainColor();

          *discardPlane.polygonContourPtr += *updatePlane.polygonContourPtr;
          updatePlane.calcConvexHull(discardPlane.polygonContourPtr);
          updatePlane.computeMassCenterAndArea();

          // Move the points to fulfill the plane equation
          updatePlane.forcePtsLayOnPlane();

          // Update area
          double area_recalc = updatePlane.planePointCloudPtr->size() * 0.0025;
          mpPlaneInferInfo->isFullExtent(updatePlane, area_recalc);
          updatePlane.areaVoxels= updatePlane.planePointCloudPtr->size() * 0.0025;

        }

    };

  }
}

#endif //#ifndef PCL_PBMAP_H_
