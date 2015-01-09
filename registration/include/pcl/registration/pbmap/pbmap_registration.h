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

#ifndef PCL_PBMAP_REGISTRATION_H_
#define PCL_PBMAP_REGISTRATION_H_

#include <pcl/registration/registration.h>
#include <vector>

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
      //template <typename PointSource, typename PointTarget, typename MatScalar = float>
    class PbMapRegistration
    {

    };
  }
}

#endif //#ifndef PCL_PBMAP_REGISTRATION_H_


///*  Plane-based Map (PbMap) library
// *  Construction of plane-based maps and localization in it from RGBD Images.
// *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
// */

//#ifndef __SUBGRAPHMATCHER_H
//#define __SUBGRAPHMATCHER_H

//#include <mrpt/config.h>
//#if MRPT_HAS_PCL

//#include <mrpt/utils/utils_defs.h>
//#include <mrpt/pbmap/link_pragmas.h>

//#include <mrpt/pbmap/heuristicParams.h>
//#include <mrpt/pbmap/PbMap.h>
//#include <mrpt/pbmap/Subgraph.h>

//namespace mrpt {
//namespace pbmap {


//  /*!This class finds the best correspondence between the planes of two subgraphs (i.e. sets of neighbor planes).
//   * It relies on an interpretation tree employing geometric restrictions that are represented as a set of unary and binary constraints.
//   *
//   * \ingroup mrpt_pbmap_grp
//   */
//  class SubgraphMatcher
//  {
//   public:

//    SubgraphMatcher();

//    /*!Check if the two input planes fulfill a set of geometric constraints, and so, if they are candidates to be the same plane.*/
//    bool evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
//    bool evalUnaryConstraints2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
//    bool evalUnaryConstraintsOdometry(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
//    bool evalUnaryConstraintsOdometry2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);

//    /*!Check if the two pair of planes plane1-plane2 ans planeA-planeB fulfill the same geometric relationship, and so,
//    if they are candidates to be the same planes.*/
//    bool evalBinaryConstraints(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);
//    bool evalBinaryConstraintsOdometry(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);

//    /*!List of combinations that have been explored in the interpretation tree.*/  // Cambiar nombre
//    std::vector<std::map<unsigned,unsigned> > alreadyExplored;

//    /*!Find the best combination of planes correspondences given two subgraphs represeting local neighborhoods of planes.*/  // Cambiar nombre o Quitar!
//    void exploreSubgraphTreeR(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);
//    void exploreSubgraphTreeR_Area(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);

//    /*!Set source (current) subgraph.*/
//    void inline setSourceSubgraph(Subgraph &subgSrc){subgraphSrc = &subgSrc;}

//    /*!Set target subgraph.*/
//    void inline setTargetSubgraph(Subgraph &subgTrg){subgraphTrg = &subgTrg;}

//    /*!Returns a list with plane matches from subgraphSrc to subgraphTrg.*/
////    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget);
//    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, const int option=0); // Options are

//    /*!One subgraph to be matched.*/
//    Subgraph *subgraphSrc;

//    /*!The other subgraph to be matched.*/
//    Subgraph *subgraphTrg;

//    int nCheckConditions;

//    int totalUnary;
//    int semanticPair;
//    int rejectSemantic;

//    /*!Return the total area of the matched planes in the frame source.*/
//    float calcAreaMatched(std::map<unsigned,unsigned> &matched_planes);

//    /*!Set of thresholds for PbMap matching.*/
//    config_heuristics configLocaliser;

//   private:

//    /*!List of planes correspondences.*/
//    std::map<unsigned, unsigned> winnerMatch;
//    float areaWinnerMatch;

//    /*!Hash table for unary constraints.*/
//    std::vector<std::vector<int8_t> > hashUnaryConstraints;

//    float calcAreaUnmatched(std::set<unsigned> &unmatched_planes);

//  };

//} } // End of namespaces

//#endif
//#endif
