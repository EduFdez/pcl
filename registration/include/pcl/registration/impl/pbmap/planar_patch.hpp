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

#ifndef PCL_PLANAR_PATCH_HPP_
#define PCL_PLANAR_PATCH_HPP_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Archive> void
pcl::pbmap::PlanarPatch<PointT>::serialize (Archive & ar, const unsigned int version)
{
  ar & id_;
  ar & num_obs_;
  ar & semantic_group_;

  ar & label_;
  ar & label_object_;
  ar & label_context_;

  ar & elongation_;
  ar & v_main_direction_;

  ar & centroid_;
  ar & covariance_;
  ar & count_;
  ar & contour_;
  ar & coefficients_;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::NormalizePlaneCoefs ()
{
  float normABC = sqrt(coefficients_[0]*coefficients_[0] + coefficients_[1]*coefficients_[1] + coefficients_[2]*coefficients_[2]);
  if (normABC != 1.f)
  {
    assert(normABC != 0);

    coefficients_[0] /= normABC;
    coefficients_[1] /= normABC;
    coefficients_[2] /= normABC;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::forcePtsLayOnPlane ()
{
  assert(coefficients_[0]*coefficients_[0] + coefficients_[1]*coefficients_[1] + coefficients_[2]*coefficients_[2] == 1.f);

  // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter.
  // The vector of coefficients stores (A,B,C,D)
  for(unsigned i = 0; i < patch_points_->size(); i++)
  {
    double dist = coefficients_[0]*patch_points_->points[i].x + coefficients_[1]*patch_points_->points[i].y + coefficients_[2]*patch_points_->points[i].z + coefficients_[3];
    patch_points_->points[i].x -= coefficients_[0] * dist;
    patch_points_->points[i].y -= coefficients_[1] * dist;
    patch_points_->points[i].z -= coefficients_[2] * dist;
  }
  // Do the same with the points defining the convex hull
  for(unsigned i = 0; i < contour_.size(); i++)
  {
    double dist = coefficients_[0]*contour_[i].x + coefficients_[1]*contour_[i].y + coefficients_[2]*contour_[i].z + coefficients_[3];
    contour_[i].x -= coefficients_[0] * dist;
    contour_[i].y -= coefficients_[1] * dist;
    contour_[i].z -= coefficients_[2] * dist;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::calcElongationAndPpalDir ()
{
  // Covariance matrix of the patch's area distribution. See www.wikihow.com/Sample/Area-of-a-Triangle-Side-Length
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero ();
  float total_area = 0;
  float total_perimeter = 0;
  for(unsigned i = 0; i < contour_.size (); i++)
  {
    unsigned ii = i+1 % contour_.size ();
    // Compute the center of each triangle formed by two adjacent vertex of the patch's convex hull and its centroid
    Eigen::Vector3f centroid;
    centroid[0] = (contour_[i].x + contour_[ii].x + centroid_[0]) / 3;
    centroid[1] = (contour_[i].y + contour_[ii].y + centroid_[1]) / 3;
    centroid[2] = (contour_[i].z + contour_[ii].z + centroid_[2]) / 3;
    float l1 = sqrt ( pow ((contour_[i].x-contour_[ii].x),2) + pow((contour_[i].y-contour_[ii].y),2) + pow((contour_[i].z-contour_[ii].z),2) );
    float l2 = sqrt ( pow ((contour_[i].x-centroid_[0]),2) + pow((contour_[i].y-centroid_[1]),2) + pow((contour_[i].z-centroid_[2]),2) );
    float l3 = sqrt ( pow ((centroid_[0]-contour_[ii].x),2) + pow((centroid_[1]-contour_[ii].y),2) + pow((centroid_[2]-contour_[ii].z),2) );
    float semi_perimeter = (l1 + l2 + l3) / 2;
    float area_tri = sqrt( semi_perimeter * (semi_perimeter-l1) * (semi_perimeter-l2) * (semi_perimeter-l3) );
    total_area += area_tri;
    total_perimeter += l1;
    cov += area_tri * centroid * centroid.transpose ();
  }
  cov /= total_area;
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
  elongation_ = svd.singularValues ()[0] / svd.singularValues ()[1];
  v_main_direction_ = svd.matrixU ().block (0,0,3,1);
}

///////////////////////////////////////////////////////////////////////////////////////////
//template<typename dataType>
template <typename PointT> double
pcl::pbmap::PlanarPatch<PointT>::getHistMeanShift (std::vector<float> &data, float &range_in, double max_range)
{
  size_t size = data.size ();
  std::vector<float> data_temp = data;
  double mean_shift, std_dev_hist;
  pcl::getMeanStd (data, mean_shift, std_dev_hist);

  double shift = 1000;
  //int iteration_counter = 0;
  double convergence = max_range * 0.001;
  while (2*data_temp.size () > size && shift > convergence)
  {
    for (typename std::vector<float>::iterator it=data_temp.begin (); it != data_temp.end (); )
    {
      if (fabs (*it - mean_shift) > std_dev_hist)
        data_temp.erase (it);
      else
        ++it;
    }
    double mean_updated;
    pcl::getMeanStd (data_temp, mean_updated, std_dev_hist);
    shift = fabs (mean_updated - mean_shift);
    mean_shift = mean_updated;

    //++iteration_counter;
  }

  return mean_shift;
  //return static_cast<dataType>(mean_shift);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::computeDominantColour ()
{
    assert( patch_points_->size() > 0); // Check the patch is not empty

    size_t pts_size = patch_points_->size ();
    std::vector<float> r(pts_size), g(pts_size), b(pts_size), intensity(pts_size);

    size_t countPix = 0;
    for (size_t i=0; i < pts_size; i++)
    {
      float sumRGB = (float)patch_points_->points[i].r + patch_points_->points[i].g + patch_points_->points[i].b;
      intensity[i] = sumRGB;
      if(sumRGB != 0)
      {
        r[countPix] = patch_points_->points[i].r / sumRGB;
        g[countPix] = patch_points_->points[i].g / sumRGB;
        b[countPix] = patch_points_->points[i].b / sumRGB;
        ++countPix;
      }
    }
    r.resize (countPix);
    g.resize (countPix);
    b.resize (countPix);
    intensity.resize (countPix);

  v_dominant_rgb_int(0) = getHistMeanShift (r, 1.0, v_dominant_rgb_dev_(0));
  v_dominant_rgb_int(1) = getHistMeanShift (g, 1.0, v_dominant_rgb_dev_(1));
  v_dominant_rgb_int(2) = getHistMeanShift (b, 1.0, v_dominant_rgb_dev_(2));

  v_dominant_rgb_int(4) = 0;
  int count_fringe_05 = 0; // Count the number of pixels that lie in a fringe of +-5% around the dominant colour
  for(unsigned i=0; i < r.size(); i++)
    if(fabs (r[i] - v_dominant_rgb_int(0)) < 0.05 && fabs (g[i] - v_dominant_rgb_int(1)) < 0.05 && fabs (b[i] - v_dominant_rgb_int(2)) < 0.05)
    {
      v_dominant_rgb_int(4) += intensity[i];
      ++count_fringe_05;
    }
  assert(count_fringe_05 > 0); // If this assert gives and error, then there is a bug to fix
  v_dominant_rgb_int(4) /= count_fringe_05; // Count the percentage of pixels that lie in a fringe of +-5% around the dominant colour
  float concentration_05 = static_cast<float>(count_fringe_05) / r.size();
  if(concentration_05 > 0.5) // To accept a dominant colour, we require that 50% of the pixels are in a bandwidth of 5% around the mean-shift
    b_dominant_color_ = true;
  else
    b_dominant_color_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::computeHueHistogram()
{
  float fR, fG, fB;
  float fH, fS, fV;
  const float FLOAT_TO_BYTE = 255.0f;
  const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

  const size_t SIZE_HIST = 74;
  std::vector<int> hist(SIZE_HIST,0);

  for(unsigned i=0; i < patch_points_->size(); i++)
  {
    // Convert from 8-bit integers to floats.
    fR = patch_points_->points[i].r * BYTE_TO_FLOAT;
    fG = patch_points_->points[i].g * BYTE_TO_FLOAT;
    fB = patch_points_->points[i].b * BYTE_TO_FLOAT;
    // Convert from RGB to HSV, using float max_ranges 0.0 to 1.0.
    float fDelta;
    float fMin, fMax;
    int iMax;
    // Get the min and max, but use integer comparisons for slight speedup.
    if (patch_points_->points[i].b < patch_points_->points[i].g) {
      if (patch_points_->points[i].b < patch_points_->points[i].r) {
        fMin = fB;
        if (patch_points_->points[i].r > patch_points_->points[i].g) {
          iMax = patch_points_->points[i].r;
          fMax = fR;
        }
        else {
          iMax = patch_points_->points[i].g;
          fMax = fG;
        }
      }
      else {
        fMin = fR;
        fMax = fG;
        iMax = patch_points_->points[i].g;
      }
    }
    else {
      if (patch_points_->points[i].g < patch_points_->points[i].r) {
        fMin = fG;
        if (patch_points_->points[i].b > patch_points_->points[i].r) {
          fMax = fB;
          iMax = patch_points_->points[i].b;
        }
        else {
          fMax = fR;
          iMax = patch_points_->points[i].r;
        }
      }
      else {
        fMin = fR;
        fMax = fB;
        iMax = patch_points_->points[i].b;
      }
    }
    fDelta = fMax - fMin;
    fV = fMax;	// Value (Brightness).
    if(fV < 0.01)
    {
      hist[72]++; // Histogram has 72 bins for Hue values, and 2 bins more for black and white
      continue;
    }
    else
//    if (iMax != 0)
    {			// Make sure its not pure black.
      fS = fDelta / fMax;		// Saturation.
      if(fS < 0.2)
      {
//        fH = 73; // Histogram has 72 bins for Hue values, and 2 bins more for black and white
        hist[73]++;
        continue;
      }

//      float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
      float ANGLE_TO_UNIT = 12.0f / fDelta;	// 12 bins
      if (iMax == patch_points_->points[i].r) {		// between yellow and magenta.
        fH = (fG - fB) * ANGLE_TO_UNIT;
      }
      else if (iMax == patch_points_->points[i].g) {		// between cyan and yellow.
        fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
      }
      else {				// between magenta and cyan.
        fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
      }
      // Wrap outlier Hues around the circle.
      if (fH < 0.0f)
        fH += 72.0f;
      if (fH >= 72.0f)
        fH -= 72.0f;
    }
    hist[int(fH)]++;
  }
  // Normalize histogram
  float numPixels = 0;
  for (unsigned i=0; i < SIZE_HIST; i++)
    numPixels += hist[i];
  hue_histogram_.resize (SIZE_HIST);
  for (unsigned i=0; i < SIZE_HIST; i++)
    hue_histogram_[i] = hist[i] / numPixels;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::pbmap::PlanarPatch<PointT>::transformAffine (Eigen::Matrix4f &Rt)
{
  // Transform normal and ppal direction
  coefficients_.block(0,0,3,1) = Rt.block (0,0,3,3) * coefficients_.block (0,0,3,1);
  coefficients_[3] = -(coefficients_.block (0,0,3,1). dot (centroid_));
  v_main_direction_ = Rt.block (0,0,3,3) * v_main_direction_;

  // Transform centroid
  centroid_ = Rt.block (0,0,3,3) * centroid_ + Rt.block (0,3,3,1);

  // Transform convex hull points
  for (size_t i = 0; i < contour_.size (); ++i)
  {
    Eigen::Vector3f pt_transf = Rt.block (0,0,3,3) * Eigen::Vector3f(contour_[i].x, contour_[i].y, contour_[i].z) + Rt.block (0,3,3,1);
    contour_[i].x = pt_transf[0]; contour_[i].y = pt_transf[1]; contour_[i].z = pt_transf[2];
  }

  // Transform patch's points
  pcl::transformPointCloud (*patch_points_, *patch_points_, Rt);
}

#endif //#ifndef PCL_PLANAR_PATCH_HPP_
