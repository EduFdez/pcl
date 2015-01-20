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

#include <pcl/registration/pbmap/utils.h>

using namespace pcl::pbmap;

///////////////////////////////////////////////////////////////////////////////////////////
double
getHistMeanShift (std::vector<float> &data,
                  double max_range )
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
