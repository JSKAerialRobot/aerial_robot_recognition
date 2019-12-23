// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#pragma once

#include <boost/array.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace line_extraction
{
  struct CachedData
  {
    std::vector<unsigned int> indices;
    std::vector<double> bearings;
    std::vector<double> cos_bearings;
    std::vector<double> sin_bearings;
  };

  struct RangeData
  {
    std::vector<double> ranges;
    std::vector<double> xs;
    std::vector<double> ys;
  };

  struct Params
  {
    double bearing_var;
    double range_var;
    double least_sq_angle_thresh;
    double least_sq_radius_thresh;
    double max_line_gap;
    double min_line_length;
    double min_range;
    double min_split_dist;
    double outlier_dist;
    unsigned int min_line_points;
  };

  struct PointParams
  {
    std::vector<double> a;
    std::vector<double> ap;
    std::vector<double> app;
    std::vector<double> b;
    std::vector<double> bp;
    std::vector<double> bpp;
    std::vector<double> c;
    std::vector<double> s;
  };

  class Line
  {

  public:
    // Constructor / destructor
    Line(const CachedData&, const RangeData&, const Params&, std::vector<unsigned int>);
    Line(double angle, double radius, const boost::array<double, 4> &covariance,
         const boost::array<double, 2> &start, const boost::array<double, 2> &end,
         const std::vector<unsigned int> &indices);
    ~Line();
    // Get methods for the line parameters
    double                           getAngle() const;
    const boost::array<double, 4>&   getCovariance() const;
    const boost::array<double, 2>&   getEnd() const;
    const std::vector<unsigned int>& getIndices() const;
    double                           getRadius() const;
    const boost::array<double, 2>&   getStart() const;
    // Methods for line fitting
    double       distToPoint(unsigned int);
    void         endpointFit();
    void         leastSqFit();
    double       length() const;
    unsigned int numPoints() const;
    void         projectEndpoints();

  private:
    std::vector<unsigned int> indices_;
    // Data structures
    CachedData c_data_;
    RangeData r_data_;
    Params params_;
    PointParams p_params_;
    // Point variances used for least squares
    std::vector<double> point_scalar_vars_;
    std::vector<boost::array<double, 4> > point_covs_;
    double p_rr_;
    // Line parameters
    double angle_;
    double radius_;
    boost::array<double, 2> start_;
    boost::array<double, 2> end_;
    boost::array<double, 4> covariance_;
    // Methods
    void    angleFromEndpoints();
    void    angleFromLeastSq();
    double  angleIncrement();
    void    calcCovariance();
    void    calcPointCovariances();
    void    calcPointParameters();
    void    calcPointScalarCovariances();
    void    radiusFromEndpoints();
    void    radiusFromLeastSq();
  }; // class Line


  class LineExtraction
  {

  public:
    // Constructor / destructor
    LineExtraction();
    ~LineExtraction();
    // Run
    void extractLines(std::vector<Line>&, bool);
    // Data setting
    void setCachedData(const std::vector<double>&, const std::vector<double>&,
                       const std::vector<double>&, const std::vector<unsigned int>&);
    void setRangeData(const std::vector<double>&);
    // Parameter setting
    void setBearingVariance(double);
    void setRangeVariance(double);
    void setLeastSqAngleThresh(double);
    void setLeastSqRadiusThresh(double);
    void setMaxLineGap(double);
    void setMinLineLength(double);
    void setMinLinePoints(unsigned int);
    void setMinRange(double);
    void setMinSplitDist(double);
    void setOutlierDist(double);

  private:
    // Data structures
    CachedData c_data_;
    RangeData r_data_;
    Params params_;
    // Indices after filtering
    std::vector<unsigned int> filtered_indices_;
    // Line data
    std::vector<Line> lines_;
    // Methods
    double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                      const Eigen::Matrix2d&);
    double distBetweenPoints(unsigned int index_1, unsigned int index_2);
    void   filterClosePoints();
    void   filterOutlierPoints();
    void   filterLines();
    void   mergeLines();
    void   split(const std::vector<unsigned int>&, bool);
  };
}
