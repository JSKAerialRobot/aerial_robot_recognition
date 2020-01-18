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

#include <aerial_robot_perception/line_extraction.h>

namespace line_extraction
{
  double pi_to_pi(double angle)
  {
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
      angle -= 2 * M_PI;
    return angle;
  }

  Line::Line(const CachedData &c_data, const RangeData &r_data, const Params &params, 
             std::vector<unsigned int> indices):
    c_data_(c_data),
    r_data_(r_data),
    params_(params),
    indices_(indices)
  {
  }

  Line::Line(double angle, double radius, const boost::array<double, 4> &covariance,
             const boost::array<double, 2> &start, const boost::array<double, 2> &end,
             const std::vector<unsigned int> &indices):
    angle_(angle),
    radius_(radius),
    covariance_(covariance),
    start_(start),
    end_(end),
    indices_(indices)
  {
  }

  Line::~Line()
  {
  }

  double Line::getAngle() const
  {
    return angle_;
  }

  const boost::array<double, 4>& Line::getCovariance() const
  {
    return covariance_;
  }

  const boost::array<double, 2>& Line::getEnd() const
  {
    return end_;
  }

  const std::vector<unsigned int>& Line::getIndices() const
  {
    return indices_;
  }

  double Line::getRadius() const
  {
    return radius_;
  }

  const boost::array<double, 2>& Line::getStart() const
  {
    return start_;
  }

  double Line::distToPoint(unsigned int index)
  {
    double p_rad = sqrt(pow(r_data_.xs[index], 2) + pow(r_data_.ys[index], 2));
    double p_ang = atan2(r_data_.ys[index], r_data_.xs[index]);
    return fabs(p_rad * cos(p_ang - angle_) - radius_);
  }

  double Line::length() const
  {
    return sqrt(pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2));
  }

  unsigned int Line::numPoints() const
  {
    return indices_.size();  
  }

  void Line::projectEndpoints()
  {
    double s = -1.0 / tan(angle_);
    double b = radius_ / sin(angle_);
    double x = start_[0];
    double y = start_[1];
    start_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
    start_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
    x = end_[0];
    y = end_[1];
    end_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
    end_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
  }

  void Line::endpointFit()
  {
    start_[0] = r_data_.xs[indices_[0]]; 
    start_[1] = r_data_.ys[indices_[0]]; 
    end_[0] = r_data_.xs[indices_.back()]; 
    end_[1] = r_data_.ys[indices_.back()]; 
    angleFromEndpoints();
    radiusFromEndpoints();
  }

  void Line::angleFromEndpoints()
  {
    double slope;
    if (fabs(end_[0] - start_[0]) > 1e-9)
      {
        slope = (end_[1] - start_[1]) / (end_[0] - start_[0]);
        angle_ = pi_to_pi(atan(slope) + M_PI/2);
      }
    else
      {
        angle_ = 0.0;
      }
  }

  void Line::radiusFromEndpoints()
  {
    radius_ = start_[0] * cos(angle_) + start_[1] * sin(angle_);
    if (radius_ < 0)
      {
        radius_ = -radius_;
        angle_ = pi_to_pi(angle_ + M_PI);
      }
  }

  void Line::leastSqFit()
  {
    calcPointCovariances();
    double prev_radius = 0.0;
    double prev_angle = 0.0;
    while (fabs(radius_ - prev_radius) > params_.least_sq_radius_thresh ||
           fabs(angle_ - prev_angle) > params_.least_sq_angle_thresh) 
      {
        prev_radius = radius_;
        prev_angle = angle_;
        calcPointScalarCovariances();
        radiusFromLeastSq();
        angleFromLeastSq();
      }
    calcCovariance();
    projectEndpoints();
  }

  void Line::angleFromLeastSq()
  {
    calcPointParameters();
    angle_ += angleIncrement();
  }

  double Line::angleIncrement()
  {
    const std::vector<double> &a = p_params_.a;
    const std::vector<double> &ap = p_params_.ap;
    const std::vector<double> &app = p_params_.app;
    const std::vector<double> &b = p_params_.b;
    const std::vector<double> &bp = p_params_.bp;
    const std::vector<double> &bpp = p_params_.bpp;
    const std::vector<double> &c = p_params_.c;
    const std::vector<double> &s = p_params_.s;

    double numerator = 0; 
    double denominator = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
      {
        numerator += (b[i] * ap[i] - a[i] * bp[i]) / pow(b[i], 2);
        denominator += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 
                        2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
      }
    return -(numerator/denominator);
  }

  void Line::calcCovariance()
  {
    covariance_[0] = p_rr_;

    const std::vector<double> &a = p_params_.a;
    const std::vector<double> &ap = p_params_.ap;
    const std::vector<double> &app = p_params_.app;
    const std::vector<double> &b = p_params_.b;
    const std::vector<double> &bp = p_params_.bp;
    const std::vector<double> &bpp = p_params_.bpp;
    const std::vector<double> &c = p_params_.c;
    const std::vector<double> &s = p_params_.s;

    double G = 0;
    double A = 0;
    double B = 0;
    double r, phi;
    for (std::size_t i = 0; i < a.size(); ++i)
      {
        r = r_data_.ranges[indices_[i]]; // range
        phi = c_data_.bearings[indices_[i]]; // bearing
        G += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
        A += 2 * r * sin(angle_ - phi) / b[i];
        B += 4 * pow(r, 2) * pow(sin(angle_ - phi), 2) / b[i];
      }
    covariance_[1] = p_rr_ * A / G;
    covariance_[2] = covariance_[1];
    covariance_[3] = pow(1.0 / G, 2) * B;
  }

  void Line::calcPointCovariances()
  {
    point_covs_.clear();
    double r, phi, var_r, var_phi;
    for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
      {
        r = r_data_.ranges[*cit]; // range
        phi = c_data_.bearings[*cit]; // bearing
        var_r = params_.range_var; // range variance
        var_phi = params_.bearing_var; // bearing variance
        boost::array<double, 4> Q; 
        Q[0] = pow(r, 2) * var_phi * pow(sin(phi), 2) + var_r * pow(cos(phi), 2);
        Q[1] = -pow(r, 2) * var_phi * sin(2 * phi) / 2.0 + var_r * sin(2 * phi) / 2.0;
        Q[2] = Q[1]; 
        Q[3] = pow(r, 2) * var_phi * pow(cos(phi), 2) + var_r * pow(sin(phi), 2);
        point_covs_.push_back(Q);
      }
  }

  void Line::calcPointParameters()
  {
    p_params_.a.clear();
    p_params_.ap.clear();
    p_params_.app.clear();
    p_params_.b.clear();
    p_params_.bp.clear();
    p_params_.bpp.clear();
    p_params_.c.clear();
    p_params_.s.clear();

    double r, phi, var_r, var_phi;
    double a, ap, app, b, bp, bpp, c, s;
    for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
      {
        r = r_data_.ranges[*cit]; // range
        phi = c_data_.bearings[*cit]; // bearing
        var_r = params_.range_var; // range variance
        var_phi = params_.bearing_var; // bearing variance
        c = cos(angle_ - phi);
        s = sin(angle_ - phi);
        a = pow(r * c - radius_, 2);
        ap = -2 * r * s * (r * c - radius_);
        app = 2 * pow(r, 2) * pow(s, 2) - 2 * r * c * (r * c - radius_);
        b = var_r * pow(c, 2) + var_phi * pow(r, 2) * pow(s, 2);
        bp = 2 * (pow(r, 2) * var_phi - var_r) * c * s;
        bpp = 2 * (pow(r, 2) * var_phi - var_r) * (pow(c, 2) - pow(s, 2));
        p_params_.a.push_back(a);
        p_params_.ap.push_back(ap);
        p_params_.app.push_back(app);
        p_params_.b.push_back(b);
        p_params_.bp.push_back(bp);
        p_params_.bpp.push_back(bpp);
        p_params_.c.push_back(c);
        p_params_.s.push_back(s);
      }
  }

  void Line::calcPointScalarCovariances()
  {
    point_scalar_vars_.clear();
    double P;
    double inverse_P_sum = 0;
    for (std::vector<boost::array<double, 4> >::const_iterator cit = point_covs_.begin();
         cit != point_covs_.end(); ++cit)
      {
        P = (*cit)[0] * pow(cos(angle_), 2) + 2 * (*cit)[1] * sin(angle_) * cos(angle_) +
          (*cit)[3] * pow(sin(angle_), 2);
        inverse_P_sum += 1.0 / P;
        point_scalar_vars_.push_back(P);
      }
    p_rr_ = 1.0 / inverse_P_sum;
  }

  void Line::radiusFromLeastSq()
  {
    radius_ = 0;
    double r, phi;
    for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
      {
        r = r_data_.ranges[*cit]; // range
        phi = c_data_.bearings[*cit]; // bearing
        radius_ += r * cos(angle_ - phi) / point_scalar_vars_[cit - indices_.begin()]; // cit to index
      }
  
    radius_ *= p_rr_;
  }


  LineExtraction::LineExtraction()
  {
  }

  LineExtraction::~LineExtraction()
  {
  }

  void LineExtraction::extractLines(std::vector<Line>& lines, bool verbose)
  {
    // Resets
    filtered_indices_ = c_data_.indices;
    lines_.clear();

    // Filter indices
    filterClosePoints();
    filterOutlierPoints();

    // Return no lines if not enough points left
    if (filtered_indices_.size() <= std::max(params_.min_line_points, static_cast<unsigned int>(3)))
      {
        return;
      }

    // Split indices into lines and filter out short and sparse lines
    split(filtered_indices_, verbose);
    if(verbose) std::cout << "size of lines: " << lines_.size() << std::endl;

    filterLines();
    // Fit each line using least squares and merge colinear lines
    for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end(); ++it)
      {
        it->leastSqFit();
      }

    // If there is more than one line, check if lines should be merged based on the merging criteria
    if (lines_.size() > 1)
      {
        mergeLines();
      }

    lines = lines_;

    if(verbose)
      {
        std::cout << "========== result: ===============" << std::endl;
        for(const auto& line: lines)
          std::cout << line.getIndices().at(0) << " to " << line.getIndices().back() << std::endl;
      }
  }

  void LineExtraction::setIndices(std::vector<unsigned int> indices)
  {
    c_data_.indices = indices;
  }


  void LineExtraction::setCachedData(std::vector<double> bearings,
                                     std::vector<double> cos_bearings,
                                     std::vector<double> sin_bearings,
                                     std::vector<unsigned int> indices)
  {
    c_data_.bearings = bearings;
    c_data_.cos_bearings = cos_bearings;
    c_data_.sin_bearings = sin_bearings;
    setIndices(indices);
  }

  void LineExtraction::setRangeData(const std::vector<double>& ranges)
  {
    r_data_.ranges = ranges;
    r_data_.xs.clear();
    r_data_.ys.clear();
    for (int i = 0; i < ranges.size(); i++)
      {
        r_data_.xs.push_back(c_data_.cos_bearings.at(i) * ranges.at(i));
        r_data_.ys.push_back(c_data_.sin_bearings.at(i) * ranges.at(i));
      }
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Parameter setting
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtraction::setBearingVariance(double value)
  {
    params_.bearing_var = value;
  }

  void LineExtraction::setRangeVariance(double value)
  {
    params_.range_var = value;
  }

  void LineExtraction::setLeastSqAngleThresh(double value)
  {
    params_.least_sq_angle_thresh = value;
  }

  void LineExtraction::setLeastSqRadiusThresh(double value)
  {
    params_.least_sq_radius_thresh = value;
  }

  void LineExtraction::setMaxLineGap(double value)
  {
    params_.max_line_gap = value;
  }

  void LineExtraction::setMinLineLength(double value)
  {
    params_.min_line_length = value;
  }

  void LineExtraction::setMinLinePoints(unsigned int value)
  {
    params_.min_line_points = value;
  }

  void LineExtraction::setMinRange(double value)
  {
    params_.min_range = value;
  }

  void LineExtraction::setMinSplitDist(double value)
  {
    params_.min_split_dist = value;
  }

  void LineExtraction::setOutlierDist(double value)
  {
    params_.outlier_dist = value;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Utility methods
  ///////////////////////////////////////////////////////////////////////////////
  double LineExtraction::chiSquared(const Eigen::Vector2d &dL, const Eigen::Matrix2d &P_1,
                                    const Eigen::Matrix2d &P_2)
  {
    return dL.transpose() * (P_1 + P_2).inverse() * dL;
  }

  double LineExtraction::distBetweenPoints(unsigned int index_1, unsigned int index_2)
  {
    return sqrt(pow(r_data_.xs[index_1] - r_data_.xs[index_2], 2) + 
                pow(r_data_.ys[index_1] - r_data_.ys[index_2], 2));
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Filtering points
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtraction::filterClosePoints()
  {
    std::vector<unsigned int> output;
    for (std::vector<unsigned int>::const_iterator cit = filtered_indices_.begin(); 
         cit != filtered_indices_.end(); ++cit)
      {
        if (r_data_.ranges[*cit] >= params_.min_range)
          {
            output.push_back(*cit);
          }
      }
    filtered_indices_ = output;
  }

  void LineExtraction::filterOutlierPoints()
  {
    if (filtered_indices_.size() < 3)
      {
        return;
      }

    std::vector<unsigned int> output;
    unsigned int p_i, p_j, p_k;
    for (std::size_t i = 0; i < filtered_indices_.size(); ++i)
      {

        // Get two closest neighbours

        p_i = filtered_indices_[i];
        if (i == 0) // first point
          {
            p_j = filtered_indices_[i + 1];
            p_k = filtered_indices_[i + 2];
          }
        else if (i == filtered_indices_.size() - 1) // last point
          {
            p_j = filtered_indices_[i - 1];
            p_k = filtered_indices_[i - 2];
          }
        else // middle points
          {
            p_j = filtered_indices_[i - 1];
            p_k = filtered_indices_[i + 1];
          }

        // Check if point is an outlier

        if (fabs(r_data_.ranges[p_i] - r_data_.ranges[p_j]) > params_.outlier_dist &&
            fabs(r_data_.ranges[p_i] - r_data_.ranges[p_k]) > params_.outlier_dist)
          {
            // Check if it is close to line connecting its neighbours
            std::vector<unsigned int> line_indices;
            line_indices.push_back(p_j);
            line_indices.push_back(p_k);
            Line line(c_data_, r_data_, params_, line_indices);
            line.endpointFit();
            if (line.distToPoint(p_i) > params_.min_split_dist)
              {
                continue; // point is an outlier
              }
          }

        if (std::isinf(r_data_.ranges[p_i])) continue;

        output.push_back(p_i);
      }

    filtered_indices_ = output;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Filtering and merging lines
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtraction::filterLines()
  {
    std::vector<Line> output;
    for (std::vector<Line>::const_iterator cit = lines_.begin(); cit != lines_.end(); ++cit)
      {
        if (cit->length() >= params_.min_line_length && cit->numPoints() >= params_.min_line_points)
          {
            output.push_back(*cit);
          }
      }
    lines_ = output;
  }

  void LineExtraction::mergeLines()
  {
    std::vector<Line> merged_lines;

    for (std::size_t i = 1; i < lines_.size(); ++i)
      {
        // Get L, P_1, P_2 of consecutive lines
        Eigen::Vector2d L_1(lines_[i-1].getRadius(), lines_[i-1].getAngle());
        Eigen::Vector2d L_2(lines_[i].getRadius(), lines_[i].getAngle());
        Eigen::Matrix2d P_1;
        P_1 << lines_[i-1].getCovariance()[0], lines_[i-1].getCovariance()[1],
          lines_[i-1].getCovariance()[2], lines_[i-1].getCovariance()[3];
        Eigen::Matrix2d P_2;
        P_2 << lines_[i].getCovariance()[0], lines_[i].getCovariance()[1],
          lines_[i].getCovariance()[2], lines_[i].getCovariance()[3];

        // Merge lines if chi-squared distance is less than 3
        if (chiSquared(L_1 - L_2, P_1, P_2) < 3)
          {
            // Get merged angle, radius, and covariance
            Eigen::Matrix2d P_m = (P_1.inverse() + P_2.inverse()).inverse();
            Eigen::Vector2d L_m = P_m * (P_1.inverse() * L_1 + P_2.inverse() * L_2);
            // Populate new line with these merged parameters
            boost::array<double, 4> cov;
            cov[0] = P_m(0,0);
            cov[1] = P_m(0,1);
            cov[2] = P_m(1,0);
            cov[3] = P_m(1,1);
            std::vector<unsigned int> indices;
            const std::vector<unsigned int> &ind_1 = lines_[i-1].getIndices();
            const std::vector<unsigned int> &ind_2 = lines_[i].getIndices();
            indices.resize(ind_1.size() + ind_2.size());
            indices.insert(indices.end(), ind_1.begin(), ind_1.end());
            indices.insert(indices.end(), ind_2.begin(), ind_2.end());
            Line merged_line(L_m[1], L_m[0], cov, lines_[i-1].getStart(), lines_[i].getEnd(), indices);
            // Project the new endpoints
            merged_line.projectEndpoints();
            lines_[i] = merged_line;
          }
        else
          {
            merged_lines.push_back(lines_[i-1]);
          }

        if (i == lines_.size() - 1)
          {
            merged_lines.push_back(lines_[i]);
          }
      }
    lines_ = merged_lines;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Splitting points into lines
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtraction::split(const std::vector<unsigned int>& indices, bool verbose)
  {
    // Don't split if only a single point (only occurs when orphaned by gap)
    if (indices.size() <= 1)
      {
        return;
      }

    if(verbose) std::cout << indices[0] << " to " << indices.back() << std::endl;
    Line line(c_data_, r_data_, params_, indices);
    line.endpointFit();
    double dist_max = 0;
    double gap_max = 0;
    double dist, gap;
    int i_max, i_gap;

    // Find the farthest point and largest gap
    for (std::size_t i = 1; i < indices.size() - 1; ++i)
      {
        dist = line.distToPoint(indices[i]);


        if (dist > dist_max)
          {
            dist_max = dist;
            i_max = i;

          }
        gap = distBetweenPoints(indices[i], indices[i+1]);

        if (gap > gap_max)
          {
            gap_max = gap;
            i_gap = i;

          }

        if(verbose) std::cout << indices[i] << ": " << r_data_.ranges.at(indices[i]) << ", dist:" << dist << std::endl;
      }

    // Check for gaps at endpoints
    double gap_start = distBetweenPoints(indices[0], indices[1]);
    if (gap_start > gap_max)
      {
        gap_max = gap_start;
        i_gap = 1;
      }
    double gap_end = distBetweenPoints(indices.rbegin()[1], indices.rbegin()[0]);
    if (gap_end > gap_max)
      {
        gap_max = gap_end;
        i_gap = indices.size() - 1;
      }

    if(verbose) std::cout << "max dist: " << dist_max << " in " << indices[i_max] << "; max gap: " << gap_max << " in " << indices[i_gap] << std::endl; 
    // Check if line meets requirements or should be split
    if (dist_max < params_.min_split_dist && gap_max < params_.max_line_gap)
      {
        if(verbose) std::cout << " --------------------- add line: " << indices[0] << " to " << indices.back() << std::endl;
        lines_.push_back(line);
      }
    else
      {
        int i_split = dist_max >= params_.min_split_dist ? i_max : i_gap;
        std::vector<unsigned int> first_split(&indices[0], &indices[i_split]);
        if(verbose) std::cout << indices[i_split - 1] << "; " << indices[i_split] << std::endl;
        std::vector<unsigned int> second_split(&indices[i_split], &indices.back() + 1);
        split(first_split, verbose);
        split(second_split, verbose);
      }

  }
} // namespace line_extraction
