/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#ifndef POLYGON_TYPES_H_
#define POLYGON_TYPES_H_

#include <vector>

namespace cob_3d_segmentation
{
  class PolygonPoint
  {
  public:
    PolygonPoint() : x(0), y(0) { }
    PolygonPoint(int ctor_x, int ctor_y) : x(ctor_x), y(ctor_y) { }

    virtual ~PolygonPoint() { };

    inline bool operator<(const PolygonPoint& rhs) const
    {
      if(y==rhs.y) { return x<rhs.x; }
      return y<rhs.y;
    }

    static int getInd(const int x, const int y) { return x + y * 640; }

  public:
    int x,y;
  };


  template <typename TPoint>
  class PolygonContours
  {
  public:
    virtual ~PolygonContours() { };

    void addPolygon() { polys_.push_back(std::vector<TPoint>()); }
    void removePolygon() { polys_.erase(polys_.end()-1); }
    void addPoint(int x, int y) { polys_.back().push_back(TPoint(x,y)); }
    /*void removeLastPoints(typename std::vector<TPoint>::size_type n)
    {
      if(polys_.back().size() > n) { polys_.back().resize(polys_.back().size() - n); }
      }*/
    void removeLastPoints(int n)
    {
      if(n && polys_.back().size() > n)
      {
        polys_.back().erase( polys_.back().begin()+(polys_.back().size()-n), polys_.back().end() );
      }
    }

  public:
    std::vector<std::vector<TPoint> > polys_;
  };
}

#endif
