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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef __IMPL_FAST_SEGMENTATION_HPP__
#define __IMPL_FAST_SEGMENTATION_HPP__

#include <set>

#include <pcl/common/eigen.h>
#include <Eigen/LU>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_segmentation/fast_segmentation.h"

#define LISTMOD(a,b) ( ((a%b)+b)%b )

template <typename PointT, typename PointNT, typename PointLabelT, typename OptionsT, typename SensorT, typename ClusterHdlT> void
cob_3d_segmentation::FastSegmentation<PointT,PointNT,PointLabelT,OptionsT,SensorT,ClusterHdlT>::createSeedPoints()
{
  int n = labels_->width * labels_->height;
  // make sure, image border are NANs
  for(size_t i = 0; i<labels_->width; ++i)
    (*labels_)[i].label = I_NAN;
  for(size_t i = labels_->size() - labels_->width; i<labels_->size(); ++i)
    (*labels_)[i].label = I_NAN;
  for(size_t i = labels_->width; i<labels_->size(); i+=labels_->width) {
    (*labels_)[i].label = I_NAN; (*labels_)[i+labels_->width-1].label = I_NAN; }

  seeds_.clear();
  p_seeds_.resize(n);
  switch (seed_method_)
  {
  case SEED_RANDOM:
  {
    p_seeds_[0] = seeds_.insert(seeds_.end(), 0);
    for(unsigned int i = 1; i<n; ++i)
    {
      //if(surface_->points[i].z != surface_->points[i].z)
      if((*labels_)[i].label == I_NAN) {
        p_seeds_[i] = p_seeds_[rand()%i];
      } else if((*normals_)[i].normal[2] != (*normals_)[i].normal[2]) {
        (*labels_)[i].label = I_NAN;
        p_seeds_[i] = p_seeds_[rand()%i];
        //std::cout << "NAN---" << std::endl;
      } else {
        p_seeds_[i] = seeds_.insert(p_seeds_[rand()%i],i);
      }
    }
    p_seeds_[0] = seeds_.insert(p_seeds_[rand()%n],0);
    seeds_.pop_back();
    break;
  }

  case SEED_LINEAR:
  {
    for(unsigned int i = 0; i<n; ++i)
    {
      //if(surface_->points[i].z != surface_->points[i].z)
      if((*labels_)[i].label == I_NAN)
        p_seeds_[i] = seeds_.end();
      else
        p_seeds_[i] = seeds_.insert(seeds_.end(),i);
    }
    break;
  }
  }
}


template <typename PointT, typename PointNT, typename PointLabelT, typename OptionsT, typename SensorT, typename ClusterHdlT> bool
cob_3d_segmentation::FastSegmentation<PointT,PointNT,PointLabelT,OptionsT,SensorT,ClusterHdlT>::compute()
{
  clusters_->setPointCloudIn(surface_);
  clusters_->setNormalCloudIn(normals_);
  clusters_->setLabelCloudInOut(labels_);
  graph_->edges()->setPointCloudIn(surface_);
  graph_->edges()->setLabelCloudIn(labels_);

  createSeedPoints();

  int w = labels_->width, h = labels_->height, s = 1;
  int mask[4][3] = { { -s, -w,  s }, // from below
                     { -w,  s,  w }, // from left
                     {  s,  w, -s }, // from above
                     {  w, -s, -w } }; // from right
  int mask_size = 3;

  clusters_->clear();
  clusters_->createCluster(I_NAN);
  while(seeds_.size() != 0)
  {
    unsigned int idx = seeds_.front();
    seeds_.pop_front();
    ClusterPtr c = clusters_->createCluster();

    std::list<SeedPoint::Ptr> seg_q;
    seg_q.push_back( SeedPoint::Ptr(new SeedPoint(idx, 0, 0.0)) );
    clusters_->addPoint(c, idx);
    (*labels_)[idx].label = c->id();

    while(seg_q.size() != 0)
    {
      SeedPoint p = *seg_q.front();
      seg_q.pop_front();

      for(int i=0; i<mask_size; ++i)
      {
        int i_idx = p.idx + mask[p.i_came_from][i];
        if (i_idx >= w*h || i_idx < 0) continue;

        if(!SensorT::areNeighbors((*surface_)[p.idx].getVector3fMap(), (*surface_)[i_idx].getVector3fMap(),4.0f)) continue;

        int* p_label = &(labels_->points[ i_idx ]).label;
        if( hasLabel(*p_label, c->id(), i_idx, p.idx, OptionsT()) ) continue;

        Eigen::Vector3f i_n = c->getOrientation();
        //Eigen::Vector3f i_c = c->getCentroid();

        float dot_value = fabs( i_n.dot((*normals_)[i_idx].getNormalVector3fMap()) );

        if(dot_value > n_threshold(c->size()))
        {
          seg_q.push_back( SeedPoint::Ptr(new SeedPoint(i_idx, LISTMOD(i + p.i_came_from - 1, 4), dot_value)) );
          clusters_->addPoint(c,i_idx);
          seeds_.erase(p_seeds_[i_idx]);
          *p_label = c->id();
        }
      }
    }
  }
  clusters_->addBorderIndicesToClusters();
  return true;
}

#endif
