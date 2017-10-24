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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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

#ifndef __ORGANIZED_CURVATURE_ESTIMATION_H__
#define __ORGANIZED_CURVATURE_ESTIMATION_H__

#include "cob_3d_features/organized_features.h"
#include "cob_3d_mapping_common/label_defines.h"

// number of currently used labels:
#define I_SIZE 3

namespace cob_3d_features
{
  template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT>
    class OrganizedCurvatureEstimation : public OrganizedFeatures<PointInT,PointOutT>
  {
    public:

    using OrganizedFeatures<PointInT,PointOutT>::pixel_search_radius_;
    using OrganizedFeatures<PointInT,PointOutT>::pixel_steps_;
    using OrganizedFeatures<PointInT,PointOutT>::circle_steps_;
    using OrganizedFeatures<PointInT,PointOutT>::n_points_;
    using OrganizedFeatures<PointInT,PointOutT>::mask_;
    using OrganizedFeatures<PointInT,PointOutT>::input_;
    using OrganizedFeatures<PointInT,PointOutT>::indices_;
    using OrganizedFeatures<PointInT,PointOutT>::surface_;
    using OrganizedFeatures<PointInT,PointOutT>::skip_distant_point_threshold_;
    using OrganizedFeatures<PointInT,PointOutT>::feature_name_;

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointNT> NormalCloudIn;
    typedef typename NormalCloudIn::Ptr NormalCloudInPtr;
    typedef typename NormalCloudIn::ConstPtr NormalCloudInConstPtr;

    typedef pcl::PointCloud<PointLabelT> LabelCloudOut;
    typedef typename LabelCloudOut::Ptr LabelCloudOutPtr;
    typedef typename LabelCloudOut::ConstPtr LabelCloudOutConstPtr;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;
    typedef typename PointCloudOut::Ptr PointCloudOutPtr;
    typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;

    OrganizedCurvatureEstimation ()
    {
      feature_name_ = "OrganizedCurvatureEstimation";
    };

    inline void
      setInputNormals(const NormalCloudInConstPtr &normals) { normals_ = normals; }

    inline void
      setOutputLabels(LabelCloudOutPtr labels) {labels_ = labels; }

    inline void
      setEdgeClassThreshold(float th)
    {
      edge_curvature_threshold_ = th;
    }

    void
      computePointCurvatures(
	const NormalCloudIn &normals,
	const int index,
	const std::vector<int> &indices,
	float &pcx,
	float &pcy,
	float &pcz,
	float &pc1,
	float &pc2,
	int &label_out);

    void
      computePointCurvatures(
	const NormalCloudIn &normals,
	const int index,
	const std::vector<int> &indices,
	const std::vector<float> &sqr_distances,
	float &pcx,
	float &pcy,
	float &pcz,
	float &pc1,
	float &pc2,
	int &label_out);

    protected:

    void
      computeFeature (PointCloudOut &output);

    NormalCloudInConstPtr normals_;
    LabelCloudOutPtr labels_;

    float edge_curvature_threshold_;

  };
}

#endif
