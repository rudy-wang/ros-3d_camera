/*
 * quad_regression_algo.hpp
 *
 *  Created on: 14.02.2013
 *      Author: josh
 */

#include "../sub_structures/poly2d.hpp"

#include <cob_3d_mapping_common/stop_watch.h>

// ------------- CAMERA MODEL -------------



template <typename Point>
void CameraModel_Kinect<Point>::getParams(const pcl::PointCloud<Point> &pc) {
  if(f != 0.f) return;

  Point p1, p2;
  p1=p1;
  p2=p2;
  int i1=-1, i2=-1;

  // find one point
  for(size_t x=0; x<pc.width; x+=8) {
    for(size_t y=0; y<pc.height; y+=8) {
      int ind = ((x)+(y)*pc.width);
      if(pcl_isfinite(pc[ind].z)&&pc[ind].z<10.f) {
        p1=pc[ind];
        i1=ind;
        x=pc.width;
        break;
      }
    }
  }

  // find another point
  for(int x=pc.width-1; x>=0; x-=8) {
    for(int y=pc.height-1; y>=0; y-=8) {
      int ind = ((x)+(y)*pc.width);
      if(pcl_isfinite(pc[ind].z)&&pc[ind].z!=p1.z&&pc[ind].z<10.f) {
        p2=pc[ind];
        i2=ind;
        x=-1;
        break;
      }
    }
  }

  if(i1==-1||i2==-1) {
    ROS_WARN("no valid points");
    return;
  }

  //solve equation $...$ to retrieve f, dx and dy of camera
  int x=i1%pc.width;
  int y=i1/pc.width;
  float ax1,ax2, bx1,bx2;
  float ay1,ay2, by1,by2;

  ax1=p1.z/p1.x*x;
  bx1=p1.z/p1.x;
  ay1=p1.z/p1.y*y;
  by1=p1.z/p1.y;

  x=i2%pc.width;
  y=i2/pc.width;
  ax2=p2.z/p2.x*x;
  bx2=p2.z/p2.x;
  ay2=p2.z/p2.y*y;
  by2=p2.z/p2.y;

  dx = (ax1-ax2)/(bx1-bx2);
  dy = (ay1-ay2)/(by1-by2);
  f = ax1 - bx1*dx;
}



template <typename Point>
void CameraModel_SR4500<Point>::getParams(const pcl::PointCloud<Point> &pc) {
  if(f != 0.f) return;

  dx = 176/2;
  dy = 144/2;
  f = 1/std::tan(0.39f*M_PI/180);
}

// ------------- QUAD REGRESSION -------------

template <int Degree, typename Point, typename CameraModel>
QuadRegression<Degree, Point, CameraModel>::QuadRegression():
#ifdef SICK
MIN_LOD(4), FINAL_LOD(0), GO_DOWN_TO_LVL(4),
#else
MIN_LOD(8), FINAL_LOD(0), GO_DOWN_TO_LVL(3),
#endif
ch_(NULL), outline_check_(0), outline_check_size_(0),
filter_(-1.f), only_planes_(false)
{
  Contour2D::generateSpline2D();
}


template <int Degree, typename Point, typename CameraModel>
bool QuadRegression<Degree, Point, CameraModel>::compute() {
  polygons_.clear();
  this->buildTree(*input_);
  calc();
  return true;
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::prepare(const pcl::PointCloud<Point> &pc) {
  int w=input_->width;
  int h=input_->height;

  if(w<=1||h<=1) {
    ROS_ERROR("need ordered pointcloud");
    return;
  }

  for(int i=0; i<5; i++){
    levels_.push_back(SubStructure::ParamC<Degree>(w,h));

    w/=2;
    h/=2;
  }

  ch_=new int[levels_[0].w*levels_[0].h];
  memset(ch_,0,levels_[0].w*levels_[0].h*4);
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression_Downsampled<Degree, Point, CameraModel>::prepare(const pcl::PointCloud<Point> &pc) {
  int w=this->input_->width/2;
  int h=this->input_->height/2;

  if(w<=1||h<=1) {
    ROS_ERROR("need ordered pointcloud");
    return;
  }

  for(int i=0; i<4; i++){
    this->levels_.push_back(SubStructure::ParamC<Degree>(w,h));

    w/=2;
    h/=2;
  }

  this->ch_=new int[this->levels_[0].w*this->levels_[0].h];
  memset(this->ch_,0,this->levels_[0].w*this->levels_[0].h*4);
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::buildTree(const pcl::PointCloud<Point> &pc) {
#ifdef STOP_TIME
  PrecisionStopWatch ssw;
  ssw.precisionStart();
#endif

  //go through all levels
  for(size_t i=0; i<levels_.size(); i++) {

    SubStructure::ParamC<Degree> *lvl = &levels_[i];
    if(i==0) {  ///lowest-level: take points
      Eigen::Vector3f v;
      int j = 0;
      for(size_t y=0; y<lvl->h; y++) {
        for(size_t x=0; x<lvl->w; x++) {
          v(0) = pc[j].x;
          v(1) = pc[j].y;
          v(2) = pc[j].z;
          lvl->data[j]=v;
          lvl->data[j].occopied=-1;

          ++j;
        }
      }
    }
    else { //other levels take lower level
      SubStructure::ParamC<Degree> *lvl_prev = &levels_[i-1];

      int j=0, k=0;
      for(size_t y=0; y<lvl->h; y++) {
        for(size_t x=0; x<lvl->w; x++) {
          //j=getInd(x,y);
          lvl->data[j] = lvl_prev->data[k];    //getInd(i-1, x*2,y*2)
          lvl->data[j]+= lvl_prev->data[k+1]; //getInd(i-1, x*2,y*2+1)
          lvl->data[j]+= lvl_prev->data[k+levels_[i-1].w];    //getInd(i-1, x*2+1,y*2)
          lvl->data[j]+= lvl_prev->data[k+levels_[i-1].w+1];        //getInd(i-1, x*2+1,y*2+1)
          ++j;
          k+=2;
        }
        k+=levels_[i-1].w;
      }
    }

  }

#ifdef STOP_TIME
  execution_time_quadtree_ = ssw.precisionStop();
#endif
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression_Downsampled<Degree, Point, CameraModel>::buildTree(const pcl::PointCloud<Point> &pc) {
#ifdef STOP_TIME
  PrecisionStopWatch ssw;
  ssw.precisionStart();
#endif

  //go through all levels
  for(size_t i=0; i<this->levels_.size(); i++) {

    SubStructure::ParamC<Degree> *lvl = &this->levels_[i];
    if(i==0) {  ///lowest-level: take points
      Eigen::Vector3f p,t;
      int j = 0;
      for(size_t y=0; y<lvl->h; y++) {
        for(size_t x=0; x<lvl->w; x++) {
          int num=0;
          p(0)=p(1)=p(2)=0.f;

          // sub-sample 4 points to one
          j=this->getIndPC(pc.width, 2*x,2*y);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=this->getIndPC(pc.width, 2*x,2*y+1);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=this->getIndPC(pc.width, 2*x+1,2*y);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=this->getIndPC(pc.width, 2*x+1,2*y+1);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          //revalidate
          j=this->getIndPC(pc.width, 2*x,2*y);
          if((pc[j].getVector3fMap()-p/num).squaredNorm()>0.005f)
          {
            p(0)-=pc[j].x;
            p(1)-=pc[j].y;
            p(2)-=pc[j].z;
            --num;
          }

          j=this->getIndPC(pc.width, 2*x,2*y+1);
          if((pc[j].getVector3fMap()-p/num).squaredNorm()>0.005f)
          {
            p(0)-=pc[j].x;
            p(1)-=pc[j].y;
            p(2)-=pc[j].z;
            --num;
          }

          j=this->getIndPC(pc.width, 2*x+1,2*y);
          if((pc[j].getVector3fMap()-p/num).squaredNorm()>0.005f)
          {
            p(0)-=pc[j].x;
            p(1)-=pc[j].y;
            p(2)-=pc[j].z;
            --num;
          }

          j=this->getIndPC(pc.width, 2*x+1,2*y+1);
          if((pc[j].getVector3fMap()-p/num).squaredNorm()>0.005f)
          {
            p(0)-=pc[j].x;
            p(1)-=pc[j].y;
            p(2)-=pc[j].z;
            --num;
          }

          j=this->getInd(i,x,y);
          lvl->data[j]=p/num;

          lvl->data[j].occopied=-1;
        }
      }
    }
    else { //other levels take lower level
      SubStructure::ParamC<Degree> *lvl_prev = &this->levels_[i-1];

      int j=0, k=0;
      for(size_t y=0; y<lvl->h; y++) {
        for(size_t x=0; x<lvl->w; x++) {
          //j=getInd(x,y);
          lvl->data[j] = lvl_prev->data[k];    //getInd(i-1, x*2,y*2)
          lvl->data[j]+= lvl_prev->data[k+1]; //getInd(i-1, x*2,y*2+1)
          lvl->data[j]+= lvl_prev->data[k+this->levels_[i-1].w];    //getInd(i-1, x*2+1,y*2)
          lvl->data[j]+= lvl_prev->data[k+this->levels_[i-1].w+1];        //getInd(i-1, x*2+1,y*2+1)
          ++j;
          k+=2;
        }
        k+=this->levels_[i-1].w;
      }
    }

  }

#ifdef STOP_TIME
  this->execution_time_quadtree_ = ssw.precisionStop();
#endif
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::calc() {

#ifdef STOP_TIME
  execution_time_polyextraction_=0.;
  PrecisionStopWatch ssw;
  ssw.precisionStart();
#endif

  for(int i=(int)levels_.size()-1; i>(int)(levels_.size()-GO_DOWN_TO_LVL); i--) {

    //from one corner to the other
    size_t sx=0, sy=0;
    while(sx<levels_[i].w-1 && sy<levels_[i].h-1) {
      for(int x=sx+1; x<(int)levels_[i].w-1; x++) {
        if(isOccupied(i,x,sy)==-1)
        {
          SubStructure::Model<Degree> m;
          grow(m, i, x,sy);
        }
      }
      sx+=2;

      for(size_t y=sy+1; y<levels_[i].h-1; y++) {
        if(isOccupied(i,sx,y)==-1)
        {
          SubStructure::Model<Degree> m;
          grow(m, i, sx,y);
        }
      }
      sy+=2;
    }
    /*
    //from one corner to the other
    size_t sx=0, sy=0;
    while(sx<levels_[i].w-1 && sy<levels_[i].h-1) {
      for(int x=sx+1; x<(int)levels_[i].w-1; x++) {
        //if(isOccupied2(i,x,sy)==-1&&!checkOccupiedDeep(i,x,sy))
        if(isOccupied(i,levels_[i].w-1-x,sy)==-1)
        {
          SubStructure::Model<Degree> m;
          grow(m, i, levels_[i].w-1-x,sy);
        }
      }
      sx+=2;

      for(size_t y=sy+1; y<levels_[i].h-1; y++) {
        //if(isOccupied2(i,sx,y)==-1&&!checkOccupiedDeep(i,sx,y))
        if(isOccupied(i,levels_[i].w-1-sx,y)==-1)
        {
          SubStructure::Model<Degree> m;
          grow(m, i, levels_[i].w-1-sx,y);
        }
      }
      sy+=2;
    }
*/

  }

#ifdef STOP_TIME
  execution_time_growing_ = ssw.precisionStop();
#endif

  //preparePolygons();
}


template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::grow(SubStructure::Model<Degree> &model, const int i, const int x, const int y) {
  static SubStructure::VISITED_LIST<SubStructure::SVALUE> list;
  list.init();
  list.add( SubStructure::SVALUE(getInd(i,x,y),levels_[i].w*levels_[i].h) );

  grow(list, model, i, polygons_.size(), true);
}

template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::grow(SubStructure::VISITED_LIST<SubStructure::SVALUE> &list, SubStructure::Model<Degree> &model, const int i, const int mark, bool first_lvl) {
  int x, y, hops, occ;
  unsigned int found=0;
  bool bNew,bNew2;
  int last_size=-1;
#ifdef CHECK_CONNECTIVITY
  S_POLYGON_CONNECTIVITY connectivity;
#endif

  do {
    bNew=false;
    bNew2=false;
    while(list.pos+1<list.size) {
      list.move();

      x= list.vals[list.pos].v%levels_[i].w;
      y= list.vals[list.pos].v/levels_[i].w;
      hops = list.vals[list.pos].hops;

      occ = isOccupied(i,x,y);
      if(occ==mark) {
        if(list.pos>last_size&&hops) list.remove();
        continue;
      }
      else if(occ!=-1) {
#ifdef CHECK_CONNECTIVITY
        if(/*i==FINAL_LOD && */checkModelAt(model, i,x,y,
                                            0.02)) { //TODO: check threshold
          while(occ>=connectivity.connections_.size())
            connectivity.connections_.push_back(0);
          connectivity.connections_[occ]+=(1<<i);
        }
#endif
        continue;
      }

      float d=std::min(
          std::abs(levels_[i].data[getInd(i,x,y)].z_(0)/levels_[i].data[getInd(i,x,y)].model_(0,0)),
          std::abs(model.param.z_(0)/model.param.model_(0,0))
      );
      /*
#if defined(SIMULATION_)
      if(d>20.f) continue;
      const float thr=(d+2.f)*0.0015f;
#elif defined(DO_NOT_DOWNSAMPLE_)
      const float thr=(d*d+1.2f)*0.004f;      //TODO: check
#else
      const float thr=(d*d+1.2f)*0.004f;
#endif
       */
      if( hops>0 && x>0&&y>0&&x+1<(int)levels_[i].w&&y+1<(int)levels_[i].h &&
          d!=0.f && ((found<1&&first_lvl
              //#ifdef USE_MIN_MAX_RECHECK_
              //                &&(levels_[i].data[getInd(x,y)].v_max_-levels_[i].data[getInd(x,y)].v_min_)<0.02f*(1<<i)*std::abs(levels_[i].data[getInd(x,y)].z_(0)/levels_[i].data[getInd(x,y)].model_(0,0))
              //#endif
          ) ||
          (
#ifdef USE_MIN_MAX_RECHECK_
#ifdef DO_NOT_DOWNSAMPLE_

              //expected maximum:   model.get_max_gradient(levels_[i].data[getInd(i, x,y)])*d*(1<<i)/camera_.f
              //+3sigma*(standard deviation at max./min.)

              (levels_[i].data[getInd(i, x,y)].v_max_-levels_[i].data[getInd(i, x,y)].v_min_) <
              (model.get_max_gradient(levels_[i].data[getInd(i, x,y)])*d*(1<<i)/camera_.f
                  +3*(camera_.std(levels_[i].data[getInd(i, x,y)].v_max_)
                      +camera_.std(levels_[i].data[getInd(i, x,y)].v_min_)) )


                      //(levels_[i].data[getInd(i, x,y)].v_max_-levels_[i].data[getInd(i, x,y)].v_min_)< (model.get_max_gradient(levels_[i].data[getInd(i, x,y)])*d*(1<<i)/camera_.f+2*thr) /*std::min(0.5f,std::max(0.02f,0.05f*d))*/ //TODO: in anbhaengigkeit der steigung
#else
                      (levels_[i].data[getInd(i, x,y)].v_max_-levels_[i].data[getInd(i, x,y)].v_min_)< 2*(model.get_max_gradient(levels_[i].data[getInd(i, x,y)])*d*(1<<i)/camera_.f+4*thr) /*std::min(0.5f,std::max(0.02f,0.05f*d))*/ //TODO: in anbhaengigkeit der steigung
#endif
                      //&& std::abs(model.model(levels_[i].data[getInd(x,y)].v_min_(0),levels_[i].data[getInd(x,y)].v_min_(1))-levels_[i].data[getInd(x,y)].v_min_(2))<thr
                      //&& std::abs(model.model(levels_[i].data[getInd(x,y)].v_max_(0),levels_[i].data[getInd(x,y)].v_max_(1))-levels_[i].data[getInd(x,y)].v_max_(2))<thr
                      &&
#endif
                      (
                          //checkModelAt(model, i,x,y, thr)
                          checkModelAt(model, i,x,y)
                          /*||
                  checkModelAtZ(model, i,x,y,
                                0.01f)*/)
                                //d*(0.04-0.005*levels_[i].data[getInd(x,y)].model_(0,0)/model.param.model_(0,0))))
                                //0.015)) {
                                //d*(0.01-0.005*levels_[i].data[getInd(x,y)].model_(0,0)/model.param.model_(0,0)))
          )
#ifdef USE_NORMAL_CHECK
          && ( (i!=1&&i!=2) || std::abs(SubStructure::Model(levels_[i].data[getInd(x,y)]).getLinearNormal().dot(
              model.getNormal(levels_[i].data[getInd(x,y)].model_(1,0)/levels_[i].data[getInd(x,y)].model_(0,0),
                              levels_[i].data[getInd(x,y)].model_(3,0)/levels_[i].data[getInd(x,y)].model_(0,0)) ))>0.8 ) //TODO: optimize normalize out
#endif
          )
      ) {
        ++found;
        levels_[i].data[getInd(i,x,y)].occopied=mark;
        model+=levels_[i].data[getInd(i,x,y)];
        if(hops>10 || found%20==0)
          model.get();
        bNew=bNew2;

        list.add(SubStructure::SVALUE(getInd(i, x+1,y),hops-1));
        list.add(SubStructure::SVALUE(getInd(i, x-1,y),hops-1));
        list.add(SubStructure::SVALUE(getInd(i, x,y+1),hops-1));
        list.add(SubStructure::SVALUE(getInd(i, x,y-1),hops-1));

      }
      else
        bNew2=true;
    }
    last_size=list.pos;
    list.pos=-1;
    //break;
  } while(bNew);

  if(first_lvl && found<MIN_LOD) {
    for(int j=0; j<list.size; j++) {
      if(levels_[i].data[list.vals[j].v].occopied==mark)
        levels_[i].data[list.vals[j].v].occopied=-1;
    }
    return;
  }

  // if finished then create polygon
  if(i==(int)FINAL_LOD) {
    if(!list.size)
    {
      ROS_ASSERT(0);
      return;
    }

    if(only_planes_ && !model.isLinearAndTo())
      return;

    S_POLYGON<Degree> poly;
    poly=model;
    poly.mark_ = mark;

    //std::cout<<model.p<<"\n";

#ifdef CHECK_CONNECTIVITY
    poly.connectivity_=connectivity;
#endif

    static std::vector<SubStructure::SXY> outs;
    SubStructure::SXY pt;

    outs.clear();

    for(int j=0; j<list.size; j++) {
      x= list.vals[j].v%levels_[i].w;
      y= list.vals[j].v/levels_[i].w;

      if(x>0 && y>0 && x<(int)levels_[i].w && y<(int)levels_[i].h && isOccupied(i,x,y)==mark) continue;

      pt.x=x;pt.y=y;
#ifdef USE_MIN_MAX_RECHECK_
      //        const float delta = (levels_[i+2].data[getInd2(x/4,y/4)].v_max_-
      //            model.model(levels_[i].data[getInd(x,y)].model_(1)/levels_[i].data[getInd(x,y)].model_(0,0),
      //                        levels_[i].data[getInd(x,y)].model_(3)/levels_[i].data[getInd(x,y)].model_(0,0)));
      //        pt.back = (delta > -0.01f && levels_[i+2].data[getInd(i+2, x/4,y/4)].v_min_<0.1f) || delta>2*(model.get_max_gradient(levels_[i].data[getInd(x,y)])*levels_[i].data[getInd(x,y)].z_(0)/levels_[i].data[getInd(x,y)].model_(0,0)*(1<<i)/kinect_params_.f+4*0.03f);
      pt.back = levels_[i+2].data[getInd(i+2, x/4,y/4)].v_max_-
          model.model(levels_[i+2].data[getInd(i+2, x/4,y/4)].model_(1)/levels_[i+2].data[getInd(i+2, x/4,y/4)].model_(0,0),
                      levels_[i+2].data[getInd(i+2, x/4,y/4)].model_(3)/levels_[i+2].data[getInd(i+2, x/4,y/4)].model_(0,0))
                      > (model.get_max_gradient(levels_[i+2].data[getInd(i+2, x/4,y/4)])*levels_[i+2].data[getInd(i+2, x/4,y/4)].z_(0)/levels_[i+2].data[getInd(i+2, x/4,y/4)].model_(0,0)*(1<<i)/camera_.f+4*0.03f);
      //TODO: improve this stupid thing (but it was easy :) )
#endif
      outs.push_back(pt);
    }

    outline(ch_, levels_[i].w,levels_[i].h,outs,i, poly, model, mark);
//    if(poly.segments_.size()<1)
//    {
//      //ROS_WARN("segment empty");
//      return;
//    }

    if(filter_>0.f) {
      float area = poly.area();

      if(poly.weight_*poly.model_.param.v_max_*poly.model_.param.v_max_/area>filter_)
        polygons_.push_back(poly);
    }
    else
      polygons_.push_back(poly);

    return;
  }

  //region growing
  SubStructure::VISITED_LIST<SubStructure::SVALUE> list_lower;
  for(int j=0; j<list.size; j++) {
    x= list.vals[j].v%levels_[i].w;
    y= list.vals[j].v/levels_[i].w;

    hops=9;     // max. allowed movement in inner nodes

    if(filterOccupied(i,x,y,mark))
      continue;

    bool above = y>0 &&                    filterOccupied(i,x,y-1,mark);
    bool below = y+1<(int)levels_[i].h &&  filterOccupied(i,x,y+1,mark);
    bool left  = x>0 &&                    filterOccupied(i,x-1,y,mark);
    bool right = x+1<(int)levels_[i].w &&  filterOccupied(i,x+1,y,mark);

    if(above&&below&&left&&right)
      continue;

    if(above) {
      list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x,2*y),hops/2));
      list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x+1,2*y),hops/2));
    }
    if(below) {
      list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x,2*y+1),hops/2));
      list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x+1,2*y+1),hops/2));
    }
    if(left && !above) list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x,2*y),hops/2));
    if(left && !below) list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x,2*y+1),hops/2));
    if(right && !above) list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x+1,2*y),hops/2));
    if(right && !below) list_lower.add(SubStructure::SVALUE(getInd(i-1, 2*x+1,2*y+1),hops/2));
  }

  grow(list_lower, model, i-1, mark, false);
}

template <int Degree, typename Point, typename CameraModel>
int QuadRegression<Degree, Point, CameraModel>::getPos(int *ch, const int xx, const int yy, const int w, const int h) {
  int p=0;
  for(int x=-1; x<=1; x++) {
    for(int y=-1; y<=1; y++) {
      if( xx+x>=0 && yy+y>=0 && xx+x<w && yy+y<h &&
          (x||y) && ch[getInd(0, xx+x,yy+y)]>0)
      {
        p |= (1<<Contour2D::SplineMap[ (y+1)*3 + x+1]);
      }
    }

  }
  return p;
}

template <int Degree, typename Point, typename CameraModel>
void QuadRegression<Degree, Point, CameraModel>::outline(int *ch, const int w, const int h, std::vector<SubStructure::SXY> &out, const int i, S_POLYGON<Degree> &poly, const SubStructure::Model<Degree> &model, const int mark)
{
  //    std::cout<<"OFF:\n"<<poly.param_.col(0)<<"\n";
  //    std::cout<<"PLANE:\n"<<poly.proj2plane_<<"\n";
  //    std::cout<<"P1:\n"<<poly.param_.col(1)<<"\n";
  //    std::cout<<"P2:\n"<<poly.param_.col(2)<<"\n";

#ifdef STOP_TIME
  PrecisionStopWatch ssw;
  ssw.precisionStart();
#endif

  SubStructure::SXYcmp ttt;
  std::sort(out.begin(),out.end(), ttt);

  for(size_t j=0; j<out.size(); j++) {
    ch[ getInd(i, out[j].x,out[j].y) ]=(int)j+1;
  }

#if DEBUG_LEVEL>200
  {  char buf[128];
  int *bs = new int[w*h];
  memset(bs,0,w*h*4);
  for(size_t j=0; j<out.size(); j++) {
    bs[ getInd(out[j].x,out[j].y) ]=-(out[j].back+1);
  }
  sprintf(buf,"/tmp/poly%d.ppm",polygons_.size());
  QQPF_Debug::ppm(buf,w,h,bs);
  delete [] bs;
  }
#endif

  if(outline_check_size_<out.size()) {
    delete [] outline_check_;
    outline_check_ = new bool[out.size()];
    outline_check_size_=out.size();
  }
  memset(outline_check_,false,out.size());

  int n=-1;
  while(n+1<(int)out.size()) {
    ++n;
    if(outline_check_[n])
      continue;

    poly.segments_.push_back(std::vector<Eigen::Vector3f>());
#ifdef USE_BOOST_POLYGONS_
    poly.segments2d_.push_back(std::vector<BoostPoint>());
#else
    poly.segments2d_.push_back(std::vector<Eigen::Vector2i>());
#endif

    int x=out[n].x;
    int y=out[n].y;
    int bf=8;
    int v=0;
    int back=0;
    int start_x=x, start_y=y;

    addPoint(i,x,y,mark, poly,model);
    int num=0,numb=0;

    while(1) {

      if(x<0 || y<0 || x>=w || y>=h || ch[ getInd(i,x,y) ]<1) {
        break;
      }

      back+=out[ch[ getInd(i,x,y) ]-1].back?1:0;
      ++numb;
      outline_check_[ch[ getInd(i,x,y) ]-1]=true;
      ch[ getInd(i,x,y) ]=-2;
      int p=getPos(ch,x,y,w,h);

      if(p==0|| (!Contour2D::g_Splines[bf][p].x&&!Contour2D::g_Splines[bf][p].y) )
      {
        break;
      }

      v+=v+Contour2D::g_Splines[bf][p].v;
      x+=Contour2D::g_Splines[bf][p].x;
      y+=Contour2D::g_Splines[bf][p].y;
      bf=Contour2D::g_Splines[bf][p].bf;
      ++num;

#ifdef EVALUATE
      if(std::abs(v)>6) {
#else
        if(std::abs(v)>3) {
#endif
        v=0;
        Eigen::Vector2f tv;
        tv(0)=x;tv(1)=y;
        addPoint(i,x,y,mark, poly,model,back/(float)numb);
        numb=back=0;
      }
    }
    if(poly.segments_.back().size()>0) poly.segments_.back()[0](2)=back/(float)numb;

    if(poly.segments_.back().size()<3 || (std::abs(x-start_x)+std::abs(y-start_y))>10 ) {
      poly.segments_.erase(poly.segments_.end()-1);

#if defined(USE_BOOST_POLYGONS_) || defined(BACK_CHECK_REPEAT)
      poly.segments2d_.erase(poly.segments2d_.end()-1);
#endif
    }


  }

  //    for(size_t j=0; j<poly.segments_.size(); j++)
  //      for(size_t k=0; k<poly.segments_[j].size(); k++)
  //      {
  //        std::cout<<"s\n"<<poly.segments_[j][k].head<2>()<<"\n";
  //        std::cout<<"p\n"<<poly.project2world(poly.segments_[j][k].head<2>())<<"\n";
  //      }

  for(size_t j=0; j<out.size(); j++) {
    ch[ getInd(i, out[j].x,out[j].y) ]=0;
  }

#ifdef STOP_TIME
  execution_time_polyextraction_ += ssw.precisionStop();
#endif

}
