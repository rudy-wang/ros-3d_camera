#include "scc.h"
#include "cv_imgproc.h"

#define THRD_NUM 4

template< class DATA_TYPE, int PARAM_DIM >
void computDist_threadTask( int cidx, SCCcluster< DATA_TYPE, PARAM_DIM > master, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > point, float &closestDist, int &closestIdx )
{
	float currentDist = master.computeDist( point );
	if( currentDist <= master.thres() )
	{
		if( closestDist < 0 )
		{
			closestDist = currentDist;
			closestIdx = cidx;
		}
		else if( currentDist < closestDist )
		{
			closestDist = currentDist;
			closestIdx = cidx;
		}
	}
};

template< class DATA_TYPE, int PARAM_DIM >
void SCCclustering( std::vector< SCCcluster< DATA_TYPE, PARAM_DIM > > clstrHdlr, pointXYZYUV inputdata, float init_thres, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > init_std )
{
	for( int pidx = 0; pidx < inputdata.size(); pidx++ )
	{
		if( !( clstrHdlr.size() > 0 ) )
		{
			clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, inputdata.point( pidx ),  init_thres, init_std ) );
		}
		else
		{
			float closestDist = -1;
			int closestIdx = -1;
			std::thread threads[ THRD_NUM ];
			std::vector< int > idle_thread;
			for( int init_idle = 0; init_idle < THRD_NUM; init_idle++ )
			{
				idle_thread.push_back( init_idle );
			}
			for( int cidx = 0; cidx < clstrHdlr.size(); cidx++ )
			{
				std::cout << "point: " << pidx << " , cluster: " << cidx << " , thread: " << idle_thread.back() << std::endl;
				/*******Multi-thread*******/
				if( idle_thread.size() > 0 ){
					std::cout << "111" << std::endl;
					std::cout << threads[ 0 ].joinable() << std::endl;
					threads[ idle_thread.back() ] = std::thread( computDist_threadTask< DATA_TYPE, PARAM_DIM >, cidx, clstrHdlr[ cidx ], inputdata.point( pidx ), boost::ref( closestDist ) , boost::ref( closestIdx ) );
					idle_thread.pop_back();
				}
				else
				{
					int tidx = 0;
					while( !threads[ tidx ].joinable() )
					{
						tidx++;
						tidx = tidx % THRD_NUM;
					}
					threads[ tidx ].join();
					idle_thread.push_back( tidx );
				}
			}
			if( closestIdx == -1 )
				clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, inputdata.point( pidx ),  init_thres, init_std ) );
			else
			{
				clstrHdlr[ closestIdx ].updateCluster( pidx, inputdata.point( pidx ) );
			}
		}
	}	
}

