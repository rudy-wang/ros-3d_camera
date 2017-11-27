#include "scc.h"
#include "data_type.h"

#define THRD_NUM 1

template< class DATA_TYPE, int PARAM_DIM >
int assignCluster( std::vector< SCCcluster< DATA_TYPE, PARAM_DIM > > clstrHdlr, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > point )
{
	float closestDist = -1;
	int closestIdx = -1;
	
	for( int cidx = 0; cidx < clstrHdlr.size(); cidx++ )
	{
		float currentDist = clstrHdlr[ cidx ].computeDist( point );
      
		if( currentDist <= clstrHdlr[ cidx ].thres() )
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
	}
	return closestIdx;
}

template< class DATA_TYPE, int PARAM_DIM >
void SCCclustering( std::vector< SCCcluster< DATA_TYPE, PARAM_DIM > > &clstrHdlr, pointXYZYUV &inputdata, float init_thres, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > init_std )
{
	clock_t t1 = clock();
	for( int pidx = 0; pidx < inputdata.size(); pidx++ )
	{
		
		if( !( clstrHdlr.size() > 0 ) )
		{
			clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, inputdata.point( pidx ),  init_thres, init_std ) );
		}
		else
		{
			int pointCluster = -1;
			pointCluster = assignCluster( clstrHdlr, inputdata.point( pidx ) );
			
			if( pointCluster == -1 )
				clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, inputdata.point( pidx ),  init_thres, init_std ) );
			else
			{
				clstrHdlr[ pointCluster ].updateCluster( pidx, inputdata.point( pidx ) );
			}
		}
		if( pidx % 1000 == 0 )
		{
			std::cout << "ProcTimePer1000Point: " << (clock()-t1)/(double)(CLOCKS_PER_SEC) << " sec" << std::endl;
			t1 = clock();
		}
	}	
}
