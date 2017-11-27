#include "scc.h"
#include "data_type.h"

#define THRD_NUM 1

template< class DATA_TYPE, int PARAM_DIM >
void assignCluster( std::vector< SCCcluster< DATA_TYPE, PARAM_DIM > > &clstrHdlr, int pidx_b, int pidx_e, pointXYZYUV &pointRef, float init_thres, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > init_std )
{
	float closestDist = -1;
	int closestIdx = -1;
	
	for( int pidx = pidx_b; pidx < pidx_e; pidx++)
	{
		if( !( clstrHdlr.size() > 0 ) )
			clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, pointRef.point( pidx ),  init_thres, init_std ) );
		else
		{
			for( int cidx = 0; cidx < clstrHdlr.size(); cidx++ )
			{
				float currentDist = clstrHdlr[ cidx ].computeDist( pointRef.point( pidx ) );
				if( pidx==18093 )
				{
					std::cout << currentDist << std::endl;
					std::cin.ignore();
				}
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
			if( closestIdx == -1 )
				clstrHdlr.push_back( SCCcluster< DATA_TYPE, PARAM_DIM >( pidx, pointRef.point( pidx ),  init_thres, init_std ) );
			else
			{
				clstrHdlr[ closestIdx ].updateCluster( pidx, pointRef.point( pidx ) );
			}
		}
		std::cout << clstrHdlr.size() << std::endl;
	}
}

template< class DATA_TYPE, int PARAM_DIM >
void SCCclustering( std::vector< SCCcluster< DATA_TYPE, PARAM_DIM > > &clstrHdlr, pointXYZYUV &inputdata, float init_thres, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > init_std )
{
	clock_t t1 = clock();
	std::thread threads[ THRD_NUM ];
	for( int tidx = 0; tidx < THRD_NUM; tidx++ )
	{
		int start = inputdata.size() / THRD_NUM * tidx;
		int end = inputdata.size() / THRD_NUM * ( tidx + 1 );
		auto threadTask = std::bind( assignCluster< DATA_TYPE, PARAM_DIM >, std::ref( clstrHdlr ), start, end, inputdata, init_thres, init_std );
		threads[ tidx ] = std::thread( threadTask );
	}
	for( int tidx = 0; tidx < THRD_NUM; tidx++ )
	{
		threads[ tidx ].join();
	}
	std::cout << "ProcTime: " << (clock()-t1)/(double)(CLOCKS_PER_SEC) << " sec" << std::endl;
}
