#include "scc.h"
#include "input_data.h"

#define ROUGH 0
#define REFINE 1
//#define PT_PER_THRD 50000

template< int PARAM_DIM >
void assignCluster( SCCcluster< PARAM_DIM > &clstrHdlr, Eigen::Array< float, Eigen::Dynamic, PARAM_DIM > point, int idx_b, int idx_e, int cstart, int cend, Eigen::Array< int, Eigen::Dynamic, 1 > &clstrLabel, int refineFlag = 0 )
{
	std::vector< float > IdxSimPair = { -1, -1, -1 };
	for( int pidx = idx_b; pidx < idx_e; pidx++ )
	{
		if( pidx % 10000 == 0 )
			std::cout << pidx << std::endl << "CNUM: " << clstrHdlr.size() << std::endl;
		if( refineFlag == 1 && clstrLabel[ pidx ] != -1 )
		{
			clstrHdlr.refineCluster( clstrLabel, pidx, point.row( pidx ) );
		}
		IdxSimPair = clstrHdlr.computeSim( point.row( pidx ), cstart, cend );
		if( IdxSimPair[ 0 ] == -1 )
		{
			clstrHdlr.newCluster( pidx, point.row( pidx ) );
			clstrLabel[ pidx ] = clstrHdlr.size() - 1;
		}
		else
		{
			if( IdxSimPair[ 2 ] == 1 )
			{
				clstrHdlr.updateCluster( IdxSimPair[ 0 ], pidx, point.row( pidx ) );
				clstrLabel[ pidx ] = IdxSimPair[ 0 ];
			}
			else if( IdxSimPair[ 2 ] == -1 )
			{
				clstrHdlr.newCluster( pidx, point.row( pidx ), clstrHdlr.group( IdxSimPair[ 0 ] ) );
				clstrLabel[ pidx ] = clstrHdlr.size() - 1;
			}
		}
	}
}


template< int PARAM_DIM >
void rgbdSCC( pointXYZC &input, InitSetting< PARAM_DIM > iniSet, InitSetting< PARAM_DIM > iniSet2 )
{
	SCCcluster< PARAM_DIM > CclusterHdler( iniSet );	
	SCCcluster< PARAM_DIM > GclusterHdler( iniSet2 );
	SCCparallel( CclusterHdler, input.point(), input.Ccluster, 6, 11, 4, ROUGH );
	Eigen::Array< int, Eigen::Dynamic, 1 > Gcluster( CclusterHdler.size(), 1 );
	CclusterHdler.computeNormal();
	SCCparallel( GclusterHdler, CclusterHdler.mean(), Gcluster, 5, 11, 1, ROUGH );
	std::cout << GclusterHdler.size() << "___" << GclusterHdler.groupNum() << std::endl;
	for( int i = 0; i < input.size(); i++ )
	{
		input.Gcluster[ i ] = Gcluster[ input.Ccluster[ i ] ];
	}
	input.showCluster( CclusterHdler, COLOR_ );
	input.showCluster( GclusterHdler, GEO_ );
	SCCparallel( GclusterHdler, CclusterHdler.mean(), Gcluster, 5, 11, 1, REFINE );
	for( int i = 0; i < input.size(); i++ )
	{
		input.Gcluster[ i ] = Gcluster[ input.Ccluster[ i ] ];
	}
	std::cout << "clustering done" << std::endl;
	std::cout << GclusterHdler.size() << "___" << GclusterHdler.groupNum() << std::endl;
	input.showCluster( GclusterHdler, GEO_ );
	//std::cout << GclusterHdler.size() << std::endl;
	//input.showImg();
	//input.showFuseImg( int( CclusterHdler.size() ), COLOR_ );
	//input.showFuseImg( int( GclusterHdler.size() ), GEO_ );
}

template< class DATA_TYPE, int PARAM_DIM >
void SCCparallel( SCCcluster< PARAM_DIM > &clstrHdlr, Eigen::Array< DATA_TYPE, Eigen::Dynamic, PARAM_DIM > inputdata, Eigen::Array< int, Eigen::Dynamic, 1 > &clstrLabel, int cstart, int cend, int THRD_NUM, int refineFlag = 0 )
{
	if( refineFlag == 1 && THRD_NUM > 1 )
	{
		std::cout << "ERROR: Refinement must be process on single thread." << std::endl;
		exit(-1);
	}
	clock_t t1 = clock();
	std::thread threads[ THRD_NUM ];
	std::vector< int > idlethread;
	int PT_PER_THRD = ceil( inputdata.rows() / THRD_NUM );
	SCCcluster< PARAM_DIM > tempClstrHdlr[ THRD_NUM ];
	int labelInter[ THRD_NUM ][ 2 ];
	
	for( int tidx = THRD_NUM - 1; tidx >= 0; tidx-- )
	{
		idlethread.push_back( tidx );
	}
	int start = 0;
	int end = PT_PER_THRD > inputdata.rows() ? inputdata.rows() : PT_PER_THRD;
	while( start < inputdata.rows() || idlethread.size() < THRD_NUM )
	{
		while( idlethread.size() > 0 && start < inputdata.rows() )
		{
			int currentCidx = idlethread.back();
			if( refineFlag == 0 )
			{
				tempClstrHdlr[ currentCidx ].reset();
				tempClstrHdlr[ currentCidx ].initialize( clstrHdlr.showInitSet() );
				auto threadTask = std::bind( assignCluster< PARAM_DIM >, std::ref( tempClstrHdlr[ currentCidx ] ), inputdata.template cast< float >(), start, end, cstart, cend, std::ref( clstrLabel ), refineFlag );
				threads[ currentCidx ] = std::thread( threadTask );
			}
			else
			{
				auto threadTask = std::bind( assignCluster< PARAM_DIM >, std::ref( clstrHdlr ), inputdata.template cast< float >(), start, end, cstart, cend, std::ref( clstrLabel ), refineFlag );
				threads[ currentCidx ] = std::thread( threadTask );
			}			
			labelInter[ currentCidx ][ 0 ] = start;
			labelInter[ currentCidx ][ 1 ] =  end - start;
			idlethread.pop_back();
			start = start + PT_PER_THRD;
			end = ( end + PT_PER_THRD > inputdata.rows() ) ? inputdata.rows() : end + PT_PER_THRD;
		}
		for( int tidx = 0; tidx < THRD_NUM; tidx++ )
		{
			while( threads[ tidx ].joinable() )
			{
				threads[ tidx ].join();
				if( refineFlag == 0 )
				{
					clstrLabel.block( labelInter[ tidx ][ 0 ], 0, labelInter[ tidx ][ 1 ], 1 ) += clstrHdlr.size();
					clstrHdlr.newCluster( tempClstrHdlr[ tidx ] );
				}
				idlethread.push_back( tidx );
				std::cout << "CNUM: " << clstrHdlr.size() << std::endl;
			}
		}
	}
	std::cout << "ProcTime: " << (clock()-t1)/(double)(CLOCKS_PER_SEC) << " sec" << std::endl;
}


