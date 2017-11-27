#include "system_base.h"
#include "cv_imgproc.h"
#include "ros_msg.h"
#include "pcl_base.h"
#include "pcl_imgproc.h"

template< int PARAM_DIM >
class InitSetting
{
	public:
		float clusterThres;
		float factor;
		Eigen::Array< float, 1, PARAM_DIM > weight;
		Eigen::Array< float, 1, PARAM_DIM > std;
		std::vector< int > excepIdx;
		std::vector< int > crispbound;
		Eigen::Array< float, 1, PARAM_DIM > ignoreW;
};

template< int PARAM_DIM >
class SCCcluster
{
	public:
		/*******Constructor*******/
		SCCcluster< PARAM_DIM >(){}
		
		SCCcluster< PARAM_DIM >( InitSetting< PARAM_DIM > initSet )
		{
			init_param_ = initSet;
		}
				
		/*******Methods*******/
		int size(){ return mean_.rows(); }
		
		int groupNum(){ return groupMemberNum_.size(); }
		
		int groupNum( int gidx ){ return groupMemberNum_[ gidx ]; }
		
		std::vector< int > groupMemberNum(){ return groupMemberNum_; }
		
		int group( int cidx ){ return group_[ cidx ]; }
		
		std::vector< int > group(){ return group_; }

		InitSetting< PARAM_DIM > showInitSet(){ return init_param_; }
		
		Eigen::Array< float, 1, PARAM_DIM > mean( int cidx ){ return mean_.row( cidx ); }
		
		float mean( int cidx, int pidx ){ return mean_( cidx, pidx ); }
		
		Eigen::Array< float, Eigen::Dynamic, PARAM_DIM > mean(){ return mean_; }
		
		Eigen::Array< float, 1, PARAM_DIM > std( int idx ){ return std_.row( idx ); }
		
		float std( int cidx, int pidx ){ return std_( cidx, pidx ); }
		
		Eigen::Array< float, Eigen::Dynamic, PARAM_DIM > std(){ return std_; }
		
		Eigen::Array< float, 1, PARAM_DIM > weight(){ return init_param_.weight; }
		
		std::vector< uint32_t > member( int cidx ){ return indices[ cidx ]; }
		
		uint32_t member( int cidx, int idx ){ return indices[ cidx ][ idx ]; }
		
		std::vector< std::vector< uint32_t > > member(){ return indices; }
		
		void initialize( InitSetting< PARAM_DIM > inputSetting )
		{
			init_param_ = inputSetting;
		}
		
		void reset()
		{
			mean_ = Eigen::Array< float, Eigen::Dynamic, PARAM_DIM, Eigen::RowMajor >();
			std_ = Eigen::Array< float, Eigen::Dynamic, PARAM_DIM, Eigen::RowMajor >();
			indices.clear();
		}
		
		void kickout( int cidx, int idx )
		{
			indices[ cidx ].erase( indices[ cidx ].begin() + idx );
		}
		
		void pushin( int cidx, int idx )
		{
			indices[ cidx ].push_back( idx );
		}
		
		bool isIn( int cidx, int idx )
		{
			if( idx > indices[ cidx ].back() )
			{
				return false;
			}
			else
			{
				if( idx > indices[ cidx ][ floor( indices[ cidx ].size() / 2 ) ] )
				{
					for( int i = indices[ cidx ].size() - 1; i >= 0; i-- )
					{
						if( indices[ cidx ][ i ] == idx )
						{
							return true;
						}
					}
					return false;
				}
				else
				{
					for( int i = 0; i < indices[ cidx ].size(); i++ )
					{
						if( indices[ cidx ][ i ] == idx )
						{
							return true;
						}
					}
					return false;
				}
			}
		}

		void newCluster( int idx, Eigen::Array< float, 1, PARAM_DIM > point, int gidx = -1 )
		{
			mean_.conservativeResize( mean_.rows() + 1, PARAM_DIM );
			mean_.row( mean_.rows() - 1 ) = point;
			std_.conservativeResize( std_.rows() + 1, PARAM_DIM );
			std_.row( std_.rows() - 1 ) = init_param_.std;
			indices.push_back( std::vector< uint32_t >() );
			indices.back().push_back( idx );
			if( gidx >= 0 )
			{
				group_.push_back( gidx );
				groupMemberNum_[ gidx ] += 1;
			}
			else
			{
				group_.push_back( groupMemberNum_.size() );
				groupMemberNum_.push_back( 1 );
			}
		}
		
		void newCluster( SCCcluster appendClster )
		{
			int oldNum = mean_.rows();
			mean_.conservativeResize( oldNum + appendClster.size(), PARAM_DIM );
			mean_.block( oldNum, 0, appendClster.size(), mean_.cols() ) = appendClster.mean();
			std_.conservativeResize( oldNum + appendClster.size(), PARAM_DIM );
			std_.block( oldNum, 0, appendClster.size(), std_.cols() ) = appendClster.std();
			if( indices.size() == 0 )
			{
				indices.reserve( appendClster.member().capacity() );
				indices = appendClster.member();
				group_ = appendClster.group();
				groupMemberNum_ = appendClster.groupMemberNum();
			}
			else
			{
				indices.reserve( indices.size() + appendClster.member().capacity() );
				for( int i = 0; i < appendClster.size(); i++ )
				{
					indices.push_back( appendClster.member( i ) );
					group_.push_back( appendClster.group( i ) + groupMemberNum_.size() );
				}
				groupMemberNum_.reserve( groupMemberNum_.size() + appendClster.groupMemberNum().capacity() );
				groupMemberNum_.insert( groupMemberNum_.end(), appendClster.groupMemberNum().begin(), appendClster.groupMemberNum().end() );
			}
		}
				
		std::vector< float > computeSim( Eigen::Array< float, 1, PARAM_DIM > point, int Cstart = 0, int Cend = PARAM_DIM, int Rstart = 0, int Rend = -1 )
		{
			if( Rend == -1 )
			{
				Rend = mean_.rows();
			}
			float highestidx = -1, highestsim = -1, clusterSim = -1, middleSim = -1, groupSim = -1, update = -1;
			int rowN = ( Rend - Rstart );
			int colN = ( Cend - Cstart );
			
			if( rowN <= 0 || colN <= 0 )
				return std::vector< float >{ highestidx, highestsim, update };
				
			Eigen::Array< float, 1, Eigen::Dynamic > oneArray = Eigen::Array< float, 1, Eigen::Dynamic >::Ones( 1, colN );
			Eigen::Array< float, Eigen::Dynamic, Eigen::Dynamic > AllSim = ( ( Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic >::Ones( rowN, 1 ) * point.block( 0, Cstart, 1, colN ).matrix() ).array() - mean_.block( Rstart, Cstart, rowN, colN ) ).abs() / ( std_.block( Rstart, Cstart, rowN, colN ) * init_param_.clusterThres );
			Eigen::Array< float, 1, Eigen::Dynamic >  tempweight;
			int flag = -1;
			
			for( int cidx = Rstart; cidx < Rend; cidx++ )
			{
				flag = -1;
				for( int i = 0; i < init_param_.crispbound.size(); i++ )
				{
					if( abs( point( init_param_.crispbound[ i ] ) - mean( cidx, init_param_.crispbound[ i ] ) ) > std( cidx, init_param_.crispbound[ i ] ) )
					{
						flag = 0;
						break;
					}
				}
				if( flag == 0 )
				{
					continue;
				}
				for( int i = 0; i < init_param_.excepIdx.size(); i++ )
				{
					// If specified ECEPTION INDEX( excepIdx ) is zero, it computes a weighted distance from a part of parameters only
					if( init_param_.excepIdx[ i ] >= Cstart && init_param_.excepIdx[ i ] < Cend && ( point( init_param_.excepIdx[ i ] ) == 0 || mean_( cidx, init_param_.excepIdx[ i ] ) == 0 ) )
					{
						tempweight = ( init_param_.weight * init_param_.ignoreW ).block( 0, Cstart, 1, colN );
						flag = 1;
						break;
					}
				}
				if( flag == -1 )
				{
					tempweight = init_param_.weight.block( 0, Cstart, 1, colN );
				}
				clusterSim = ( ( oneArray - AllSim.row( cidx ) ) * tempweight ).sum() / tempweight.sum();
				groupSim = ( ( oneArray - AllSim.row( cidx ) / init_param_.factor ) * tempweight ).sum() / tempweight.sum();
				if( groupSim > 0 && groupSim > highestsim )
				{
					highestidx = cidx;
					highestsim = groupSim;
					if( clusterSim > 0 )
					{
						update = 1;
					}
				}
			}
			return std::vector< float >{ highestidx, highestsim, update };
		}
		
		void updateCluster( int cidx, int idx, Eigen::Array< float, 1, PARAM_DIM > point )
		{
			Eigen::Array< float, 1, PARAM_DIM > n_mean;
			Eigen::Array< float, 1, PARAM_DIM > n_std;
			Eigen::Array< float, 1, PARAM_DIM > remainIdx = Eigen::Array< float, 1, PARAM_DIM >::Ones() - init_param_.ignoreW;
			
			int flag = -1;
			// Because of ECEPTION INDEX( excepIdx ), we update mean & std partly
			for( int i = 0; i < init_param_.excepIdx.size(); i++ )
			{
				// If specified ECEPTION INDEX( excepIdx ) is zero, it computes a weighted distance from a part of parameters only
				if( point( init_param_.excepIdx[ i ] ) == 0 )
				{
					Eigen::Array< float, 1, PARAM_DIM > point_t = ( mean_.row( cidx ) * remainIdx ) + ( point * init_param_.ignoreW );
					n_mean = ( mean_.row( cidx ) * indices[ cidx ].size() + point_t ) /  ( indices[ cidx ].size() + 1 );
					std_.row( cidx ) = ( ( ( std_.row( cidx ) - init_param_.std ).square() * indices[ cidx ].size() + mean_.row( cidx ).square() * indices[ cidx ].size() + point_t.square() - ( indices[ cidx ].size() + 1 ) * n_mean.square() ) / ( indices[ cidx ].size() + 1 ) ).abs().sqrt() + init_param_.std;
					mean_.row( cidx ) = n_mean;
					flag = 1;
					break;
				}
				else if( mean_( cidx, init_param_.excepIdx[ i ] ) == 0 )
				{
					// If exception occured to cluster but not to input point, we use point value to set new cluster value
					n_mean = ( point * remainIdx ) + ( ( ( mean_.row( cidx ) * indices[ cidx ].size() + point ) / ( indices[ cidx ].size() + 1 ) ) * init_param_.ignoreW );
					std_.row( cidx ) = ( ( ( std_.row( cidx ) - init_param_.std ).square() * indices[ cidx ].size() + mean_.row( cidx ).square() * indices[ cidx ].size() + point.square() - ( indices[ cidx ].size() + 1 ) * n_mean.square() ) / ( indices[ cidx ].size() + 1 ) ).abs().sqrt() + init_param_.std;
					mean_.row( cidx ) = n_mean;
					flag = 1;
					break;
				}
			}
			if( flag == -1 )
			{
				n_mean = ( mean_.row( cidx ) * indices[ cidx ].size() + point ) /  ( indices[ cidx ].size() + 1 );
				std_.row( cidx ) = ( ( ( std_.row( cidx ) - init_param_.std ).square() * indices[ cidx ].size() + mean_.row( cidx ).square() * indices[ cidx ].size() + point.square() - ( indices[ cidx ].size() + 1 ) * n_mean.square() ) / ( indices[ cidx ].size() + 1 ) ).abs().sqrt() + init_param_.std;
				mean_.row( cidx ) = n_mean;
			}
			indices[ cidx ].push_back( idx );
		}
		
		void refineCluster( Eigen::Array< int, Eigen::Dynamic, 1 > &cluster, int idx, Eigen::Array< float, 1, PARAM_DIM > point )
		{
			if( indices[ cluster[ idx ] ].size() == 1 )
			{
				indices.erase( indices.begin() + cluster( idx ) );
				group_.erase( group_.begin() + cluster( idx ) );
				std_.block( cluster[ idx ], 0, std_.rows() - cluster[ idx ] - 1, PARAM_DIM) = std_.block( cluster[ idx ] + 1, 0, std_.rows() - cluster[ idx ] - 1, PARAM_DIM);
				std_.conservativeResize( std_.rows() - 1, PARAM_DIM);
				mean_.block( cluster[ idx ], 0, mean_.rows() - cluster[ idx ] - 1, PARAM_DIM) = mean_.block( cluster[ idx ] + 1, 0, mean_.rows() - cluster[ idx ] - 1, PARAM_DIM);
				mean_.conservativeResize( mean_.rows() - 1, PARAM_DIM);
				for( int i = 0; i < cluster.rows(); i++ )
				{
					if( cluster[ i ] > cluster[ idx ] )
					{
						cluster[ i ] -= 1;
					}
				}
			}
			else
			{
				Eigen::Array< float, 1, PARAM_DIM > n_mean;
				Eigen::Array< float, 1, PARAM_DIM > n_std;
				Eigen::Array< float, 1, PARAM_DIM > remainIdx = Eigen::Array< float, 1, PARAM_DIM >::Ones() - init_param_.ignoreW;
				
				int flag = -1;
				// Because of ECEPTION INDEX( excepIdx ), we update mean & std partly
				for( int i = 0; i < init_param_.excepIdx.size(); i++ )
				{
					// If specified ECEPTION INDEX( excepIdx ) is zero, it computes a weighted distance from a part of parameters only
					if( point( init_param_.excepIdx[ i ] ) == 0 )
					{
						Eigen::Array< float, 1, PARAM_DIM > point_t = ( mean_.row( cluster[ idx ] ) * remainIdx ) + ( point * init_param_.ignoreW );
						n_mean = ( mean_.row( cluster[ idx ] ) * indices[ cluster[ idx ] ].size() - point_t ) /  ( indices[ cluster[ idx ] ].size() - 1 );
						std_.row( cluster[ idx ] ) = ( ( ( std_.row( cluster[ idx ] ) - init_param_.std ).square() * indices[ cluster[ idx ] ].size() + mean_.row( cluster[ idx ] ).square() * indices[ cluster[ idx ] ].size() - point.square() - ( indices[ cluster[ idx ] ].size() - 1 ) * n_mean.square() ) / ( indices[ cluster[ idx ] ].size() - 1 ) ).abs().sqrt() + init_param_.std;
						mean_.row( cluster[ idx ] ) = n_mean;
						flag = 1;
						break;
					}
					else if( mean_( cluster[ idx ], init_param_.excepIdx[ i ] ) == 0 )
					{
						// If exception occured to cluster but not to input point, we use point value to set new cluster value
						n_mean = ( mean_.row( cluster[ idx ] ) * indices[ cluster[ idx ] ].size() - point ) /  ( indices[ cluster[ idx ] ].size() - 1 );
						std_.row( cluster[ idx ] ) = ( ( ( std_.row( cluster[ idx ] ) - init_param_.std ).square() * indices[ cluster[ idx ] ].size() + mean_.row( cluster[ idx ] ).square() * indices[ cluster[ idx ] ].size() - point.square() - ( indices[ cluster[ idx ] ].size() - 1 ) * n_mean.square() ) / ( indices[ cluster[ idx ] ].size() - 1 ) ).abs().sqrt() + init_param_.std;
						mean_.row( cluster[ idx ] ) = n_mean;
						flag = 1;
						break;
					}
				}
				if( flag == -1 )
				{
					n_mean = ( mean_.row( cluster[ idx ] ) * indices[ cluster[ idx ] ].size() + point ) /  ( indices[ cluster[ idx ] ].size() + 1 );
					std_.row( cluster[ idx ] ) = ( ( ( std_.row( cluster[ idx ] ) - init_param_.std ).square() * indices[ cluster[ idx ] ].size() + mean_.row( cluster[ idx ] ).square() * indices[ cluster[ idx ] ].size() - point.square() - ( indices[ cluster[ idx ] ].size() - 1 ) * n_mean.square() ) / ( indices[ cluster[ idx ] ].size() - 1 ) ).abs().sqrt() + init_param_.std;
					mean_.row( cluster[ idx ] ) = n_mean;
				}
				for( int i = 0; i < indices[ cluster[ idx ] ].size(); i++ )
				{
					if( indices[ cluster[ idx ] ][ i ] == idx )
					{
						indices[ cluster[ idx ] ].erase( indices[ cluster[ idx ] ].begin() + i );
						break;
					}
				}
			}
		}
		
	private:
		InitSetting< PARAM_DIM > init_param_;
		std::vector< int > groupMemberNum_;
		std::vector< int > group_;
		std::vector< std::vector< uint32_t > > indices;
		Eigen::Array< float, Eigen::Dynamic, PARAM_DIM, Eigen::RowMajor > mean_;
		Eigen::Array< float, Eigen::Dynamic, PARAM_DIM, Eigen::RowMajor > std_;
};
