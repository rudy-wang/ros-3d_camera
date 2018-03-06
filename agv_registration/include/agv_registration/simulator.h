#include "system_base.h"
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME		"Registration Simulator"
#define FRAME_PADDING		30
#define BUTTON_PADDING	10
#define TRANS_RATIO			0.5
#define FRAME_W				1200
#define FRAME_H 				880
#define DRAWAREA_W		1000
#define DRAWAREA_H 		820
#define BUTTON_W			135
#define BUTTON_H			50
#define REFLECT_L				4


int cargoW = 100;
int uagvW = 100;
int drawAreaP1[ 2 ] = { FRAME_PADDING, FRAME_PADDING + 20 };
int drawAreaP2[ 2 ] = { FRAME_PADDING + DRAWAREA_W, FRAME_PADDING + DRAWAREA_H };
cv::Point startP = cv::Point( 0, 0 );
bool polyEnd = true;

void defaultLayout( cv::Mat &guiFrame, int &objectType, float &mouseAction, int &cargoW, int &uagvW )
{
	// create main GUI, fill with color
	guiFrame = cv::Scalar( 49, 52, 49 );

	// create area for drawing
	cvui::window( guiFrame, FRAME_PADDING, FRAME_PADDING, DRAWAREA_W, DRAWAREA_H, "Drawing Area" );		
	mouseAction = cvui::iarea( FRAME_PADDING, FRAME_PADDING + 20, DRAWAREA_W, DRAWAREA_H - 20 );
		
	// create control panel
	cvui::beginColumn( guiFrame, FRAME_W - BUTTON_W - FRAME_PADDING / 2, FRAME_PADDING, -1, -1, BUTTON_PADDING / 2 );
		// create buttons
		cvui::beginColumn( -1, -1, BUTTON_PADDING );
			if( cvui::button( BUTTON_W, BUTTON_H, "(&1)Add Cargo" ) )
			{
				objectType = 0;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&2)Draw Block" ) )
			{
				objectType = 1;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&3)Draw Circle" ) )
			{
				objectType = 2;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&4)Draw with Line" ) )
			{
				objectType = 3;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&5)Draw with Path" ) )
			{
				objectType = 4;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&Z)Undo" ) )
			{
				objectType = 5;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&Q)Quit" ) )
			{
				exit( 0 );
			}
		cvui::endColumn();
		
		// create other objects
		cvui::beginColumn( -1, -1, BUTTON_PADDING / 2 );
			cvui::text( "Cargo Width", 0.4, 0x00ff00 );
			cvui::counter( &cargoW );
			cvui::trackbar( BUTTON_W, &cargoW, 0, 200 );
			cvui::text( "UAGV Width", 0.4, 0x00ff00 );
			cvui::counter( &uagvW );
			cvui::trackbar( BUTTON_W, &uagvW, 0, 200 );
			cvui::text( "Draw Action:", 0.4, 0x00ff00 );
			cvui::printf( 0.4, 0x00ff00, "  %s", ( mouseAction == 2 ? "Press" : ( mouseAction == 3 ? "Lay On" : ( mouseAction == 4 ? "Idle" : "Out of Area" ) ) ) );
			cvui::text( "Using Object:", 0.4, 0x00ff00 );
			cvui::printf( 0.4, 0x00ff00, "  %s", ( objectType == 0 ? "Cargo" : ( objectType == 1 ? "Block" : ( objectType == 2 ? "Circle" : ( objectType == 3 ? "Line" : ( objectType == 4 ? "Path" : "None" ) ) ) ) ) );
		cvui::endColumn();
		cvui::beginColumn( -1, -1, BUTTON_PADDING );
			if( cvui::button( BUTTON_W, BUTTON_H, "(&A)Set UAGV" ) )
			{
				objectType = 6;
			}
			if( cvui::button( BUTTON_W, BUTTON_H, "(&S)Start" ) )
			{
				objectType = 6;
			}
		cvui::endColumn();
	cvui::endColumn();
	cvui::text( guiFrame, DRAWAREA_W + FRAME_PADDING - 100, FRAME_PADDING + DRAWAREA_H + 10, "1 pixel : 2 cm", 0.4, 0x00ff00 );
}
void addObject( cv::Mat &frame, int objectType, int x, int y, std::vector< float > points )
{
	if( objectType == 4 )
	{
		cv::Point tmp[ 1 ][ ( points.size() + 1 ) / 2 ];
		tmp[ 0 ][ 0 ] = cv::Point( x, y );
		for( int i = 1; i < points.size(); i+=2 )
		{
			tmp[ 0 ][ ( i + 1 ) / 2 ] = cv::Point( ( int ) points[ i ], ( int ) points[ i + 1 ] );
		}
		const cv::Point* ppt[ 1 ] = { tmp[ 0 ] };
		int npt[] = { ( int )( ( points.size() + 1 ) / 2 ) };
		cv::fillPoly( frame, ppt, npt, 1, true, 0x000000 );
	}
}
void addObject( cv::Mat &frame, int objectType, int x, int y, int objectW, int objectH = -1 )
{
	if( objectH == -1 )
	{
		objectH = objectW;
	}
	switch( objectType )
	{
		case 0:
			cvui::rect( frame, x - 0.5 * TRANS_RATIO * objectW, y - 0.5 * TRANS_RATIO * objectH, TRANS_RATIO * objectW, TRANS_RATIO * objectH, 0x772222 );
			cvui::rect( frame, x - 0.5 * TRANS_RATIO * objectW, y - 0.5 * TRANS_RATIO * objectH, TRANS_RATIO * REFLECT_L, TRANS_RATIO * REFLECT_L, 0xffffff );
			cvui::rect( frame, x + TRANS_RATIO * ( 0.5 * objectW - REFLECT_L ), y - 0.5 * TRANS_RATIO * objectH, TRANS_RATIO * REFLECT_L, TRANS_RATIO * REFLECT_L, 0xffffff );
			cvui::rect( frame, x - 0.5 * TRANS_RATIO * objectW, y + TRANS_RATIO * ( 0.5 * objectH - REFLECT_L ), TRANS_RATIO * REFLECT_L, TRANS_RATIO * REFLECT_L, 0xffffff );
			cvui::rect( frame, x + TRANS_RATIO * ( 0.5 * objectW - REFLECT_L ), y + TRANS_RATIO * ( 0.5 * objectH - REFLECT_L ), TRANS_RATIO * REFLECT_L, TRANS_RATIO * REFLECT_L, 0xffffff );
		break;
		case 1:
			cv::rectangle( frame, cv::Point( x, y ), cv::Point( objectW, objectH ), 0x000000, CV_FILLED );
		break;
		case 2:
			cv::circle( frame, cv::Point( x, y ), objectW, 0x000000, CV_FILLED );
		break;
		case 3:
			cv::line( frame, cv::Point( x, y ), cv::Point( objectW, objectH ), 0x000000 );
		break;
		default:
		break;
	}
}

void showObject( cv::Mat &frame, int objectType, std::vector< std::vector< float > > &objectHandler )
{
	switch( objectType )
	{
		case 0:
			if( cvui::mouse().x > drawAreaP1[ 0 ] + 0.5 * TRANS_RATIO * cargoW && cvui::mouse().y > drawAreaP1[ 1 ] + 0.5 * TRANS_RATIO * cargoW && cvui::mouse().x < drawAreaP2[ 0 ] - 0.5 * TRANS_RATIO * cargoW && cvui::mouse().y < drawAreaP2[ 1 ] - 0.5 * TRANS_RATIO * cargoW )
			{
				addObject( frame, objectType, cvui::mouse().x, cvui::mouse().y, cargoW );
			}
			if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) )
			{
				std::vector< float > tmp = { objectType, cvui::mouse().x, cvui::mouse().y, cargoW };
				objectHandler.push_back( tmp );
			}
		break;
		case 1:
			if( cvui::mouse( cvui::LEFT_BUTTON, cvui::DOWN ) )
			{
				startP = cv::Point( cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] )
			{
				addObject( frame, objectType, startP.x, startP.y, cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && abs( cvui::mouse().x - startP.x ) > 0 && abs( cvui::mouse().y - startP.y ) > 0 && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] )
			{
				std::vector< float > tmp = { objectType, startP.x, startP.y, cvui::mouse().x, cvui::mouse().y };
				objectHandler.push_back( tmp );
				startP = cv::Point( 0, 0 );
			}
		break;
		case 2:
			if( cvui::mouse( cvui::LEFT_BUTTON, cvui::DOWN ) )
			{
				startP = cv::Point( cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) )
			{
				int tempX = cvui::mouse().x - startP.x;
				int tempY = cvui::mouse().y - startP.y;
				int radious = round( sqrt( pow( tempX, 2 ) + pow( tempY, 2 ) ) );
				if( !( startP.x - radious > drawAreaP1[ 0 ] && startP.x + radious < drawAreaP2[ 0 ] && startP.y - radious > drawAreaP1[ 1 ] && startP.y + radious < drawAreaP2[ 1 ] ) )
				{
					int tempR = startP.x - drawAreaP1[ 0 ];
					if( drawAreaP2[ 0 ] - startP.x < tempR ){ tempR = drawAreaP2[ 0 ] - startP.x; }
					if( drawAreaP2[ 1 ] - startP.y < tempR ){ tempR = drawAreaP2[ 1 ] - startP.y; }
					if( startP.y - drawAreaP1[ 1 ] < tempR ){ tempR = startP.y - drawAreaP1[ 1 ]; }
					radious = tempR;
				}
				addObject( frame, objectType, startP.x, startP.y, radious );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && abs( cvui::mouse().x - startP.x ) > 0 && abs( cvui::mouse().y - startP.y ) > 0 )
			{
				int tempX = cvui::mouse().x - startP.x;
				int tempY = cvui::mouse().y - startP.y;
				int radious = round( sqrt( pow( tempX, 2 ) + pow( tempY, 2 ) ) );
				if( !( startP.x - radious > drawAreaP1[ 0 ] && startP.x + radious < drawAreaP2[ 0 ] && startP.y - radious > drawAreaP1[ 1 ] && startP.y + radious < drawAreaP2[ 1 ] ) )
				{
					int tempR = startP.x - drawAreaP1[ 0 ];
					if( drawAreaP2[ 0 ] - startP.x < tempR ){ tempR = drawAreaP2[ 0 ] - startP.x; }
					if( drawAreaP2[ 1 ] - startP.y < tempR ){ tempR = drawAreaP2[ 1 ] - startP.y; }
					if( startP.y - drawAreaP1[ 1 ] < tempR ){ tempR = startP.y - drawAreaP1[ 1 ]; }
					radious = tempR;
				}
				std::vector< float > tmp = { objectType, startP.x, startP.y, radious };
				objectHandler.push_back( tmp );
				startP = cv::Point( 0, 0 );
			}
		break;
		case 3:
			if( cvui::mouse( cvui::LEFT_BUTTON, cvui::DOWN ) )
			{
				startP = cv::Point( cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] )
			{
				addObject( frame, objectType, startP.x, startP.y, cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && abs( cvui::mouse().x - startP.x ) > 0 && abs( cvui::mouse().y - startP.y ) > 0 && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] )
			{
				std::vector< float > tmp = { objectType, startP.x, startP.y, cvui::mouse().x, cvui::mouse().y };
				objectHandler.push_back( tmp );
				startP = cv::Point( 0, 0 );
			}
		break;
		case 4:
			if( cvui::mouse( cvui::LEFT_BUTTON, cvui::DOWN ) && polyEnd )
			{
				startP = cv::Point( cvui::mouse().x, cvui::mouse().y );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) && ( abs( cvui::mouse().x - startP.x ) > 0 || abs( cvui::mouse().y - startP.y ) > 0 ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] && polyEnd )
			{
				addObject( frame, objectType, startP.x, startP.y, std::vector< float >( { -1, cvui::mouse().x, cvui::mouse().y } ) );
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && ( abs( cvui::mouse().x - startP.x ) > 0 || abs( cvui::mouse().y - startP.y ) > 0 ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ]  && polyEnd )
			{
				std::vector< float > tmp = { objectType, startP.x, startP.y };
				objectHandler.push_back( tmp );
				tmp = { -1, cvui::mouse().x, cvui::mouse().y, cvui::mouse().x, cvui::mouse().y };
				objectHandler.push_back( tmp );
				polyEnd = false;
			}
			else if( cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ]  && !polyEnd )
			{
				if( abs( cvui::mouse().x - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 4 ] ) > 0 || abs( cvui::mouse().y - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 3 ] ) > 0 )
				{
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().x );
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().y );
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().x );
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().y );
					polyEnd = false;
				}
			}
			else if( !cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) && !cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && !cvui::mouse( cvui::RIGHT_BUTTON, cvui::CLICK ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] && !polyEnd )
			{
				if( abs( cvui::mouse().x - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 2 ] ) > 0 || abs( cvui::mouse().y - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 1 ] ) > 0 )
				{
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().x );
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().y );
					polyEnd = false;
				}
			}
			else if( !cvui::mouse( cvui::LEFT_BUTTON, cvui::IS_DOWN ) && !cvui::mouse( cvui::LEFT_BUTTON, cvui::CLICK ) && cvui::mouse( cvui::RIGHT_BUTTON, cvui::CLICK ) && cvui::mouse().x > drawAreaP1[ 0 ] && cvui::mouse().x < drawAreaP2[ 0 ] && cvui::mouse().y > drawAreaP1[ 1 ] && cvui::mouse().y < drawAreaP2[ 1 ] && !polyEnd )
			{
				if( abs( cvui::mouse().x - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 4 ] ) > 0 || abs( cvui::mouse().y - objectHandler[ objectHandler.size() - 1 ][ objectHandler[ objectHandler.size() - 1 ].size() - 3 ] ) > 0 )
				{
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].pop_back();
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().x );
					objectHandler[ objectHandler.size() - 1 ].push_back( cvui::mouse().y );
					startP = cv::Point( 0, 0 );
					polyEnd = true;
				}
			}
		break;
		default:
		break;
	}
}

void obtainObject( cv::Mat &frame, std::vector< std::vector< float > > &objectHandler )
{
	for( int i = 0; i < objectHandler.size(); i++ )
	{
		if( objectHandler[ i ][ 0 ] >= 0 && objectHandler[ i ][ 0 ] < 4 )
		{
			addObject( frame, ( int ) objectHandler[ i ][ 0 ], objectHandler[ i ][ 1 ], objectHandler[ i ][ 2 ], objectHandler[ i ][ 3 ], objectHandler[ i ].size() > 4	 ? objectHandler[ i ][ 4 ] : -1 );
		}
		else if( objectHandler[ i ][ 0 ] == 4 )
		{
			addObject( frame, ( int ) objectHandler[ i ][ 0 ], objectHandler[ i ][ 1 ], objectHandler[ i ][ 2 ], objectHandler[ i + 1 ]  );
			i += 1;
		}
	}
}
