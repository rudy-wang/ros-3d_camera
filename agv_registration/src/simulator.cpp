#include "simulator.h"

int main(int argc, char** argv)
{
	cv::Mat guiFrame = cv::Mat( FRAME_H, FRAME_W, CV_8UC3 );
	
	cv::namedWindow( WINDOW_NAME );
	cvui::init( WINDOW_NAME, 1 );
	
	int objectType = -1;
	int prev_objectType = -1;
	float mouseAction = -1;
	std::vector< std::vector< float > > objectHandler;
	objectHandler.clear();
	
	while( true )
	{
		defaultLayout( guiFrame, objectType, mouseAction, cargoW, uagvW );
		if( mouseAction != 5 )
		{
			showObject( guiFrame, objectType, objectHandler );
		}
		if( objectType == 5 )
		{
			if( objectHandler.size() > 0 )
			{
				if( objectHandler.back().front() == -1 )
				{
					objectHandler.pop_back();
				}
				objectHandler.pop_back();
			} 
			objectType = prev_objectType;
		}
		obtainObject( guiFrame, objectHandler );
		if( cvui::mouse( cvui::RIGHT_BUTTON, cvui::DOWN ) && polyEnd )
		{
			objectType = -1;
		}
		prev_objectType = objectType;
		cvui::update();
		cv::imshow( WINDOW_NAME, guiFrame );	
	}
	
	objectHandler.clear();
	return 0;
}


