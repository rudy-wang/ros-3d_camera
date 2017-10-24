#include <pcl/common/common_headers.h>
#include <librealsense/rs.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace std;

class PointCluster {
public:
	PointCluster();
	PointCluster(float, float, float, float, float, float);
	float similarity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	void push(uint32_t, float, float, float, float, float, float);
private:
	uint32_t size;
	float center[6];
	float depth;
	uint8_t noisyPoints[];
};
	
