# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint8 seg_id         # segment ID

float32[] x        	 # pcl x
float32[] y        	 # pcl y
float32[] z        	 # pcl z

uint8[] r 		     # pcl r
uint8[] g 		     # pcl g
uint8[] b 		     # pcl b

uint32 width         # pcl width
uint32 height        # pcl height
uint32 size          # pcl points number
