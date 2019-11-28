struct PointCloud
{
} 	(
	yarp.name = "yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>"
	yarp.includefile = "yarp/sig/PointCloud.h"
	)

struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )


service PCR_IDL {
    Bottle get_point_cloud(1:string objectToFind);
    Bottle get_point_cloud_from_3D_position(1:double x, 2:double y, 3:double z);
    Bottle get_point_cloud_from_image_position(1:double u, 2:double v);
    bool stream_one(1:string objectToFind);
    bool stream_start(1:string objectToFind);
    bool stream_stop();
    bool dump_one(1:string objectToFind, 2:string format);
    bool dump_scene(1:string format);
    bool set_period(1:double modulePeriod);
}
