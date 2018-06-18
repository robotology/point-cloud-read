struct PointCloud
{
} 	(
	yarp.name = "yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>"
	yarp.includefile = "yarp/sig/PointCloud.h"
	)


service PCR_IDL {
    PointCloud get_point_cloud(1:string objectToFind);
    bool stream_one(1:string objectToFind);
    bool stream_start(1:string objectToFind);
    bool stream_stop();
    bool dump_one(1:string objectToFind, 2:string format);
    bool set_period(1:double modulePeriod);
}
