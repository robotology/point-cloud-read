# point-cloud-read

Acquire point clouds of specific objects in the scene in order to save or stream them.

### Dependencies
- [Yarp](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [IOL](https://github.com/robotology/iol)

### Installation
```
git clone https://github.com/fbottarel/point-cloud-read.git
cd point-cloud-read
mkdir build && cd build
cmake ../
make install
```
### Usage
The user might want to use this module when they want to acquire point clouds of objects currently in the scene in front of the robot. ` point-cloud-read` takes care of interfacing with the modules in [IOL](https://github.com/robotology/iol) to determine whether the queried object is in the scene, where it is, and retrieve the point cloud from the robot's stereo vision system. 

Once the module is running and connected (the code is comprehensive of a .xml application file for `yarpmanager`) it can be interacted with using the ports it opens: 
- `/pointCloudRead/rpc` exposes an rpc interface to trigger acquisitions. See below for details
- `/pointCloudRead/pointCloud:o` is a `yarp::os::BufferedPort` where the module publishes the acquired point clouds as `<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>>`

Typing `help` as rpc command returns a list of available commands, here briefly listed.
- `set_period periodInSeconds` sets the streaming period to `periodInSeconds` seconds. Default `periodInSeconds` is 1.0
- `stream_start objectName` starts the point cloud stream of an object recognized as `objectName` in the scene with period `periodInSeconds`. Empty point clouds are streamed if no `objectName` is found in the scene
- `stream_stop` stops the point cloud stream
- `stream_one objectName` triggers a one-shot acquisition and streaming. If no `objectName` is found in the scene, the command fails 
- `dump_one objectName` commands the module to acquire a point cloud and dump it to a .off file in the format `objectName_point_cloud_XXX.off`, where `XXX` is a zero-filled number that increases when more point clouds of the same object are dumped. Such file is easily readable with programs such as MeshLab.





