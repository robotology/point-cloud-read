/*
 *  Module to retrieve point clouds from SFM module and dump them to
 *  OFF files or stream them. Interfaces with OPC to retrieve the position
 *  of a desired object and retrieves a point cloud from SFM.
 *
 *  Author: Fabrizio Bottarel - <fabrizio.bottarel@iit.it>
 *
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/PointCloud.h>

#ifdef POINTCLOUDREAD_USES_PCL
#include <yarp/pcl/Pcl.h>
#endif

#include <string>
#include <iostream>
#include <fstream>
#include <deque>
#include <algorithm>

#include <climits>

#include <src/PCR_IDL.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**************************************************************/

/*
 * Define a class to represent points in the xyz space with color
 * attributes.
*/

class Point3DRGB
{
public:
    double x, y, z;
    unsigned char r, g, b;

    Point3DRGB(double x, double y, double z, unsigned char r, unsigned char g, unsigned char b)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->r = r;
        this->g = g;
        this->b = b;
    }

    Vector toYarpVectorRGB()
    {
        Vector point_vec(6);

        point_vec(0) = x;
        point_vec(1) = y;
        point_vec(2) = z;
        point_vec(3) = r;
        point_vec(4) = g;
        point_vec(5) = b;

        return point_vec;
    }

    Vector toYarpVector()
    {
        Vector point_vec(3);

        point_vec(0) = x;
        point_vec(1) = y;
        point_vec(2) = z;

        return point_vec;
    }

    DataXYZRGBA toYarpXYZRGBA()
    {
        DataXYZRGBA point;

        point.x = this->x;
        point.y = this->y;
        point.z = this->z;
        point.r = this->r;
        point.g = this->b;
        point.b = this->b;

        //  set the alpha channel to one
        point.a = UCHAR_MAX;

        return point;

    }

};

class PointCloudReadModule: public RFModule,
                            public PCR_IDL
{
protected:

    enum class OpMode
    {
        OP_MODE_STREAM_ONE,
        OP_MODE_STREAM_MANY,
        OP_MODE_DUMP_ONE,
        OP_MODE_NONE
    };

    RpcServer inCommandPort;
    RpcClient outCommandOPC;
    RpcClient outCommandSFM;
    RpcClient outCommandSegm;

    BufferedPort<PointCloud<DataXYZRGBA>> outPort;
    BufferedPort<ImageOf<PixelRgb>> inImgPort;

    BufferedPort<Bottle> outBottlePointCloud;

    Mutex mutex;

    string moduleName;
    string objectToFind;
    string baseDumpFileName;

    double moduleUpdatePeriod;

    OpMode operationMode;

    bool attach(RpcServer &source) override
    {
        return this->yarp().attachAsServer(source);
    }

    bool retrieveObjectBoundingBox(const string objName, Vector &top_left_xy, Vector &bot_right_xy)
    {
        //  get object bounding box from OPC module given object name
        top_left_xy.resize(2);
        bot_right_xy.resize(2);

        //  command message format: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" <val1>) ...)
        Bottle cmd, reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList();
        Bottle &cond_1=content.addList();
        cond_1.addString("entity");
        cond_1.addString("==");
        cond_1.addString("object");
        content.addString("&&");
        Bottle &cond_2=content.addList();
        cond_2.addString("name");
        cond_2.addString("==");
        cond_2.addString(objName);
        outCommandOPC.write(cmd,reply);

        //  reply message format: [nack]; [ack] ("id" (<num0> <num1> ...))
        if (reply.size()>1)
        {
            //  verify that first element is "ack"
            if (reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                //  get list of all id's of objects named objName
                if (Bottle *idField = reply.get(1).asList())
                {
                    if (Bottle *idValues = idField->get(1).asList())
                    {
                        //  if there are more objects under the same name, pick the first one
                        int id = idValues->get(0).asInt();

                        //  get the actual bounding box
                        //  command message format:  [get] (("id" <num>) (propSet ("prop0" "prop1" ...)))
                        cmd.clear();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &content = cmd.addList();
                        Bottle &list_bid = content.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id);
                        Bottle &list_propSet = content.addList();
                        list_propSet.addString("propSet");
                        Bottle &list_items = list_propSet.addList();
                        list_items.addString("position_2d_left");
                        Bottle replyProp;
                        outCommandOPC.write(cmd,replyProp);

                        //reply message format: [nack]; [ack] (("prop0" <val0>) ("prop1" <val1>) ...)
                        if (replyProp.get(0).asVocab() == Vocab::encode("ack"))
                        {
                            if (Bottle *propField = replyProp.get(1).asList())
                            {
                                if (Bottle *position_2d_bb = propField->find("position_2d_left").asList())
                                {
                                    //  position_2d_left contains x,y of top left and x,y of bottom right
                                    top_left_xy(0)  = position_2d_bb->get(0).asInt();
                                    top_left_xy(1)  = position_2d_bb->get(1).asInt();
                                    bot_right_xy(0) = position_2d_bb->get(2).asInt();
                                    bot_right_xy(1) = position_2d_bb->get(3).asInt();
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        yError() << "RPC to OPC returned unexpected reply";
        return false;

    }

    bool retrieveObjectPointCloud(deque<Point3DRGB> &objectPointCloud)
    {
        //  get object bounding box given object name

        Vector bb_top_left, bb_bot_right;

        if (retrieveObjectBoundingBox(objectToFind, bb_top_left, bb_bot_right))
        {
            int width   = abs(bb_bot_right(0) - bb_top_left(0)) + 1;
            int height  = abs(bb_bot_right(1) - bb_top_left(1)) + 1;

            //  get list of points that belong to the object from lbpextract
            //  command message format: [get_component_around x y]
            int center_bb_x = bb_top_left(0) + width/2;
            int center_bb_y = bb_top_left(1) + (bb_bot_right(1) - bb_top_left(1))/2;

            Bottle cmdSeg, replySeg;
            cmdSeg.addString("get_component_around");
            cmdSeg.addInt(center_bb_x);
            cmdSeg.addInt(center_bb_y);

            if (!outCommandSegm.write(cmdSeg,replySeg)){
                yError() << "Could not write to segmentation RPC port";
                return false;
            }

            yDebug() << "Segmented point list obtained from segmentation module";

            //  lbpExtract replies with a list of points
            if (replySeg.size() < 1)
                {
                yError() << "Empty point list retrieved from segmentation module!" ;
                return false;
                }

            Bottle *pointList = replySeg.get(0).asList();

            if (pointList->size() > 0){

                //  each point is a list of 2 coordinates
                //  build one list of points to query SFM for 3d coordinates
                Bottle cmdSFM, replySFM;
                cmdSFM.addString("Points");

                yDebug() << "Retrieved " << pointList->size() << " 2D points.";

                for (int point_idx=0; point_idx < pointList->size(); point_idx++)
                {
                Bottle *point2D = pointList->get(point_idx).asList();
                cmdSFM.addInt(point2D->get(0).asInt());
                cmdSFM.addInt(point2D->get(1).asInt());
                }

                //yDebug() << "Query: " << cmdSFM.toString();

                if (!outCommandSFM.write(cmdSFM, replySFM))
                {
                    yError() << "Could not write to SFM RPC port";
                    return false;
                }

                yDebug() << "3D point list obtained from SFM";

                //  acquire image from camera input
                ImageOf<PixelRgb> *inCamImg = inImgPort.read();

                yDebug() << "Image obtained from camera stream";

                //  empty point cloud
                objectPointCloud.clear();

                yDebug() << "Reply from SFM obtained. Cycling through " << pointList->size() << " 3D points...";

                for (int point_idx = 0; point_idx < replySFM.size()/3; point_idx++)
                {
                    double x = replySFM.get(point_idx*3).asDouble();   // x value
                    double y = replySFM.get(point_idx*3+1).asDouble();   // y value
                    double z = replySFM.get(point_idx*3+2).asDouble();   // z value

                    //yDebug() << x << y << z;

                    //  0 0 0 points are invalid and must be discarded
                    if (x==0.0 && y==0.0 && z==0.0)
                        continue;

                    //  fetch rgb from image according to 2D coordinates
                    Bottle *point2D = pointList->get(point_idx).asList();
                    PixelRgb point_rgb = inCamImg->pixel(point2D->get(0).asInt(), point2D->get(1).asInt());

                    objectPointCloud.push_back(Point3DRGB(x, y, z, point_rgb.r, point_rgb.g, point_rgb.b));
                }

                yInfo() << "Point cloud retrieved: " << objectPointCloud.size() << " points stored.";

                return true;
            }
            else
            {
                yError() << "Empty point cloud retrieved for object " << objectToFind;
                return false;
            }

        }
        else
        {
            yError() << "Could not retrieve bounding box for object " << objectToFind;
            return false;
        }
    }

    bool retrieveObjectPointCloud(Matrix &objectPointCloud)
    {
        //  get object bounding box given object name

        Vector bb_top_left, bb_bot_right;

        if (retrieveObjectBoundingBox(objectToFind, bb_top_left, bb_bot_right))
        {
            int width   = abs(bb_bot_right(0) - bb_top_left(0)) + 1;
            int height  = abs(bb_bot_right(1) - bb_top_left(1)) + 1;

            //  get point cloud from SFM by querying it with a bounding box
            //  command message format: [Rect tlx tly w h step]
            Bottle cmd, reply;
            cmd.addString("Rect");
            cmd.addInt(bb_top_left(0));
            cmd.addInt(bb_top_left(1));
            cmd.addInt(width);
            cmd.addInt(height);
            cmd.addInt(1);

            outCommandSFM.write(cmd, reply);

            //  point cloud has the form of a n x 3 matrix
            objectPointCloud.resize(width*height, 3);
            objectPointCloud.zero();

            //  reply contains a list of X Y Z triplets
            //  WARNING: LOGS INVALID DISPARITY POINTS AS WELL
            for (int idx = 0; idx < reply.size(); idx+=3)
            {
                objectPointCloud(idx/3, 0) = reply.get(idx+0).asDouble();   // x value
                objectPointCloud(idx/3, 1) = reply.get(idx+1).asDouble();   // y value
                objectPointCloud(idx/3, 2) = reply.get(idx+2).asDouble();   // z value
            }
            return true;
        }
        else
        {
            yError() << "Could not retrieve bounding box for object " << objectToFind;
            return false;
        }

    }

    bool retrieveObjectPointCloudFromName(PointCloud<DataXYZRGBA> &objectPointCloud, const string &object)
    {
        //  get object bounding box given object name

        Vector bb_top_left, bb_bot_right;

        if (retrieveObjectBoundingBox(object, bb_top_left, bb_bot_right))
        {
            int width   = abs(bb_bot_right(0) - bb_top_left(0)) + 1;
            int height  = abs(bb_bot_right(1) - bb_top_left(1)) + 1;

            Vector center_bb(2);
            center_bb(0) = bb_top_left(0) + width/2;
            center_bb(1) = bb_top_left(1) + height/2;

            return this->retrieveObjectPointCloudFromImagePosition(objectPointCloud, center_bb);
        }
        else
        {
            yError() << "retrieveObjectPointCloudFromName: Could not retrieve bounding box for object " << object;
            return false;
        }
    }

    bool retrieveObjectPointCloudFrom3DPosition(PointCloud<DataXYZRGBA> &objectPointCloud, const Vector &position3D)
    {
        if(position3D.size() == 3)
        {
            //  get projection of point in image from SFM
            //  command message format: [cart2stereo X Y Z]
            Bottle cmd, reply;
            cmd.addString("cart2stereo");
            cmd.addDouble(position3D(0));
            cmd.addDouble(position3D(1));
            cmd.addDouble(position3D(2));

            outCommandSFM.write(cmd, reply);

            if (reply.size() >=2)
            {
                Vector imagePosition(2);
                imagePosition(0) = reply.get(0).asDouble();
                imagePosition(1) = reply.get(1).asDouble();

                return this->retrieveObjectPointCloudFromImagePosition(objectPointCloud, imagePosition);
            }
            else
            {
                yError() << "retrieveObjectPointCloudFrom3DPosition: Could not retrieve projection of 3D point in image plane";
                return false;
            }
        }
        else
        {
            yError() << "retrieveObjectPointCloudFrom3DPosition: Invalid dimension of object image position input vector";
            return false;
        }
    }

    bool retrieveObjectPointCloudFromImagePosition(PointCloud<DataXYZRGBA> &objectPointCloud, const Vector &objectImagePosition)
    {
        if(objectImagePosition.size() == 2)
        {
            //  get list of points that belong to the object from lbpextract
            //  command message format: [get_component_around x y]
            Bottle cmdSeg, replySeg;
            cmdSeg.addString("get_component_around");
            cmdSeg.addInt(objectImagePosition(0));
            cmdSeg.addInt(objectImagePosition(1));

            if (!outCommandSegm.write(cmdSeg,replySeg)){
                yError() << "retrieveObjectPointCloudFromImagePosition: Could not write to segmentation RPC port";
                return false;
            }

            yDebug() << "Segmented point list obtained from segmentation module";

            //  lbpExtract replies with a list of points
            if (replySeg.size() < 1)
                {
                yError() << "retrieveObjectPointCloudFromImagePosition: Empty point list retrieved from segmentation module!" ;
                return false;
                }

            Bottle *pointList = replySeg.get(0).asList();

            if (pointList->size() > 0){

                //  each point is a list of 2 coordinates
                //  build one list of points to query SFM for 3d coordinates
                Bottle cmdSFM, replySFM;
                cmdSFM.addString("Points");

                yDebug() << "Retrieved " << pointList->size() << " 2D points.";

                for (int point_idx=0; point_idx < pointList->size(); point_idx++)
                {
                Bottle *point2D = pointList->get(point_idx).asList();
                cmdSFM.addInt(point2D->get(0).asInt());
                cmdSFM.addInt(point2D->get(1).asInt());
                }

                //yDebug() << "Query: " << cmdSFM.toString();

                if (!outCommandSFM.write(cmdSFM, replySFM))
                {
                    yError() << "retrieveObjectPointCloudFromImagePosition: Could not write to SFM RPC port";
                    return false;
                }

                yDebug() << "3D point list obtained from SFM";

                //  acquire image from camera input
                ImageOf<PixelRgb> *inCamImg = inImgPort.read();

                yDebug() << "Image obtained from camera stream";

                //  empty point cloud
                objectPointCloud.clear();

                yDebug() << "Reply from SFM obtained. Cycling through " << pointList->size() << " 3D points...";

                for (int point_idx = 0; point_idx < replySFM.size()/3; point_idx++)
                {
                    double x = replySFM.get(point_idx*3).asDouble();   // x value
                    double y = replySFM.get(point_idx*3+1).asDouble();   // y value
                    double z = replySFM.get(point_idx*3+2).asDouble();   // z value

                    //yDebug() << x << y << z;

                    //  0 0 0 points are invalid and must be discarded
                    if (x==0.0 && y==0.0 && z==0.0)
                        continue;

                    //  fetch rgb from image according to 2D coordinates
                    Bottle *point2D = pointList->get(point_idx).asList();
                    PixelRgb point_rgb = inCamImg->pixel(point2D->get(0).asInt(), point2D->get(1).asInt());

                    //  bake the yarp 3D point struct
                    DataXYZRGBA point3D;
                    point3D.x = x;
                    point3D.y = y;
                    point3D.z = z;
                    point3D.r = point_rgb.r;
                    point3D.g = point_rgb.g;
                    point3D.b = point_rgb.b;
                    point3D.a = UCHAR_MAX;

                    objectPointCloud.push_back(point3D);
                }

                yInfo() << "Point cloud retrieved: " << objectPointCloud.size() << " points stored.";

                return true;
            }
            else
            {
                yError() << "retrieveObjectPointCloudFromImagePosition: Empty point cloud retrieved around image point " << objectImagePosition(0) << " " << objectImagePosition(1);
                return false;
            }

        }
        else
        {
            yError() << "retrieveObjectPointCloudFromImagePosition: Invalid dimension of object image position input vector";
            return false;
        }
    }

    bool streamSinglePointCloud(const string &object)
    {
        //  retrieve object point cloud
        PointCloud<DataXYZRGBA> &pointCloud = outPort.prepare();

        if (retrieveObjectPointCloudFromName(pointCloud, object))
        {
            //  prepare the command to sent to superquadric-model
            Bottle &cmdSQM = outBottlePointCloud.prepare();
            Bottle &pc = cmdSQM.addList();

            //  send point cloud on output port
            for (int idx_point = 0; idx_point < pointCloud.width() * pointCloud.height(); idx_point++)
            {
                Bottle &p = pc.addList();
                p.addDouble(pointCloud(idx_point).x);
                p.addDouble(pointCloud(idx_point).y);
                p.addDouble(pointCloud(idx_point).z);
            }

            //  send rpc command, get response
            outBottlePointCloud.write();

            //  farewell, point cloud
            outPort.write();
            return true;
        }
        else
        {
            yError() << "Could not retrieve point cloud for object " << object;
            return false;
        }
    }

    int dumpToOFFFile(const string &filename, deque<Point3DRGB> &pointCloud)
    {
        fstream dumpFile;
        string filename_n_ext;

        //  if file already exists, create one with different name
        for(int file_n=0; file_n<1000; file_n++)
        {
            string file_num = std::to_string(file_n);
            file_num.insert(file_num.begin(), 3-file_num.length(), '0');
            filename_n_ext = filename + "_" + file_num + ".off";
            //  check if file can be accessed
            fstream f(filename_n_ext.c_str());
            if (!f.good())
                break;
        }

        dumpFile.open(filename_n_ext, ios::out);

        if (dumpFile.is_open()){
            int n_points = pointCloud.size();

            dumpFile << "COFF"               << "\n";
            dumpFile << n_points << " 0 0"  << "\n";

            for (int idx_point = 0; idx_point < n_points; idx_point++)
            {
                //  add xyz rgb
                //dumpFile << pointCloud.at(idx_point).toYarpVectorRGB().toString() << "\n";
                dumpFile << pointCloud.at(idx_point).x << " ";
                dumpFile << pointCloud.at(idx_point).y << " ";
                dumpFile << pointCloud.at(idx_point).z << " ";
                dumpFile << (int)pointCloud.at(idx_point).r << " ";
                dumpFile << (int)pointCloud.at(idx_point).g << " ";
                dumpFile << (int)pointCloud.at(idx_point).b << "\n";
            }

            dumpFile.close();

            return 0;
        }

        return -1;

    }

    int dumpToOFFFile(const string &filename, PointCloud<DataXYZRGBA> &pointCloud)
    {
        fstream dumpFile;
        string filename_n_ext;

        //  if file already exists, create one with different name
        for(int file_n=0; file_n<1000; file_n++)
        {
            string file_num = std::to_string(file_n);
            file_num.insert(file_num.begin(), 3-file_num.length(), '0');
            filename_n_ext = filename + "_" + file_num + ".off";
            //  check if file can be accessed
            fstream f(filename_n_ext.c_str());
            if (!f.good())
                break;
        }

        dumpFile.open(filename_n_ext, ios::out);

        if (dumpFile.is_open()){
            int n_points = pointCloud.width() * pointCloud.height();

            dumpFile << "COFF"               << "\n";
            dumpFile << n_points << " 0 0"  << "\n";

            for (int idx_point = 0; idx_point < n_points; idx_point++)
            {
                //  add xyz rgb
                //dumpFile << pointCloud.at(idx_point).toYarpVectorRGB().toString() << "\n";
                dumpFile << pointCloud(idx_point).x << " ";
                dumpFile << pointCloud(idx_point).y << " ";
                dumpFile << pointCloud(idx_point).z << " ";
                dumpFile << (int)pointCloud(idx_point).r << " ";
                dumpFile << (int)pointCloud(idx_point).g << " ";
                dumpFile << (int)pointCloud(idx_point).b << "\n";
            }

            dumpFile.close();

            return 0;
        }

        return -1;

    }

#ifdef POINTCLOUDREAD_USES_PCL
    int dumpToPCDFile(const string &filename, const PointCloud<DataXYZRGBA> &pointCloud)
    {

        string filename_n_ext;

        //  if file already exists, create one with different name
        for(int file_n=0; file_n<1000; file_n++)
        {
            string file_num = std::to_string(file_n);
            file_num.insert(file_num.begin(), 3-file_num.length(), '0');
            filename_n_ext = filename + "_" + file_num + ".pcd";
            //  check if file can be accessed
            fstream f(filename_n_ext.c_str());
            if (!f.good())
                break;
        }

        return yarp::pcl::savePCD< yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(filename_n_ext, pointCloud);

    }
#endif

    bool dumpPointCloud(const string &format, const string &object){

        PointCloud<DataXYZRGBA> yarpCloud;

        if (retrieveObjectPointCloudFromName(yarpCloud, object))
        {
            //  dump point cloud to file
            string dumpFileName = object + "_" + baseDumpFileName;

            string string_format_lowercase = format;
            transform(string_format_lowercase.begin(), string_format_lowercase.end(), string_format_lowercase.begin(), ::tolower);
            if (string_format_lowercase.compare("off") == 0)
            {
                if (dumpToOFFFile(dumpFileName, yarpCloud) == 0)
                {
                    yInfo() << "Dumped point cloud in OFF format.";
                    return true;

                }
                else
                    yError() << "Dump failed!";
            }
#ifdef POINTCLOUDREAD_USES_PCL
            else if (string_format_lowercase.compare("pcd") == 0)
            {
                if (dumpToPCDFile(dumpFileName, yarpCloud) == 0)
                {
                    yInfo() << "Dumped point cloud in PCD format.";
                    return true;

                }
                else
                    yError() << "Dump failed!";
            }
#endif
            else
                yError() << "Invalid dump format.";
        }
        else
            yError() << "Could not retrieve object point cloud";

        return false;

    }

    Bottle get_point_cloud(const string &object) override
    {
        LockGuard lg(mutex);

        //  log previous operation mode
        OpMode backupOperationMode = operationMode;
        operationMode = OpMode::OP_MODE_STREAM_ONE;

        PointCloud<DataXYZRGBA> retrievedPointCloud;
        retrievedPointCloud.clear();
        retrieveObjectPointCloudFromName(retrievedPointCloud, object);

        operationMode = backupOperationMode;

        yDebug() << "Retrieved " << retrievedPointCloud.size() << "points.";

        Bottle reply = retrievedPointCloud.toBottle();

        return reply;

    }

    Bottle get_point_cloud_from_3D_position(double x, double y, double z) override
    {
        LockGuard lg(mutex);

        //  log previous operation mode
        OpMode backupOperationMode = operationMode;
        operationMode = OpMode::OP_MODE_STREAM_ONE;

        PointCloud<DataXYZRGBA> retrievedPointCloud;
        retrievedPointCloud.clear();
        Vector position(3);
        position(0) = x;
        position(1) = y;
        position(2) = z;
        retrieveObjectPointCloudFrom3DPosition(retrievedPointCloud, position);

        operationMode = backupOperationMode;

        yDebug() << "Retrieved " << retrievedPointCloud.size() << "points.";

        Bottle reply = retrievedPointCloud.toBottle();

        return reply;

    }

    Bottle get_point_cloud_from_image_position(double u, double v) override
    {
        LockGuard lg(mutex);

        //  log previous operation mode
        OpMode backupOperationMode = operationMode;
        operationMode = OpMode::OP_MODE_STREAM_ONE;

        PointCloud<DataXYZRGBA> retrievedPointCloud;
        retrievedPointCloud.clear();
        Vector position(2);
        position(0) = u;
        position(1) = v;
        retrieveObjectPointCloudFromImagePosition(retrievedPointCloud, position);

        operationMode = backupOperationMode;

        yDebug() << "Retrieved " << retrievedPointCloud.size() << "points.";

        Bottle reply = retrievedPointCloud.toBottle();

        return reply;

    }

    bool stream_one(const string &object) override
    {
        mutex.lock();

        //  back up operation mode
        OpMode backupOperationMode = operationMode;
        operationMode = OpMode::OP_MODE_STREAM_ONE;

        //  one-shot acquisition and streaming of point cloud
        bool reply = streamSinglePointCloud(object);

        operationMode = backupOperationMode;

        mutex.unlock();

        return reply;

    }

    bool stream_start(const string &object) override
    {
        mutex.lock();

        //  log object that is being looked for
        objectToFind = object;
        operationMode = OpMode::OP_MODE_STREAM_MANY;

        mutex.unlock();

        return true;
    }

    bool stream_stop() override
    {
        mutex.lock();

        objectToFind.clear();
        operationMode = OpMode::OP_MODE_NONE;

        mutex.unlock();

        return true;

    }

    bool dump_one(const string &object, const string &format) override
    {
        mutex.lock();

        OpMode backupOperationMode = operationMode;
        operationMode = OpMode::OP_MODE_DUMP_ONE;

        bool reply = dumpPointCloud(format, object);

        operationMode = backupOperationMode;

        mutex.unlock();

        return reply;

    }

    bool set_period(const double modulePeriod) override
    {
        moduleUpdatePeriod = modulePeriod;

        yInfo() << "Update period changed to" << moduleUpdatePeriod << "[s]";

        return true;

    }

public:

    bool configure(ResourceFinder &rf) override
    {
        moduleName = "pointCloudRead";
        baseDumpFileName = "point_cloud";

        //  default update period is 1 second
        moduleUpdatePeriod = 1.0;

        bool okOpen = true;

        okOpen &= inCommandPort.open("/" + moduleName + "/rpc");
        okOpen &= inImgPort.open("/" + moduleName + "/imgL:i");
        okOpen &= outCommandOPC.open("/" + moduleName + "/OPCrpc");
        okOpen &= outCommandSFM.open("/" + moduleName + "/SFMrpc");
        okOpen &= outCommandSegm.open("/" + moduleName + "/segmrpc");
        okOpen &= outPort.open("/" + moduleName + "/pointCloud:o");

        okOpen &= outBottlePointCloud.open("/" + moduleName + "/bottledPointCloud:o");

        if (!okOpen)
        {
            yError() << "Unable to open ports!";
            return false;
        }

        attach(inCommandPort);

        operationMode = OpMode::OP_MODE_NONE;

        return true;

    }

    bool interruptModule() override
    {
        inCommandPort.interrupt();
        inImgPort.interrupt();
        outCommandOPC.interrupt();
        outCommandSFM.interrupt();
        outCommandSegm.interrupt();

        outBottlePointCloud.interrupt();

        return true;

    }

    bool close() override
    {
        inCommandPort.close();
        inImgPort.close();
        outCommandOPC.close();
        outCommandSFM.close();
        outCommandSegm.close();
        outPort.close();

        outBottlePointCloud.close();

        return true;

    }

    double getPeriod() override
    {
        return moduleUpdatePeriod;
    }

    bool updateModule() override
    {
        mutex.lock();

        if (operationMode == OpMode::OP_MODE_STREAM_MANY)
        {
            streamSinglePointCloud(objectToFind);
        }
        else if (operationMode == OpMode::OP_MODE_NONE)
        {
            //  birds chirping
        }

        mutex.unlock();

        return true;

    }

};

int main(int argc, char *argv[]) {

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    PointCloudReadModule pcmod;
    return pcmod.runModule(rf);

}
