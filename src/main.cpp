/*
 *  Module to retrieve point clouds from SFM module and dump them to PCL
 *  compatible files. Interfaces with OPC to retrieve the position
 *  of a desired object and retrieves a point cloud from SFM.
 *
 *  Author: Fabrizio Bottarel - <fabrizio.bottarel@iit.it>
 *
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**************************************************************/

class PointCloudReadModule: public yarp::os::RFModule
{
protected:

    RpcServer inCommandPort;
    RpcClient outCommandOPC;
    RpcClient outCommandSFM;

    BufferedPort< PointCloud<XYZ_DATA> > outPort;

    Mutex mutex;

    string moduleName, operationMode, objectToFind;

    bool retrieveObjectBoundingBox(const string objName, Vector &top_left_xy, Vector &bot_right_xy)
    {
        //  get object bounding box from OPC module given object name

        top_left_xy.resize(2);
        bot_right_xy.resize(2);

        //  command message format: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" <val1>) ...)
        Bottle cmd, reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("name");
        content.addString("==");
        content.addString(objName);
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

    bool retrieveObjectPointCloud(PointCloud<XYZ_DATA> &objectPointCloud)
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

            objectPointCloud.clear();
            objectPointCloud.resize(width, height);

            //  reply contains a list of X Y Z triplets
            //  WARNING: LOGS INVALID DISPARITY POINTS AS WELL
            for (int idx = 0; idx < reply.size(); idx+=3)
            {
                objectPointCloud(idx/3).x = reply.get(idx+0).asDouble();
                objectPointCloud(idx/3).y = reply.get(idx+1).asDouble();
                objectPointCloud(idx/3).z = reply.get(idx+2).asDouble();
            }
            return true;
        }
        else
        {
            yError() << "Could not retrieve bounding box for object " << objectToFind;
            return false;
        }

    }

    bool streamSinglePointCloud()
    {
        //  retrieve object point cloud
        PointCloud<XYZ_DATA> &objectPointCloud = outPort.prepare();
        objectPointCloud.clear();

        if (retrieveObjectPointCloud(objectPointCloud))
        {
            //  send point cloud on output port
            for (int point_idx = 0; point_idx < objectPointCloud.size(); point_idx++)
                yDebug() << "Point: " << objectPointCloud(point_idx).x << " " << objectPointCloud(point_idx).y << " " << objectPointCloud(point_idx).z;
            outPort.write();
            return true;
        }
        else
        {
            yError() << "Could not retrieve point cloud for object " << objectToFind;
            return false;
        }
    }

public:

    bool configure(ResourceFinder &rf)
    {
        moduleName = "pointCloudRead";

        /*
         *
         *
         * DEBUG INSTRUCTION: ALWAYS LOOK FOR CARS
         *
         *
         */
        objectToFind = "Car";

        bool okOpen = true;

        okOpen &= inCommandPort.open("/" + moduleName + "/rpc");
        okOpen &= outCommandOPC.open("/" + moduleName + "/OPCrpc");
        okOpen &= outCommandSFM.open("/" + moduleName + "/SFMrpc");
        okOpen &= outPort.open("/" + moduleName + "/pointCloud:o");

        if (!okOpen)
        {
            yError() << "Unable to open ports!";
            return false;
        }

        attach(inCommandPort);

        operationMode = "none";

        return true;

    }

    bool interruptModule()
    {
        inCommandPort.interrupt();
        outCommandOPC.interrupt();
        outCommandSFM.interrupt();

        return true;

    }

    bool close()
    {
        inCommandPort.close();
        outCommandOPC.close();
        outCommandSFM.close();

        return true;

    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        //  change operation mode based on command and available modes
        mutex.lock();

        string modeCmd = command.get(0).asString();

        if (modeCmd == "stream_one")
        {
            operationMode = "stream_one";
            reply.addString("ack");
        }
        else if (modeCmd == "stream_many")
        {
            operationMode = "stream_many";
            reply.addString("ack");
        }
        else if (modeCmd == "dump_one")
        {
            operationMode = "dump_one";
            reply.addString("ack");
        }
        else if (modeCmd == "stop_stream")
        {
            operationMode = "none";
            reply.addString("ack");
        }
        else if (modeCmd == "help")
        {
            reply.addString("Available commands:");
            reply.addString("- stream_one");
            reply.addString("- stream_many");
            reply.addString("- dump_one");
            reply.addString("- stop_stream");
            reply.addString("- quit");
        }
        else if (modeCmd == "quit")
        {
            mutex.unlock();
            return RFModule::respond(command, reply);
        }
        else
            reply.addString("Invalid command. Type help for available commands");

        mutex.unlock();

        return true;

    }

    double getPeriod()
    {
        return 1.0;

    }

    bool updateModule()
    {
        mutex.lock();

        if (operationMode == "stream_one")
        {
            streamSinglePointCloud();
            operationMode = "none";
        }
        else if (operationMode == "stream_many")
        {
            streamSinglePointCloud();
        }
        else if (operationMode == "dump_one")
        {
            yDebug() << "Dumped point cloud in PCL format";
        }
        else if (operationMode == "none")
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
