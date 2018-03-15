// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_PCR
#define YARP_THRIFT_GENERATOR_PCR

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class PCR_IDL;

class PCR_IDL : public yarp::os::Wire {
public:
  PCR_IDL();
  virtual bool stream_one(const std::string& objectToFind);
  virtual bool stream_start(const std::string& objectToFind);
  virtual bool stream_stop();
  virtual bool dump_one(const std::string& objectToFind);
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
