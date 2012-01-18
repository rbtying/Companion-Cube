#ifndef _ROS_rover_Enabled_h
#define _ROS_rover_Enabled_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover
{

  class Enabled : public ros::Msg
  {
    public:
      bool motorsEnabled;
      bool settingsDumpEnabled;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motorsEnabled;
      u_motorsEnabled.real = this->motorsEnabled;
      *(outbuffer + offset + 0) = (u_motorsEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motorsEnabled);
      union {
        bool real;
        uint8_t base;
      } u_settingsDumpEnabled;
      u_settingsDumpEnabled.real = this->settingsDumpEnabled;
      *(outbuffer + offset + 0) = (u_settingsDumpEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->settingsDumpEnabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motorsEnabled;
      u_motorsEnabled.base = 0;
      u_motorsEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motorsEnabled = u_motorsEnabled.real;
      offset += sizeof(this->motorsEnabled);
      union {
        bool real;
        uint8_t base;
      } u_settingsDumpEnabled;
      u_settingsDumpEnabled.base = 0;
      u_settingsDumpEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->settingsDumpEnabled = u_settingsDumpEnabled.real;
      offset += sizeof(this->settingsDumpEnabled);
     return offset;
    }

    const char * getType(){ return "rover/Enabled"; };
    const char * getMD5(){ return "944b9b58b23f7416adecf8816d3ed0ef"; };

  };

}
#endif