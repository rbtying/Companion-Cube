#ifndef _ROS_actionlib_TwoIntsResult_h
#define _ROS_actionlib_TwoIntsResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TwoIntsResult : public ros::Msg
  {
    public:
      int64_t sum;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint32_t base;
      } u_sum;
      u_sum.real = this->sum;
      *(outbuffer + offset + 0) = (u_sum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint32_t base;
      } u_sum;
      u_sum.base = 0;
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sum = u_sum.real;
      offset += sizeof(this->sum);
     return offset;
    }

    const char * getType(){ return "actionlib/TwoIntsResult"; };
    const char * getMD5(){ return "b88405221c77b1878a3cbbfff53428d7"; };

  };

}
#endif