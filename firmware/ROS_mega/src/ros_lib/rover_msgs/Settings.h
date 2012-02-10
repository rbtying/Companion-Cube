#ifndef _ROS_rover_msgs_Settings_h
#define _ROS_rover_msgs_Settings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rover_msgs
{

  class Settings : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float left_proportional;
      float left_integral;
      float left_derivative;
      float left_conversion_factor;
      float right_proportional;
      float right_integral;
      float right_derivative;
      float right_conversion_factor;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_left_proportional;
      u_left_proportional.real = this->left_proportional;
      *(outbuffer + offset + 0) = (u_left_proportional.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_proportional.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_proportional.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_proportional.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_proportional);
      union {
        float real;
        uint32_t base;
      } u_left_integral;
      u_left_integral.real = this->left_integral;
      *(outbuffer + offset + 0) = (u_left_integral.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_integral.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_integral.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_integral.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_integral);
      union {
        float real;
        uint32_t base;
      } u_left_derivative;
      u_left_derivative.real = this->left_derivative;
      *(outbuffer + offset + 0) = (u_left_derivative.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_derivative.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_derivative.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_derivative.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_derivative);
      union {
        float real;
        uint32_t base;
      } u_left_conversion_factor;
      u_left_conversion_factor.real = this->left_conversion_factor;
      *(outbuffer + offset + 0) = (u_left_conversion_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_conversion_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_conversion_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_conversion_factor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_conversion_factor);
      union {
        float real;
        uint32_t base;
      } u_right_proportional;
      u_right_proportional.real = this->right_proportional;
      *(outbuffer + offset + 0) = (u_right_proportional.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_proportional.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_proportional.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_proportional.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_proportional);
      union {
        float real;
        uint32_t base;
      } u_right_integral;
      u_right_integral.real = this->right_integral;
      *(outbuffer + offset + 0) = (u_right_integral.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_integral.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_integral.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_integral.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_integral);
      union {
        float real;
        uint32_t base;
      } u_right_derivative;
      u_right_derivative.real = this->right_derivative;
      *(outbuffer + offset + 0) = (u_right_derivative.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_derivative.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_derivative.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_derivative.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_derivative);
      union {
        float real;
        uint32_t base;
      } u_right_conversion_factor;
      u_right_conversion_factor.real = this->right_conversion_factor;
      *(outbuffer + offset + 0) = (u_right_conversion_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_conversion_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_conversion_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_conversion_factor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_conversion_factor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_left_proportional;
      u_left_proportional.base = 0;
      u_left_proportional.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_proportional.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_proportional.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_proportional.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_proportional = u_left_proportional.real;
      offset += sizeof(this->left_proportional);
      union {
        float real;
        uint32_t base;
      } u_left_integral;
      u_left_integral.base = 0;
      u_left_integral.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_integral.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_integral.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_integral.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_integral = u_left_integral.real;
      offset += sizeof(this->left_integral);
      union {
        float real;
        uint32_t base;
      } u_left_derivative;
      u_left_derivative.base = 0;
      u_left_derivative.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_derivative.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_derivative.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_derivative.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_derivative = u_left_derivative.real;
      offset += sizeof(this->left_derivative);
      union {
        float real;
        uint32_t base;
      } u_left_conversion_factor;
      u_left_conversion_factor.base = 0;
      u_left_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_conversion_factor = u_left_conversion_factor.real;
      offset += sizeof(this->left_conversion_factor);
      union {
        float real;
        uint32_t base;
      } u_right_proportional;
      u_right_proportional.base = 0;
      u_right_proportional.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_proportional.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_proportional.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_proportional.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_proportional = u_right_proportional.real;
      offset += sizeof(this->right_proportional);
      union {
        float real;
        uint32_t base;
      } u_right_integral;
      u_right_integral.base = 0;
      u_right_integral.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_integral.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_integral.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_integral.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_integral = u_right_integral.real;
      offset += sizeof(this->right_integral);
      union {
        float real;
        uint32_t base;
      } u_right_derivative;
      u_right_derivative.base = 0;
      u_right_derivative.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_derivative.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_derivative.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_derivative.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_derivative = u_right_derivative.real;
      offset += sizeof(this->right_derivative);
      union {
        float real;
        uint32_t base;
      } u_right_conversion_factor;
      u_right_conversion_factor.base = 0;
      u_right_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_conversion_factor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_conversion_factor = u_right_conversion_factor.real;
      offset += sizeof(this->right_conversion_factor);
     return offset;
    }

    const char * getType(){ return "rover_msgs/Settings"; };
    const char * getMD5(){ return "c0d59de285a4827280115b3f83474657"; };

  };

}
#endif