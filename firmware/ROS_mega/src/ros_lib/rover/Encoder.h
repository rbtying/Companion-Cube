#ifndef _ROS_rover_Encoder_h
#define _ROS_rover_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rover
{

  class Encoder : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float left;
      float right;
      int32_t leftCount;
      int32_t rightCount;
      int32_t leftMotor;
      int32_t rightMotor;
      float left_conversion_factor;
      float right_conversion_factor;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right);
      union {
        int32_t real;
        uint32_t base;
      } u_leftCount;
      u_leftCount.real = this->leftCount;
      *(outbuffer + offset + 0) = (u_leftCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftCount.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftCount);
      union {
        int32_t real;
        uint32_t base;
      } u_rightCount;
      u_rightCount.real = this->rightCount;
      *(outbuffer + offset + 0) = (u_rightCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightCount.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightCount);
      union {
        int32_t real;
        uint32_t base;
      } u_leftMotor;
      u_leftMotor.real = this->leftMotor;
      *(outbuffer + offset + 0) = (u_leftMotor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftMotor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftMotor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftMotor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftMotor);
      union {
        int32_t real;
        uint32_t base;
      } u_rightMotor;
      u_rightMotor.real = this->rightMotor;
      *(outbuffer + offset + 0) = (u_rightMotor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightMotor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightMotor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightMotor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightMotor);
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
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right = u_right.real;
      offset += sizeof(this->right);
      union {
        int32_t real;
        uint32_t base;
      } u_leftCount;
      u_leftCount.base = 0;
      u_leftCount.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftCount.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftCount.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftCount.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftCount = u_leftCount.real;
      offset += sizeof(this->leftCount);
      union {
        int32_t real;
        uint32_t base;
      } u_rightCount;
      u_rightCount.base = 0;
      u_rightCount.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightCount.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightCount.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightCount.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightCount = u_rightCount.real;
      offset += sizeof(this->rightCount);
      union {
        int32_t real;
        uint32_t base;
      } u_leftMotor;
      u_leftMotor.base = 0;
      u_leftMotor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftMotor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftMotor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftMotor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftMotor = u_leftMotor.real;
      offset += sizeof(this->leftMotor);
      union {
        int32_t real;
        uint32_t base;
      } u_rightMotor;
      u_rightMotor.base = 0;
      u_rightMotor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightMotor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightMotor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightMotor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightMotor = u_rightMotor.real;
      offset += sizeof(this->rightMotor);
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

    const char * getType(){ return "rover/Encoder"; };
    const char * getMD5(){ return "63b24db5a3c143c57912a66b55703ac8"; };

  };

}
#endif