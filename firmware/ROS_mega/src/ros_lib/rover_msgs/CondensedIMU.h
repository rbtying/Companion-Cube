#ifndef _ROS_rover_msgs_CondensedIMU_h
#define _ROS_rover_msgs_CondensedIMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rover_msgs
{

  class CondensedIMU : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float roll;
      float pitch;
      float yaw;
      float gyro_x;
      float gyro_y;
      float gyro_z;
      float accel_x;
      float accel_y;
      float accel_z;
      float mag_heading;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.real = this->gyro_x;
      *(outbuffer + offset + 0) = (u_gyro_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.real = this->gyro_y;
      *(outbuffer + offset + 0) = (u_gyro_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.real = this->gyro_z;
      *(outbuffer + offset + 0) = (u_gyro_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_z);
      union {
        float real;
        uint32_t base;
      } u_accel_x;
      u_accel_x.real = this->accel_x;
      *(outbuffer + offset + 0) = (u_accel_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_x);
      union {
        float real;
        uint32_t base;
      } u_accel_y;
      u_accel_y.real = this->accel_y;
      *(outbuffer + offset + 0) = (u_accel_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_y);
      union {
        float real;
        uint32_t base;
      } u_accel_z;
      u_accel_z.real = this->accel_z;
      *(outbuffer + offset + 0) = (u_accel_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_z);
      union {
        float real;
        uint32_t base;
      } u_mag_heading;
      u_mag_heading.real = this->mag_heading;
      *(outbuffer + offset + 0) = (u_mag_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_heading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.base = 0;
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_x = u_gyro_x.real;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.base = 0;
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_y = u_gyro_y.real;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.base = 0;
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_z = u_gyro_z.real;
      offset += sizeof(this->gyro_z);
      union {
        float real;
        uint32_t base;
      } u_accel_x;
      u_accel_x.base = 0;
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_x = u_accel_x.real;
      offset += sizeof(this->accel_x);
      union {
        float real;
        uint32_t base;
      } u_accel_y;
      u_accel_y.base = 0;
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_y = u_accel_y.real;
      offset += sizeof(this->accel_y);
      union {
        float real;
        uint32_t base;
      } u_accel_z;
      u_accel_z.base = 0;
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_z = u_accel_z.real;
      offset += sizeof(this->accel_z);
      union {
        float real;
        uint32_t base;
      } u_mag_heading;
      u_mag_heading.base = 0;
      u_mag_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_heading = u_mag_heading.real;
      offset += sizeof(this->mag_heading);
     return offset;
    }

    const char * getType(){ return "rover_msgs/CondensedIMU"; };
    const char * getMD5(){ return "38b4f3e2e1a67f0124fca6e99619cb65"; };

  };

}
#endif