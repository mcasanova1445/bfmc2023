#ifndef _ROS_utils_localisation_h
#define _ROS_utils_localisation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace utils
{

  class localisation : public ros::Msg
  {
    public:
      typedef double _timestamp_type;
      _timestamp_type timestamp;
      typedef float _posA_type;
      _posA_type posA;
      typedef float _posB_type;
      _posB_type posB;
      typedef float _rotA_type;
      _rotA_type rotA;
      typedef float _rotB_type;
      _rotB_type rotB;

    localisation():
      timestamp(0),
      posA(0),
      posB(0),
      rotA(0),
      rotB(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_timestamp;
      u_timestamp.real = this->timestamp;
      *(outbuffer + offset + 0) = (u_timestamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timestamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timestamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timestamp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_timestamp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_timestamp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_timestamp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_timestamp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timestamp);
      union {
        float real;
        uint32_t base;
      } u_posA;
      u_posA.real = this->posA;
      *(outbuffer + offset + 0) = (u_posA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posA);
      union {
        float real;
        uint32_t base;
      } u_posB;
      u_posB.real = this->posB;
      *(outbuffer + offset + 0) = (u_posB.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posB.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posB.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posB.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posB);
      union {
        float real;
        uint32_t base;
      } u_rotA;
      u_rotA.real = this->rotA;
      *(outbuffer + offset + 0) = (u_rotA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotA);
      union {
        float real;
        uint32_t base;
      } u_rotB;
      u_rotB.real = this->rotB;
      *(outbuffer + offset + 0) = (u_rotB.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotB.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotB.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotB.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotB);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_timestamp;
      u_timestamp.base = 0;
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_timestamp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->timestamp = u_timestamp.real;
      offset += sizeof(this->timestamp);
      union {
        float real;
        uint32_t base;
      } u_posA;
      u_posA.base = 0;
      u_posA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_posA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_posA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_posA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->posA = u_posA.real;
      offset += sizeof(this->posA);
      union {
        float real;
        uint32_t base;
      } u_posB;
      u_posB.base = 0;
      u_posB.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_posB.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_posB.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_posB.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->posB = u_posB.real;
      offset += sizeof(this->posB);
      union {
        float real;
        uint32_t base;
      } u_rotA;
      u_rotA.base = 0;
      u_rotA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotA = u_rotA.real;
      offset += sizeof(this->rotA);
      union {
        float real;
        uint32_t base;
      } u_rotB;
      u_rotB.base = 0;
      u_rotB.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotB.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotB.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotB.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotB = u_rotB.real;
      offset += sizeof(this->rotB);
     return offset;
    }

    virtual const char * getType() override { return "utils/localisation"; };
    virtual const char * getMD5() override { return "32d060122e3076e51189db3d2135636c"; };

  };

}
#endif
