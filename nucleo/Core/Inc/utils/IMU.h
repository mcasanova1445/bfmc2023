#ifndef _ROS_utils_IMU_h
#define _ROS_utils_IMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace utils
{

  class IMU : public ros::Msg
  {
    public:
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _accelx_type;
      _accelx_type accelx;
      typedef float _accely_type;
      _accely_type accely;
      typedef float _accelz_type;
      _accelz_type accelz;
      typedef float _posx_type;
      _posx_type posx;
      typedef float _posy_type;
      _posy_type posy;
      typedef double _timestamp_type;
      _timestamp_type timestamp;

    IMU():
      roll(0),
      pitch(0),
      yaw(0),
      accelx(0),
      accely(0),
      accelz(0),
      posx(0),
      posy(0),
      timestamp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      } u_accelx;
      u_accelx.real = this->accelx;
      *(outbuffer + offset + 0) = (u_accelx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accelx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accelx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelx);
      union {
        float real;
        uint32_t base;
      } u_accely;
      u_accely.real = this->accely;
      *(outbuffer + offset + 0) = (u_accely.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accely.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accely.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accely.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accely);
      union {
        float real;
        uint32_t base;
      } u_accelz;
      u_accelz.real = this->accelz;
      *(outbuffer + offset + 0) = (u_accelz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accelz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accelz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelz);
      union {
        float real;
        uint32_t base;
      } u_posx;
      u_posx.real = this->posx;
      *(outbuffer + offset + 0) = (u_posx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posx);
      union {
        float real;
        uint32_t base;
      } u_posy;
      u_posy.real = this->posy;
      *(outbuffer + offset + 0) = (u_posy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posy);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      } u_accelx;
      u_accelx.base = 0;
      u_accelx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accelx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accelx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accelx = u_accelx.real;
      offset += sizeof(this->accelx);
      union {
        float real;
        uint32_t base;
      } u_accely;
      u_accely.base = 0;
      u_accely.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accely.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accely.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accely.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accely = u_accely.real;
      offset += sizeof(this->accely);
      union {
        float real;
        uint32_t base;
      } u_accelz;
      u_accelz.base = 0;
      u_accelz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accelz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accelz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accelz = u_accelz.real;
      offset += sizeof(this->accelz);
      union {
        float real;
        uint32_t base;
      } u_posx;
      u_posx.base = 0;
      u_posx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_posx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_posx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_posx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->posx = u_posx.real;
      offset += sizeof(this->posx);
      union {
        float real;
        uint32_t base;
      } u_posy;
      u_posy.base = 0;
      u_posy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_posy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_posy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_posy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->posy = u_posy.real;
      offset += sizeof(this->posy);
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
     return offset;
    }

    virtual const char * getType() override { return "utils/IMU"; };
    virtual const char * getMD5() override { return "99077fbdb8e401c664e302a16d80e2cc"; };

  };

}
#endif
