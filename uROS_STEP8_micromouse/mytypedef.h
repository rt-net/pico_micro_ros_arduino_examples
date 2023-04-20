#ifndef MYTYPEDEF_H_
#define MYTYPEDEF_H_

typedef struct
{
  short value;
  short p_value;
  short error;
  short ref;
  short th_wall;
  short th_control;
  bool is_wall;
  bool is_control;
} t_sensor;

typedef struct
{
  double control;
  double error;
  double p_error;
  double diff;
  double sum;
  double sum_max;
  double kp;
  double kd;
  double ki;
  bool enable;
} t_control;

typedef enum { MOT_FORWARD, MOT_BACK } t_CW_CCW;

#endif  // MYTYPEDEF_H_