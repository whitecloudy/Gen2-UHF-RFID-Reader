#ifndef _IPC_CONTROLLER_forRN16_
#define _IPC_CONTROLLER_forRN16_

#include <iostream>
#include <vector>
#include <complex>
#include "IPC_controller.h"

#define _GATE_SUCCESS 3
#define _SUCCESS 1
#define _GATE_FAIL 2
#define _PREAMBLE_FAIL 0

class IPC_controller_forRN16 : public IPC_controller{

public:
  struct avg_corr_data{
    char successFlag;
    char RN16[16];
    float avg_corr;
    float avg_i;
    float avg_q;
    unsigned int round;
    float cw_i;
    float cw_q;
  } data;

  IPC_controller_forRN16();
  ~IPC_controller_forRN16();

  int send_avg_corr(std::vector<float>, double, std::complex<float>, std::complex<float>, unsigned int);

  int send_failed(int failNumber, unsigned int);
  int send_failed(int failNumber, std::complex<float>avg_ampl, unsigned int);


};



#endif
