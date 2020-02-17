#include "IPC_controller_forRN16.h"

IPC_controller_forRN16::IPC_controller_forRN16(){
}

IPC_controller_forRN16::~IPC_controller_forRN16(){
}

int IPC_controller_forRN16::send_avg_corr(std::vector<float> RN16, double avg_corr){
  data.successFlag = 1;
  for(int i = 0; i<16; i++){
    data.RN16[i] = RN16[i];
  }

  data.avg_corr = avg_corr;

  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();
  return 0;
}

int IPC_controller_forRN16::send_avg_corr(std::vector<float> RN16, std::complex<float> avg_corr){
  data.successFlag = 1;
  for(int i = 0; i<16; i++){
    data.RN16[i] = RN16[i];
  }

  data.avg_corr = norm(avg_corr);
  data.avg_i = avg_corr.real();
  data.avg_q = avg_corr.imag();

  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();

  return 0;
}

int IPC_controller_forRN16::send_avg_corr(std::vector<float> RN16,double avg_corr_norm, std::complex<float> avg_corr){
  data.successFlag = 1;
  for(int i = 0; i<16; i++){
    data.RN16[i] = RN16[i];
  }

  data.avg_corr = avg_corr_norm;
  data.avg_i = avg_corr.real();
  data.avg_q = avg_corr.imag();

  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();

  return 0;
}


int IPC_controller_forRN16::send_avg_corr(std::vector<float> RN16,double avg_corr_norm, std::complex<float> avg_corr, unsigned int round){
  data.successFlag = 1;
  for(int i = 0; i<16; i++){
    data.RN16[i] = RN16[i];
  }

  data.avg_corr = avg_corr_norm;
  data.avg_i = avg_corr.real();
  data.avg_q = avg_corr.imag();
  data.round = round;

  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();

  return 0;
}


int IPC_controller_forRN16::send_failed(int failNumber){
  data.successFlag = failNumber;
  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();

  return 0;
}
