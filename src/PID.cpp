#include "PID.h"
#include<iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    std::cout<<"Initialized"<<std::endl;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error= 0.0;
     this->d_error= 0.0;
      this->i_error= 0.0;
}

void PID::UpdateError(double cte) {
    
    this->d_error= cte- this->p_error;
    this->p_error = cte;
    this->i_error= this->i_error + cte;
    std::cout<<"Update Error Called"<<cte<<std::endl;
}

double PID::TotalError() {
    std::cout<<"TotalError"<<std::endl;
    return -this->Kp * this->p_error - this->Ki* this->i_error -this->Kd * this->d_error;
}

