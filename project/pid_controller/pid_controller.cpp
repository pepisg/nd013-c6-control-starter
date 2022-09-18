/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   kp_=Kpi;
   ki_=Kii;
   kd_=Kdi;
   max_output_=output_lim_maxi;
   min_output_=output_lim_mini;
   dt_ = 0.0000001; // to avoid zero divisions
}


void PID::UpdateError(double cte) {
   proportional_error_ = cte;
   integral_error_ = integral_error_ + cte*dt_;
   derivative_error_ = (cte - previous_error)/dt_;
   previous_error = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = kp_*proportional_error_ + ki_*integral_error_ + kd_*derivative_error_;
    control = std::min(std::max(control, min_output_), max_output_);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt_ = new_delta_time;
}