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
   Kp = Kpi;
   Kd = Kdi;
   Ki = Kii;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   p_error = Kp * cte;
   double diff_cte = cte - prev_cte;
   prev_cte = cte;
   d_error = Kd * (diff_cte/delta_time);
   // d_error = Kd * ((cte - d_error/Kd)/delta_time);
   i_error += Ki * cte * delta_time;
}

// that must be the control input u sent to the system (not the error)
double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
      double control =  (p_error + d_error + i_error);
      if (control < output_lim_min)
      {
         control = output_lim_min;
      }
      else if (control > output_lim_max)
      {
         control = output_lim_max;
      }
      // control = max(control, output_lim_min);
      // control = min(control, output_lim_max);
      return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   if (new_delta_time > 0.0)
   {
      delta_time = new_delta_time;
   }
   return delta_time;
}