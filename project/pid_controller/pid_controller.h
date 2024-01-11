/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
private:
  /* data */ 

public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
    double p_error{0};
    double d_error{0};
    double i_error{0};
    double prev_cte{0};

    /*
    * Coefficients
    */
    double Kp{1};
    double Kd{1};
    double Ki{1};

    /*
    * Output limits
    */
    double output_lim_max{10};
    double output_lim_min{-10};
  
    /*
    * Delta time
    */
    double delta_time{1};

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


