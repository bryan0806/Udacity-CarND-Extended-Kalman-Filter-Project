#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>  // for trigonometric function

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1,0,0,0,
          0,1,0,0;



}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float ro_dot = measurement_pack.raw_measurements_[2];

      ekf_.x_ << ro*cos(theta),ro*sin(theta),0,0;
      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;
      previous_timestamp_ = measurement_pack.timestamp_;

    }

    // Call KalmanFilter::Init to setup x_in, P_in, F_in,H_in, R_in, Q_in

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;


    //measurement covariance
    //ekf_.R_ = MatrixXd(2, 2);
    //ekf_.R_ << 0.0225, 0,
    //          0, 0.0225;

    //measurement matrix
    //kf_.H_ = MatrixXd(2, 4);
    //kf_.H_ << 1, 0, 0, 0,
    //                  0, 1, 0, 0;

    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    //set the acceleration noise components
    //noise_ax = 5;
    //noise_ay = 5;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the                                                                                                                                                                                                                                                                                                                                                                                                                                                              state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // TODO: YOUR CODE HERE
  //1. Modify the F matrix so that the time is integrated
  //cout << "F is "<< endl << kf_.F_ << endl;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //2. Set the process covariance matrix Q

  //acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  pow(dt,4)/4*noise_ax,0,pow(dt,3)/2*noise_ax,0,
             0,pow(dt,4)/4*noise_ay,0,pow(dt,3)/2*noise_ay,
             pow(dt,3)/2*noise_ax,0,pow(dt,2)*noise_ax,0,
             0,pow(dt,3)/2*noise_ay,0,pow(dt,2)*noise_ay;
  //cout << "Q is now " << kf_.Q_ << endl;
  //3. Call the Kalman Filter predict() function
  //cout << "before predict" << endl;
  ekf_.Predict();
  //cout << "after predict" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */



  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      VectorXd z = VectorXd(2);
      float x_measurement = measurement_pack.raw_measurements_[0];
      float y_measurement = measurement_pack.raw_measurements_[1];
      z << x_measurement,y_measurement;
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(z);

  } else {
    // Laser updates
    VectorXd z = VectorXd(2);
    float x_measurement = measurement_pack.raw_measurements_[0];
    float y_measurement = measurement_pack.raw_measurements_[1];
    z << x_measurement,y_measurement;
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
