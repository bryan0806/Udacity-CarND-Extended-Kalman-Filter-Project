#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    if(estimations.size()==0){
        cout<<"zero estimation vector size!!";
        exit;

    }
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()){
        cout << "different vector size !";
        exit;
    }
            // ... your code here
    cout << " size of estimations vector:" << estimations.size() << endl;
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i]-ground_truth[i];
    //cout << " residual:" << residual << endl;
    //cout << " residual array :" << residual.array() << endl;
    residual = residual.array()*residual.array();
    rmse += residual;

    }

    //calculate the mean
    // ... your code here
    rmse = rmse/estimations.size();
    //calculate the squared root
    // ... your code here
    rmse = rmse.array().sqrt();
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
