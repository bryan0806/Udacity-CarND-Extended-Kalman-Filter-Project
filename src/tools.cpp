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
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE

    //check division by zero
    if(px==0 & py==0){
        cout << "px and py can not be zero !!"<<endl;
        exit;
    }
    //compute the Jacobian matrix
    Hj << px/sqrt(px*px+py*py),py/sqrt(px*px+py*py),0,0,
          -py/(px*px+py*py),px/(px*px+py*py),0,0,
          py*(vx*py-vy*px)/pow((px*px+py*py),3/2),px*(vy*px-vx*py)/pow((px*px+py*py),3/2),px/sqrt(px*px+py*py),py/sqrt(px*px+py*py);

    return Hj;
}
