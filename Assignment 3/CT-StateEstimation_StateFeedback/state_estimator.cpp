#include "state_estimator.h"

void PredictionUpdate(const Matrix<1> &u, Matrix<1> &xhat) {
  // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT PredictionUpdate OF THE STATE ESTIMATOR

  // // System A&B-matrix
  // float arrayA[1][1]{{ ? }}; //Provide here the element values of state-space matrix A
  // Matrix<1,1> A = arrayA;
  // float arrayB[1][1]{{ ? }}; //Provide here the element values of state-space matrix B
  // Matrix<1,1> B = arrayB;
  
  // // Evaluate discrete-time system dynamics
  // xhat = A*xhat + B*u;
  
}

void CorrectionUpdate(const Matrix<1> &y, Matrix<1> &xhat, Matrix<1> &nu) {
  // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT CorrectionUpdate OF THE STATE ESTIMATOR
 
  // // System C-matrix - measurement equation
  // float arrayC[1][1]{{ ? }}; //Provide here the element values of state-space matrix C
  // Matrix<1,1> C = arrayC;
  
  // // Compute innovation
  // nu = y - C*xhat;
    
  // // Set estimator gain
  // Matrix<1,1> L = -0.618;
  
  // // Compute corrected system state estimate
  // xhat += L * nu;
}
