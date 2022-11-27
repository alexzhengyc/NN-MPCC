#include "nn.h"

namespace mpcc{
NN::NN()
:Ts_(1.0)
{
  std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

NN::NN(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{ 

  module = torch::jit::load(param_.nn_path); 
  for(int i=0; i<k_; i++){
    input_record.push_back(normalize(0.2, 0, 0, 0, 0));
  } 

}

StateVector NN::getF(const State &x,const Input &u){
  const double phi = x.phi;
  const double vx = x.vx;
  const double vy = x.vy;
  const double r  = x.r;
  const double D = x.D;
  const double delta = x.delta;
  const double vs = x.vs;

  const double dD = u.dD;
  const double dDelta = u.dDelta;
  const double dVs = u.dVs;

  std::vector<double> F = nnOutput(vx, vy, r, D, delta);

  StateVector f;
  f(0) = vx*std::cos(phi) - vy*std::sin(phi);
  f(1) = vy*std::cos(phi) + vx*std::sin(phi);
  f(2) = r;
  f(3) = F[0];
  f(4) = F[1];
  f(5) = F[2];
  f(6) = vs;
  f(7) = dD;
  f(8) = dDelta;
  f(9) = dVs;

  return f;
}

LinModelMatrix NN::getLinModel(const State &x, const Input &u){
    
  // compute linearized and discretized model
  const LinModelMatrix lin_model_c = getModelJacobian(x,u);
  return discretizeModel(lin_model_c);
}

LinModelMatrix NN::getModelJacobian(const State &x, const Input &u)
{
  const double phi = x.phi;
  const double vx = x.vx;
  const double vy = x.vy;
  const double r  = x.r;
  const double D = x.D;
  const double scale_factor = 0.35;  // scale_factor: max steering angle 
  const double delta = x.delta / scale_factor;

  // LinModelMatrix lin_model_c;
  A_MPC A_c = A_MPC::Zero();
  B_MPC B_c = B_MPC::Zero();
  g_MPC g_c = g_MPC::Zero();

  const double d1 = 0.1;
  const double d2 = 0.05;
  // const double d3 = 0.01;

  
  std::vector<double> F0 = nnOutput(vx, vy, r, D, delta);
  std::vector<double> F1 = nnOutput(vx+d1, vy, r, D, delta);
  std::vector<double> F2 = nnOutput(vx, vy+d1, r, D, delta);
  std::vector<double> F3 = nnOutput(vx, vy, r+d1, D, delta);
  std::vector<double> F4 = nnOutput(vx, vy, r, D+d2, delta);
  std::vector<double> F5 = nnOutput(vx, vy, r, D, delta+d2/scale_factor);
  
  // vx
  double dvx_vx = 1/d1*(F1[0] - F0[0]);
  double dvy_vx = 1/d1*(F1[1] - F0[1]);
  double dr_vx = 1/d1*(F1[2] - F0[2]);


  // vy
  double dvx_vy = 1/d1*(F2[0] - F0[0]);
  double dvy_vy = 1/d1*(F2[1] - F0[1]);
  double dr_vy = 1/d1*(F2[2] - F0[2]);

  // r
  double dvx_r = 1/d1*(F3[0] - F0[0]);
  double dvy_r = 1/d1*(F3[1] - F0[1]);
  double dr_r = 1/d1*(F3[2] - F0[2]);

  // D
  double dvx_D = 1/d2*(F4[0] - F0[0]);
  double dvy_D = 1/d2*(F4[1] - F0[1]);
  double dr_D = 1/d2*(F4[2] - F0[2]);

  // delta
  double dvx_delta = 1/d2*(F5[0] - F0[0]);
  double dvy_delta = 1/d2*(F5[1] - F0[1]);
  double dr_delta = 1/d2*(F5[2] - F0[2]);
  
  
  // Jacobians: A Matrix  
  // row1: dX
  A_c(0,2) = -vx*std::sin(phi) - vy*std::cos(phi);
  A_c(0,3) = std::cos(phi);
  A_c(0,4) = -std::sin(phi);
  // row2: dY 
  A_c(1,2) = -vy*std::sin(phi) + vx*std::cos(phi);
  A_c(1,3) = std::sin(phi);
  A_c(1,4) = std::cos(phi);
  // row3: dphi 
  A_c(2,5) = 1.0;
  // row4: dvx
  A_c(3,3) = dvx_vx;
  A_c(3,4) = dvx_vy;
  A_c(3,5) = dvx_r;
  A_c(3,7) = dvx_D;
  A_c(3,8) = dvx_delta;
  // row5: dvy
  A_c(4,3) = dvy_vx;
  A_c(4,4) = dvy_vy;
  A_c(4,5) = dvy_r;
  A_c(4,7) = dvy_D;
  A_c(4,8) = dvy_delta;
  // row6: dr
  A_c(5,3) = dr_vx;
  A_c(5,4) = dr_vy;
  A_c(5,5) = dr_r;
  A_c(5,7) = dr_D;
  A_c(5,8) = dr_delta;
  // row7: ds 
  A_c(6,9) = 1.0;
  // row8: dD
  // all zero 
  // row 9: ddelta 
  // all zero 
  // row 10: dvs
  // all zero

  // Jacobians: B Matrix 
    B_c(7,0) = 1.0;
    B_c(8,1) = 1.0;
    B_c(9,2) = 1.0;

  // Jacobians: C Matrix
  const StateVector f = getF(x,u);
  g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);

  // update past input
  for(int i=0;i<k_-1;i++){
    for(int j=0;j<5;j++){
      input_record[i][j] = input_record[i+1][j];
    }
  } 
  input_record[k_-1][0] = vx;
  input_record[k_-1][1] = vy;
  input_record[k_-1][2] = r;
  input_record[k_-1][3] = D;
  input_record[k_-1][4] = delta;

  return {A_c,B_c,g_c};

}

std::vector<double> NN::nnOutput(double vx, double vy, double r, double D, double delta)
{

  std::vector<double> newinput_v = normalize(vx, vy, r, D, delta);
  
  torch::Tensor input_t = torch::rand({1, k_, 5});
  for(int i=0; i<k_-1; i++){
    for(int j=0;j<5;j++)
      input_t[0][i][j] = float(input_record[i+1][j]);
  }
  for(int j=0; j<5; j++){
    input_t[0][k_-1][j] = float(newinput_v[j]);
  }

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(input_t);

  at::Tensor output_t = module.forward(inputs).toTensor().squeeze(0);

  std::vector<double> output_v(5);
  for(int i=0; i<3; i++){
    output_v[i] = output_t[0][i].item<double>();
  }
  
  return denormalize(output_v[0], output_v[1], output_v[2]);

}

std::vector<double> NN::normalize(double vx, double vy, double r, double D, double delta) const{
  
  std::vector<double> norm;
  vx = (vx - param_.vx_min) / (param_.vx_max - param_.vx_min);
  norm.push_back(vx);
  vy = (vy - param_.vy_min) / (param_.vy_max - param_.vy_min);
  norm.push_back(vy);
  r = (r - param_.r_min) / (param_.r_max - param_.r_min);
  norm.push_back(r);
  D = (D - param_.D_min) / (param_.D_max - param_.D_min);
  norm.push_back(D);
  delta = (delta - param_.delta_min) / (param_.delta_max - param_.delta_min);
  norm.push_back(delta);

  return norm;
}

std::vector<double> NN::denormalize(double dvx, double dvy, double dr) const{
  
  std::vector<double> denorm;
  dvx = dvx * (param_.dvx_max-param_.dvx_min) + param_.dvx_min;
  denorm.push_back(dvx);
  dvy = dvy * (param_.dvy_max-param_.dvy_min) + param_.dvy_min;
  denorm.push_back(dvy);
  dr = dr * (param_.dr_max-param_.dr_min) + param_.dr_min;
  denorm.push_back(dr);
  return denorm;
}

LinModelMatrix NN::discretizeModel(const LinModelMatrix &lin_model_c) const
{
    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX,NX>(0,0) = lin_model_c.A;
    temp.block<NX,NU>(0,NX) = lin_model_c.B;
    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
    temp = temp*Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    //temp_res = [Ad,Bd,gd;zeros]
    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);

    return {A_d,B_d,g_d};
}



}

