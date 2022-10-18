// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "plotting.h"
namespace mpcc{

Plotting::Plotting(double Ts,PathToJson path)
:model_(Model(Ts,path)),
param_(Param(path.param_path))
{
}
void Plotting::plotRun3(std::list<MPCReturn> &log1, std::list<MPCReturn> &log2, std::list<MPCReturn> &log3, const TrackPos &track_xy) const
{

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());

    std::vector<double> plot1_x, plot2_x, plot3_x;
    std::vector<double> plot1_y, plot2_y, plot3_y;
    std::vector<double> plot1_phi, plot2_phi, plot3_phi;
    std::vector<double> plot1_vx, plot2_vx, plot3_vx;
    std::vector<double> plot1_vy, plot2_vy, plot3_vy;
    std::vector<double> plot1_r, plot2_r, plot3_r;
    std::vector<double> plot1_s, plot2_s, plot3_s;
    std::vector<double> plot1_d, plot2_d, plot3_d;
    std::vector<double> plot1_delta, plot2_delta, plot3_delta;
    std::vector<double> plot1_vs, plot2_vs, plot3_vs;

    std::vector<double> plot1_dd, plot2_dd, plot3_dd;
    std::vector<double> plot1_ddelta, plot2_ddelta, plot3_ddelta;
    std::vector<double> plot1_dvs, plot2_dvs, plot3_dvs;

    std::vector<double> plot1_alpha_f, plot2_alpha_f, plot3_alpha_f;
    std::vector<double> plot1_F_rx0, plot2_F_rx0, plot3_F_rx0;
    std::vector<double> plot1_F_ry0, plot2_F_ry0, plot3_F_ry0;
    std::vector<double> plot1_F_rx1, plot2_F_rx1, plot3_F_rx1;
    std::vector<double> plot1_F_ry1, plot2_F_ry1, plot3_F_ry1;

    for(MPCReturn log_i : log1)
    {
        plot1_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot1_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot1_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot1_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot1_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot1_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot1_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot1_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot1_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot1_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot1_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot1_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot1_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

        double alpha_f = model_.getSlipAngleFront(log_i.mpc_horizon[0].xk);
        TireForces F_r0 = model_.getForceRear(log_i.mpc_horizon[0].xk);
        TireForces F_r1 = model_.getForceRear(log_i.mpc_horizon[1].xk);
        plot1_alpha_f.push_back(alpha_f);
        plot1_F_rx0.push_back(F_r0.F_x);
        plot1_F_ry0.push_back(F_r0.F_y);
        plot1_F_rx1.push_back(F_r1.F_x);
        plot1_F_ry1.push_back(F_r1.F_y);
    }
    for(MPCReturn log_i : log2)
    {
        plot2_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot2_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot2_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot2_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot2_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot2_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot2_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot2_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot2_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot2_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot2_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot2_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot2_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

        double alpha_f = model_.getSlipAngleFront(log_i.mpc_horizon[0].xk);
        TireForces F_r0 = model_.getForceRear(log_i.mpc_horizon[0].xk);
        TireForces F_r1 = model_.getForceRear(log_i.mpc_horizon[1].xk);
        plot2_alpha_f.push_back(alpha_f);
        plot2_F_rx0.push_back(F_r0.F_x);
        plot2_F_ry0.push_back(F_r0.F_y);
        plot2_F_rx1.push_back(F_r1.F_x);
        plot2_F_ry1.push_back(F_r1.F_y);
    }
    for(MPCReturn log_i : log3)
    {
        plot3_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot3_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot3_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot3_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot3_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot3_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot3_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot3_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot3_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot3_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot3_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot3_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot3_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

        double alpha_f = model_.getSlipAngleFront(log_i.mpc_horizon[0].xk);
        TireForces F_r0 = model_.getForceRear(log_i.mpc_horizon[0].xk);
        TireForces F_r1 = model_.getForceRear(log_i.mpc_horizon[1].xk);
        plot3_alpha_f.push_back(alpha_f);
        plot3_F_rx0.push_back(F_r0.F_x);
        plot3_F_ry0.push_back(F_r0.F_y);
        plot3_F_rx1.push_back(F_r1.F_x);
        plot3_F_ry1.push_back(F_r1.F_y);
    }


    std::vector<double> plot_eps_x;
    std::vector<double> plot_eps_y;
    for(double t = 0; t<2*M_PI;t+=0.1)
    {
        plot_eps_x.push_back(cos(t)*param_.Dr*param_.e_eps);
        plot_eps_y.push_back(sin(t)*param_.Dr*1./param_.e_long*param_.e_eps);
    }

    plt::figure(1);
    // plt::plot(plot_xc,plot_yc,"r--");
    plt::plot(plot_xi,plot_yi, "k-");
    plt::plot(plot_xo,plot_yo, "k-");

    plt::plot(plot1_x,plot1_y, "b--");
    plt::plot(plot2_x,plot2_y, "r-");
    plt::plot(plot3_x,plot3_y, "g-");

    plt::axis("equal");
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");

    plt::figure(2);
    plt::subplot(3,2,1);
    plt::plot(plot1_x, "b--");
    plt::plot(plot2_x, "r-");
    plt::plot(plot3_x, "g-");
    plt::ylabel("X [m]");

    plt::subplot(3,2,2);
    plt::plot(plot1_y, "b--");
    plt::plot(plot2_y, "r-");
    plt::plot(plot3_y, "g-");
    plt::ylabel("Y [m]");

    plt::subplot(3,2,3);
    plt::plot(plot1_phi, "b--");
    plt::plot(plot2_phi, "r-");
    plt::plot(plot3_phi, "g-");
    plt::ylabel("phi [rad]");

    plt::subplot(3,2,4);
    plt::plot(plot1_vx, "b--");
    plt::plot(plot2_vx, "r-");
    plt::plot(plot3_vx, "g-");
    plt::ylabel("v_x [m/s]");

    plt::subplot(3,2,5);
    plt::plot(plot1_vy, "b--");
    plt::plot(plot2_vy, "r-");
    plt::plot(plot3_vy, "g-");
    plt::ylabel("v_y [m/s]");

    plt::subplot(3,2,6);
    plt::plot(plot1_r, "b--");
    plt::plot(plot2_r, "r-");
    plt::plot(plot3_r, "g-");
    plt::ylabel("r [rad/s]");

    plt::figure(3);
    plt::subplot(3,1,1);
    plt::plot(plot1_d, "b--");
    plt::plot(plot2_d, "r-");
    plt::plot(plot3_d, "g-");
    plt::ylabel("D [-]");

    plt::subplot(3,1,2);
    plt::plot(plot1_delta, "b--");
    plt::plot(plot2_delta, "r-");
    plt::plot(plot3_delta, "g-");
    plt::ylabel("delta [rad]");

    plt::subplot(3,1,3);
    plt::plot(plot1_vs, "b--");
    plt::plot(plot2_vs, "r-");
    plt::plot(plot3_vs, "g-");
    plt::ylabel("v_s [m/s]");

    plt::figure(4);
    plt::subplot(3,1,1);
    plt::plot(plot1_d, "b--");
    plt::plot(plot2_d, "r-");
    plt::plot(plot3_d, "g-");
    plt::ylabel("dot{D} [-]");

    plt::subplot(3,1,2);
    plt::plot(plot1_delta, "b--");
    plt::plot(plot2_delta, "r-");
    plt::plot(plot3_delta, "g-");
    plt::ylabel("dot{delta} [rad/s]");

    plt::subplot(3,1,3);
    plt::plot(plot1_vs, "b--");
    plt::plot(plot2_vs, "r-");
    plt::plot(plot3_vs, "g-");
    plt::ylabel("dot{v_s} [m/s^2]");

    plt::figure(5);
    plt::plot(plot1_s, "b--");
    plt::plot(plot2_s, "r-");
    plt::plot(plot3_s, "g-");
    plt::ylabel("s [m]");

    plt::figure(6);
    plt::plot(plot1_alpha_f, "b--");
    plt::plot(plot2_alpha_f, "r-");
    plt::plot(plot3_alpha_f, "g-");
    plt::ylabel("alpha_f [rad]");

    plt::legend();
    plt::show();

}
void Plotting::plotSim3(std::list<MPCReturn> &log1, std::list<MPCReturn> &log2, std::list<MPCReturn> &log3, const TrackPos &track_xy) const
{   
    std::list<MPCReturn> log1c(log1);
    std::list<MPCReturn> log2c(log2);
    std::list<MPCReturn> log3c(log3);

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());


    std::vector<double> plot1_x, plot2_x, plot3_x;
    std::vector<double> plot1_y, plot2_y, plot3_y;

    int n_sim = log1.size();
    int step = log1.front().mpc_horizon.size();

    plt::figure(7);
    plt::plot(plot_xi,plot_yi,"k-");
    plt::plot(plot_xc,plot_yc,"r--");
    plt::plot(plot_xo,plot_yo,"k-");
    plt::axis("equal");
    plt::xlim(-10,10);
    plt::ylim(-5,5);
    
    plt::pause(10);
        
    for(int i=0; i<n_sim; i++){

        plot1_x.resize(0);
        plot1_y.resize(0);
        plot2_x.resize(0);
        plot2_y.resize(0);
        plot3_x.resize(0);
        plot3_y.resize(0);

        for(int j=0;j<step;j++)
        {
            plot1_x.push_back(log1c.front().mpc_horizon[j].xk.X);
            plot1_y.push_back(log1c.front().mpc_horizon[j].xk.Y);
            plot2_x.push_back(log2c.front().mpc_horizon[j].xk.X);
            plot2_y.push_back(log2c.front().mpc_horizon[j].xk.Y);
            plot3_x.push_back(log3c.front().mpc_horizon[j].xk.X);
            plot3_y.push_back(log3c.front().mpc_horizon[j].xk.Y);
        }

        plt::clf();
        plt::plot(plot_xi,plot_yi,"k-");
        plt::plot(plot_xo,plot_yo,"k-");
        
        plotBox(log1c.front().mpc_horizon[0].xk);
        plotBox(log2c.front().mpc_horizon[0].xk);
        plotBox(log3c.front().mpc_horizon[0].xk);

        plt::plot(plot1_x,plot1_y,"b--");
        plt::plot(plot2_x,plot2_y,"r-");
        plt::plot(plot3_x,plot3_y,"g-");
        
        plt::axis("equal");
        plt::xlim(-10,10);
        plt::ylim(-5,5);
        plt::pause(0.06);

        log1c.pop_front();
        log2c.pop_front();
        log3c.pop_front();
    }
}

void Plotting::plotRun(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
{

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());

    std::vector<double> plot_x;
    std::vector<double> plot_y;
    std::vector<double> plot_phi;
    std::vector<double> plot_vx;
    std::vector<double> plot_vy;
    std::vector<double> plot_r;
    std::vector<double> plot_s;
    std::vector<double> plot_d;
    std::vector<double> plot_delta;
    std::vector<double> plot_vs;

    std::vector<double> plot_dd;
    std::vector<double> plot_ddelta;
    std::vector<double> plot_dvs;

    std::vector<double> plot_alpha_f;
    std::vector<double> plot_F_rx0;
    std::vector<double> plot_F_ry0;
    std::vector<double> plot_F_rx1;
    std::vector<double> plot_F_ry1;

    for(MPCReturn log_i : log)
    {
        plot_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

        double alpha_f = model_.getSlipAngleFront(log_i.mpc_horizon[0].xk);
        TireForces F_r0 = model_.getForceRear(log_i.mpc_horizon[0].xk);
        TireForces F_r1 = model_.getForceRear(log_i.mpc_horizon[1].xk);
        plot_alpha_f.push_back(alpha_f);
        plot_F_rx0.push_back(F_r0.F_x);
        plot_F_ry0.push_back(F_r0.F_y);
        plot_F_rx1.push_back(F_r1.F_x);
        plot_F_ry1.push_back(F_r1.F_y);
    }

    std::vector<double> plot_eps_x;
    std::vector<double> plot_eps_y;
    for(double t = 0; t<2*M_PI;t+=0.1)
    {
        plot_eps_x.push_back(cos(t)*param_.Dr*param_.e_eps);
        plot_eps_y.push_back(sin(t)*param_.Dr*1./param_.e_long*param_.e_eps);
    }

    plt::figure(1);
    plt::plot(plot_xc,plot_yc,"r--");
    plt::plot(plot_xi,plot_yi,"k-");
    plt::plot(plot_xo,plot_yo,"k-");
    plt::plot(plot_x,plot_y,"b-");
    plt::axis("equal");
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::figure(2);
    plt::subplot(3,2,1);
    plt::plot(plot_x);
    plt::ylabel("X [m]");
    plt::subplot(3,2,2);
    plt::plot(plot_y);
    plt::ylabel("Y [m]");
    plt::subplot(3,2,3);
    plt::plot(plot_phi);
    plt::ylabel("phi [rad]");
    plt::subplot(3,2,4);
    plt::plot(plot_vx);
    plt::ylabel("v_x [m/s]");
    plt::subplot(3,2,5);
    plt::plot(plot_vy);
    plt::ylabel("v_y [m/s]");
    plt::subplot(3,2,6);
    plt::plot(plot_r);
    plt::ylabel("r [rad/s]");


    plt::figure(3);
    plt::subplot(3,1,1);
    plt::plot(plot_d);
    plt::ylabel("D [-]");
    plt::subplot(3,1,2);
    plt::plot(plot_delta);
    plt::ylabel("delta [rad]");
    plt::subplot(3,1,3);
    plt::plot(plot_vs);
    plt::ylabel("v_s [m/s]");

    plt::figure(4);
    plt::subplot(3,1,1);
    plt::plot(plot_dd);
    plt::ylabel("dot{D} [-]");
    plt::subplot(3,1,2);
    plt::plot(plot_ddelta);
    plt::ylabel("dot{delta} [rad/s]");
    plt::subplot(3,1,3);
    plt::plot(plot_dvs);
    plt::ylabel("dot{v_s} [m/s^2]");

    plt::figure(5);
    plt::plot(plot_s);
    plt::ylabel("s [m]");

    plt::figure(6);
    plt::subplot(1,2,1);
    plt::plot(plot_alpha_f);
    plt::ylabel("alpha_f [rad]");
    plt::subplot(1,2,2);
    plt::plot(plot_F_ry0,plot_F_rx0);
    plt::plot(plot_F_ry1,plot_F_rx1);
    plt::plot(plot_eps_x,plot_eps_y);
    plt::axis("equal");
    plt::xlabel("F_y [N]");
    plt::ylabel("F_x [N]");
    plt::show();

}
void Plotting::plotSim(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
{
    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());


    std::vector<double> plot_x;
    std::vector<double> plot_y;

    plt::figure(7);

    for(MPCReturn log_i : log)
    {
        plot_x.resize(0);
        plot_y.resize(0);
        for(int j=0;j<log_i.mpc_horizon.size();j++)
        {
            plot_x.push_back(log_i.mpc_horizon[j].xk.X);
            plot_y.push_back(log_i.mpc_horizon[j].xk.Y);
        }
        plt::clf();
        plt::plot(plot_xc,plot_yc,"r--");
        plt::plot(plot_xi,plot_yi,"k-");
        plt::plot(plot_xo,plot_yo,"k-");
        plotBox(log_i.mpc_horizon[0].xk);
        plt::plot(plot_x,plot_y,"b-");
        plt::axis("equal");
        plt::xlim(-10,10);
        plt::ylim(-5,5);
        plt::pause(0.2);
    }

}

void Plotting::plotRun2(std::list<MPCReturn> &log1, std::list<MPCReturn> &log2, const TrackPos &track_xy) const
{

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());

    std::vector<double> plot1_x, plot2_x;
    std::vector<double> plot1_y, plot2_y;
    std::vector<double> plot1_phi, plot2_phi;
    std::vector<double> plot1_vx, plot2_vx;
    std::vector<double> plot1_vy, plot2_vy;
    std::vector<double> plot1_r, plot2_r;
    std::vector<double> plot1_s, plot2_s;
    std::vector<double> plot1_d, plot2_d;
    std::vector<double> plot1_delta, plot2_delta;
    std::vector<double> plot1_vs, plot2_vs;

    std::vector<double> plot1_dd, plot2_dd;
    std::vector<double> plot1_ddelta, plot2_ddelta;
    std::vector<double> plot1_dvs, plot2_dvs;

    for(MPCReturn log_i : log1)
    {
        plot1_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot1_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot1_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot1_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot1_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot1_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot1_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot1_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot1_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot1_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot1_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot1_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot1_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

    }
    for(MPCReturn log_i : log2)
    {
        plot2_x.push_back(log_i.mpc_horizon[0].xk.X);
        plot2_y.push_back(log_i.mpc_horizon[0].xk.Y);
        plot2_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        plot2_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        plot2_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        plot2_r.push_back(log_i.mpc_horizon[0].xk.r);
        plot2_s.push_back(log_i.mpc_horizon[0].xk.s);
        plot2_d.push_back(log_i.mpc_horizon[0].xk.D);
        plot2_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        plot2_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        plot2_dd.push_back(log_i.mpc_horizon[0].uk.dD);
        plot2_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
        plot2_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

    }

    plt::figure(1);
    // plt::plot(plot_xc,plot_yc,"r--");
    plt::plot(plot_xi,plot_yi, "k-");
    plt::plot(plot_xo,plot_yo, "k-");

    plt::plot(plot1_x,plot1_y, "b--");
    plt::plot(plot2_x,plot2_y, "r-");

    plt::axis("equal");
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");

    plt::figure(2);
    plt::subplot(3,2,1);
    plt::plot(plot1_x, "b--");
    plt::plot(plot2_x, "r-");
    plt::ylabel("X [m]");

    plt::subplot(3,2,2);
    plt::plot(plot1_y, "b--");
    plt::plot(plot2_y, "r-");
    plt::ylabel("Y [m]");

    plt::subplot(3,2,3);
    plt::plot(plot1_phi, "b--");
    plt::plot(plot2_phi, "r-");
    plt::ylabel("phi [rad]");

    plt::subplot(3,2,4);
    plt::plot(plot1_vx, "b--");
    plt::plot(plot2_vx, "r-");
    plt::ylabel("v_x [m/s]");

    plt::subplot(3,2,5);
    plt::plot(plot1_vy, "b--");
    plt::plot(plot2_vy, "r-");
    plt::ylabel("v_y [m/s]");

    plt::subplot(3,2,6);
    plt::plot(plot1_r, "b--");
    plt::plot(plot2_r, "r-");
    plt::ylabel("r [rad/s]");

    plt::figure(3);
    plt::subplot(3,1,1);
    plt::plot(plot1_d, "b--");
    plt::plot(plot2_d, "r-");
    plt::ylabel("D [-]");

    plt::subplot(3,1,2);
    plt::plot(plot1_delta, "b--");
    plt::plot(plot2_delta, "r-");
    plt::ylabel("delta [rad]");

    plt::subplot(3,1,3);
    plt::plot(plot1_vs, "b--");
    plt::plot(plot2_vs, "r-");
    plt::ylabel("v_s [m/s]");

    plt::figure(4);
    plt::subplot(3,1,1);
    plt::plot(plot1_d, "b--");
    plt::plot(plot2_d, "r-");
    plt::ylabel("dot{D} [-]");

    plt::subplot(3,1,2);
    plt::plot(plot1_delta, "b--");
    plt::plot(plot2_delta, "r-");
    plt::ylabel("dot{delta} [rad/s]");

    plt::subplot(3,1,3);
    plt::plot(plot1_vs, "b--");
    plt::plot(plot2_vs, "r-");
    plt::ylabel("dot{v_s} [m/s^2]");

    plt::figure(5);
    plt::plot(plot1_s, "b--");
    plt::plot(plot2_s, "r-");
    plt::ylabel("s [m]");

    plt::legend();
    plt::show();

}
void Plotting::plotSim2(std::list<MPCReturn> &log1, std::list<MPCReturn> &log2, const TrackPos &track_xy) const
{   
    std::list<MPCReturn> log1c(log1);
    std::list<MPCReturn> log2c(log2);

    std::vector<double> plot_xc(track_xy.X.data(),track_xy.X.data() + track_xy.X.size());
    std::vector<double> plot_yc(track_xy.Y.data(),track_xy.Y.data() + track_xy.Y.size());

    std::vector<double> plot_xi(track_xy.X_inner.data(),track_xy.X_inner.data() + track_xy.X_inner.size());
    std::vector<double> plot_yi(track_xy.Y_inner.data(),track_xy.Y_inner.data() + track_xy.Y_inner.size());
    std::vector<double> plot_xo(track_xy.X_outer.data(),track_xy.X_outer.data() + track_xy.X_outer.size());
    std::vector<double> plot_yo(track_xy.Y_outer.data(),track_xy.Y_outer.data() + track_xy.Y_outer.size());


    std::vector<double> plot1_x, plot2_x;
    std::vector<double> plot1_y, plot2_y;

    int n_sim = log1.size();
    int step = log1.front().mpc_horizon.size();

    plt::figure(7);
    plt::plot(plot_xi,plot_yi,"k-");
    plt::plot(plot_xc,plot_yc,"r--");
    plt::plot(plot_xo,plot_yo,"k-");
    plt::axis("equal");
    plt::xlim(-10,10);
    plt::ylim(-5,5);
    
    plt::pause(10);
        
    for(int i=0; i<n_sim; i++){

        plot1_x.resize(0);
        plot1_y.resize(0);
        plot2_x.resize(0);
        plot2_y.resize(0);

        for(int j=0;j<step;j++)
        {
            plot1_x.push_back(log1c.front().mpc_horizon[j].xk.X);
            plot1_y.push_back(log1c.front().mpc_horizon[j].xk.Y);
            plot2_x.push_back(log2c.front().mpc_horizon[j].xk.X);
            plot2_y.push_back(log2c.front().mpc_horizon[j].xk.Y);

        }

        plt::clf();
        plt::plot(plot_xi,plot_yi,"k-");
        plt::plot(plot_xo,plot_yo,"k-");
        
        plotBox(log1c.front().mpc_horizon[0].xk);
        plotBox(log2c.front().mpc_horizon[0].xk);

        plt::plot(plot1_x,plot1_y,"b--");
        plt::plot(plot2_x,plot2_y,"r-");
        
        plt::axis("equal");
        plt::xlim(-10,10);
        plt::ylim(-5,5);
        plt::pause(0.06);

        log1c.pop_front();
        log2c.pop_front();
        
    }
}


void Plotting::plotBox(const State &x0) const
{
    std::vector<double> corner_x;
    std::vector<double> corner_y;
    double body_xl = std::cos(x0.phi)*param_.car_l;
    double body_xw = std::sin(x0.phi)*param_.car_w;
    double body_yl = std::sin(x0.phi)*param_.car_l;
    double body_yw = -std::cos(x0.phi)*param_.car_w;

    corner_x.push_back(x0.X + body_xl + body_xw);
    corner_x.push_back(x0.X + body_xl - body_xw);
    corner_x.push_back(x0.X - body_xl - body_xw);
    corner_x.push_back(x0.X - body_xl + body_xw);
    corner_x.push_back(x0.X + body_xl + body_xw);

    corner_y.push_back(x0.Y + body_yl + body_yw);
    corner_y.push_back(x0.Y + body_yl - body_yw);
    corner_y.push_back(x0.Y - body_yl - body_yw);
    corner_y.push_back(x0.Y - body_yl + body_yw);
    corner_y.push_back(x0.Y + body_yl + body_yw);

    plt::plot(corner_x,corner_y,"k-");
}
}