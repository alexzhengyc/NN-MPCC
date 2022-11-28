// Copyright 2019 Alexander Liniger
// Modifications copyright 2022 Alex Zheng

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

#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {

    using namespace mpcc;
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    Track track = Track(json_paths.track_path); 
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log1;
    std::list<MPCReturn> log2;
    std::list<MPCReturn> log3;

    // MPC 1: Ideal model: Ground Truth
    // MPC 2: Neural network model 
    // MPC 3: Physics-based model with 10% parameter error 

    MPC mpc1(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    MPC mpc2(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    MPC mpc3(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);

    mpc1.setTrack(track_xy.X,track_xy.Y);
    mpc2.setTrack(track_xy.X,track_xy.Y);
    mpc3.setTrack(track_xy.X,track_xy.Y);
    
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {   
        MPCReturn mpc_sol = mpc1.runMPC(x0, i, 1);

        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log1.push_back(mpc_sol);
    }
    
    x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {   
        MPCReturn mpc_sol = mpc2.runMPC(x0, i, 2);

        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log2.push_back(mpc_sol);
    }

    x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {   
        MPCReturn mpc_sol = mpc3.runMPC(x0, i, 3);

        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log2.push_back(mpc_sol);
    }

    plotter.plotSim3(log1, log2, log3, track_xy);
    plotter.plotRun3(log1, log2, log3, track_xy);

    double mean_time1 = 0.0, mean_time2 = 0.0;
    double max_time1 = 0.0, max_time2 = 0.0, secmax_time2 = 0.0;
    for(MPCReturn log_i : log1)
    {
        mean_time1 += log_i.time_total;
        if(log_i.time_total > max_time1)
            max_time1 = log_i.time_total;
    }
    for(MPCReturn log_i : log2)
    {   
        mean_time2 += log_i.time_total;
        if(log_i.time_total > max_time2)
        max_time2 = log_i.time_total;
        else if(log_i.time_total > secmax_time2)
        secmax_time2 = log_i.time_total;
    
    }

    std::cout << "mean nmpc time " << mean_time1/double(jsonConfig["n_sim"]) 
    << "  " << mean_time2/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time1 << "  " << max_time2  << " " << secmax_time2 << std::endl;
    return 0;
}


