#pragma once

#include <vector>
#include <string>



class ProMPWrapper {

public:

    struct Result {
        std::vector<double> desired_joint_positions;
        std::vector<double> current_stdDev;
        int corresponding_iteration;
    };

    ProMPWrapper(
        const std::string& demos_path,
        int basis = 8,
        int dof = 7,
        int Nd = 12,
        int n_samples = 200
    );

    Result eucledean_dist_pos(const std::array<double,7>& real_time_joint_angles);

    std::array<double, 7> get_init_pos() const;

private:
    std::vector<std::vector<double>> meanTraj;
    std::vector<std::vector<double>> stdTraj;

};