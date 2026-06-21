
#include "ProMPWrapper.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <cmath>


namespace py = pybind11;



ProMPWrapper::ProMPWrapper(
    const std::string& samples_path,
    int basis,
    int dof,
    int Nd,
    int n_samples
) {
    py::scoped_interpreter guard{};


    try {

    } catch (const py::error_already_set& e) {
        std::cerr << "Python error: " << e.what() << std::endl;
        throw;
    }



    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")("../ProMP");

    py::module_ adaptive_pos = py::module_::import("adaptive_positioning");
    auto result = adaptive_pos.attr("load_promp")(samples_path, basis, dof, Nd, n_samples);

    meanTraj = result["meanTraj"].cast<std::vector<std::vector<double>>>();
    stdTraj = result["stdTraj"].cast<std::vector<std::vector<double>>>();
    n_data = meanTraj.size();
}


ProMPWrapper::Result ProMPWrapper::eucledean_dist_pos(const std::array<double,7>& real_time_joint_angles) {
    
    double min_distance = std::numeric_limits<double>::infinity();
    int corresponding_iteration = -1;

    for (int i = 0; i<meanTraj.size(); i++) {
        double distance = 0.0;
        for (int j=0; j<real_time_joint_angles.size(); j++) {
            distance += pow(meanTraj[i][j] - real_time_joint_angles[j], 2);
        }

        if (distance < min_distance) {
            min_distance = distance;
            corresponding_iteration = i;
        }
    }

    return {
        meanTraj[corresponding_iteration],
        stdTraj[corresponding_iteration],
        corresponding_iteration
    };
}