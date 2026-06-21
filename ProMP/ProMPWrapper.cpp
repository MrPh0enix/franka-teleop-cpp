
#include "ProMPWrapper.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <cmath>
#include <iostream>


namespace py = pybind11;



ProMPWrapper::ProMPWrapper(
    const std::string& demos_path,
    int basis,
    int dof,
    int Nd,
    int n_samples
) {
    py::scoped_interpreter guard{};


    try {

        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("append")("../ProMP"); 

        py::module_ Full_ProMP = py::module_::import("Full_ProMP");
        py::module_ np = py::module_::import("numpy");
    
        py::list trajectoriesList;
        py::list timeList;

        for (int i = 1; i <= Nd; ++i) {
            py::tuple output = Full_ProMP.attr("Franka_data2")(demos_path, i);

            py::object joints_raw = output[0];
            py::object times_raw  = output[1];

            int length = py::len(joints_raw);

            py::object indices = np.attr("linspace")(
                0, length-1, n_samples, py::arg("dtype") = np.attr("int_")
            );

            py::object joints_resampled = np.attr("asarray")(py::list());

            py::list joints_list;
            py::list times_list;

            for (auto idx_obj : indices) {
                int idx = idx_obj.cast<int>();
                joints_list.append(joints_raw.attr("__getitem__")(idx));
                times_list.append(times_raw.attr("__getitem__")(idx));
            }

            trajectoriesList.append(np.attr("asarray")(joints_list));
            timeList.append(np.attr("asarray")(times_list));
        }


        int n_data = py::len(trajectoriesList[0]);
        py::object Time = np.attr("linspace")(0.0, 1.0, n_data);
        
        py::object ProMPClass = Full_ProMP.attr("ProMp");
        py::object LearnerClass = Full_ProMP.attr("Learner");

        py::object training_ProMP = ProMPClass(basis, dof, n_data);
        
        py::object learner = LearnerClass(training_ProMP);
        learner.attr("LearningFromData")(trajectoriesList, timeList);

        py::object ProMP_trained = ProMPClass(basis, dof, n_data);
        ProMP_trained.attr("mu")  = training_ProMP.attr("mu");
        ProMP_trained.attr("cov") = training_ProMP.attr("cov");

        py::object promp_trajectory = ProMP_trained.attr("trajectory_samples")(Time, 1);
        py::tuple mean_cov = ProMP_trained.attr("trajectory_mean_cov")(Time);

        py::object meanTraj_py = mean_cov[0];
        py::object covTraj_py  = mean_cov[1];

        py::tuple mean_std = ProMP_trained.attr("trajectory_mean_std")(Time);

        meanTraj_py = mean_std[0];
        py::object stdTraj_py = mean_std[1];

        this->meanTraj = meanTraj_py.cast<std::vector<std::vector<double>>>();
        this->stdTraj = stdTraj_py.cast<std::vector<std::vector<double>>>();

        std::cout << "ProMP compiled successfully." << std::endl;


    } catch (const py::error_already_set& e) {
        std::cerr << "Python error: " << e.what() << std::endl;
        throw;
    }

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



std::array<double, 7> ProMPWrapper::get_init_pos() const {
    if (meanTraj.empty()) {
        throw std::runtime_error("meanTraj is empty.");
    }

    std::vector<double> v = meanTraj[0];

    std::array<double, 7> arr;
    std::copy_n(v.begin(), 7, arr.begin());

    return arr;
}