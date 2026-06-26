#include <pybind11/embed.h>
#include <iostream>
#include <array>
#include <pybind11/eigen.h>



namespace py = pybind11;

int main() {
    py::scoped_interpreter guard{};
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("../ProMP");

    py::module adaptive_pos = py::module::import("adaptive_positioning");

    py::object result = adaptive_pos.attr("get_init_pos")();
    Eigen::Matrix<double, 7, 1> init_pos = result.cast<Eigen::Matrix<double, 7, 1>>();

    std::cout << init_pos << std::endl;
    
    return 0;
}