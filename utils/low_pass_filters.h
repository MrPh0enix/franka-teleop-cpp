#pragma once

class low_pass_filter {
    public:

        enum class Method {
            EULER,
            ALALOUI,
            ADAMS
        };

        low_pass_filter(double g, double T, Method method = Method::ALALOUI);

        double update(double U_k);
        void reset();


    private:
        double g_;
        double T_;
        Method method_;
        double Y_k_prev_;
        double U_k_prev_;
        bool   initialised_;
};