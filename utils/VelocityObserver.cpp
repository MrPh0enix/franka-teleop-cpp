
#include "VelocityObserver.h"


VelocityObserver::VelocityObserver(double g, double T, Method method)
    : g_(g), T_(T), method_(method), Y_k_prev_(0.0), U_k_prev_(0.0), initialised_(false)
{}


double VelocityObserver::update(double U_k) {

    if (!initialised_) {
        initialised_ = true;
        U_k_prev_ = U_k;
        return 0.0; 
    }

    double Y_k = 0.0;

    switch (method_) {
        case Method::EULER:
            Y_k = (1.0 / (g_*T_ + 1.0)) * (Y_k_prev_ + g_*U_k - g_*U_k_prev_);
            break;

        case Method::ALALOUI:
            Y_k = (1.0 / (7*g_*T_ + 8)) * ((8-g_*T_)*Y_k_prev_ + 8*g_*U_k - 8*g_*U_k_prev_); 
            break;

        case Method::ADAMS:
            Y_k = (1 / (3*g_*T_ + 2)) * ((g_*T_ + 2)*Y_k_prev_ + 2*g_*U_k - 2*g_*U_k_prev_);
            break;
    }

    Y_k_prev_ = Y_k;
    U_k_prev_ = U_k;
    return Y_k;
}


void VelocityObserver::reset() {
    Y_k_prev_ = 0.0;
    U_k_prev_ = 0.0;
    initialised_ = false;
}
