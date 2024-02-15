#include "Controller.h" 

double Controller::CalculateControlEffort(double current_state) {

    current_ = current_state;

    auto error = CalculateError(current_state);
    auto p_effort = CalculateProportional(error);
    auto i_effort = CalculateIntegral(error);
    auto d_effort = CalculateDerivative(error);
    auto ol_effort = CalculateOpenLoop();
    total_control_effort_ = p_effort + 
                            i_effort + 
                            d_effort + 
                            ol_effort;

    return total_control_effort_;
}

double Controller::CalculateProportional(double error) {
    auto effort = k_p_ * error;
    proportional_control_effort_ = effort;
    return effort;
}

double Controller::CalculateIntegral(double error) {
    integrated_error_ += error * DT;
    integral_control_effort_ = k_i_ * integrated_error_;
    return integral_control_effort_;
}

double Controller::CalculateDerivative(double error) {
    derivative_control_effort_ = k_d_ * (error - previous_error_)/DT;
    previous_error_ = error;
    return derivative_control_effort_;
}

double Controller::CalculateOpenLoop() {
    ol_control_effort_ = k_ol_ * desired_;
    return ol_control_effort_;
}