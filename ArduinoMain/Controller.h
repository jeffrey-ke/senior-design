#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Msgs.h"

class Controller {

    public: //constants
        constexpr static double DT = 1.0;
        constexpr static double ZERO = 0.0;

    public: // Constructors
        Controller(double kp, double ki, double kd, double kol=0.0): k_p_(kp), k_i_(ki), k_d_(kd), k_ol_(kol), integrated_error_{}{};

    public: // methods
        double CalculateControlEffort(double current_state);
        void SetDesired(double desired) {desired_ = desired;};
    
    public: // Setters
        void SetKp(double kp) {k_p_ = kp;};
        void SetKi(double ki) {k_i_ = ki;};
        void SetKd(double kd) {k_d_ = kd;};
        void SetKol(double kol) {k_ol_ = kol;};
        void ResetIntegratedError() {integrated_error_ = 0.0;};

    public: //Getters
        double GetKp() const {return k_p_;};
        double GetKi() const {return k_i_;};
        double GetKd() const {return k_d_;};
        double GetKol() const {return k_ol_;};
        double GetDesired() const {return desired_;};


    private: // member vars
        double desired_;
        double current_;

        double k_p_, k_i_, k_d_, k_ol_;
        double integrated_error_;
        double previous_error_;

        double total_control_effort_;
        double proportional_control_effort_;
        double integral_control_effort_;
        double derivative_control_effort_;
        double ol_control_effort_;


    private: //helper functions
        double CalculateError(double actual) {return desired_ - actual;};
        double CalculateProportional(double error);
        double CalculateIntegral(double error);
        double CalculateDerivative(double error);
        double CalculateOpenLoop();

};



#endif