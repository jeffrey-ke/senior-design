#include "WaypointController.h"

WaypointController::WaypointController(double kp, 
                                       double ki, 
                                       double kd, 
                                       double kol): k_p_(kp), 
                                                    k_i_(ki),
                                                    k_d_(k_d_),
                                                    k_ol_(kol) {}


