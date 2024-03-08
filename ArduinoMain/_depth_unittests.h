#include <ArduinoUnit.h>
#include "_helper_tests.h"
#include "DepthController.h"
#include "units.h"
#include "Msgs.h"
/*
(1) test integral windup
(2) test down and back up again
*/

test(integral_windup){
    DepthController d(4, 0.2, 0, 10);
    meters desired_depth{10.0};
    d.SetDesiredDepth(desired_depth);
  
    meters depth1{1.0}, depth2{1.0}, depth3{1.0};
    Msg::PWM pwm1 = d.CalculateControlEffort(depth1);
    /*
        error = 9.0
        kp = 4
        ki = 0.1

        control_effort = kp * error + ki * error
                        = 4.0 * 9.0 + 0.2 * 9.0 
                        = 37.8
                      
        expected_pwm = {1500 + 37/2, 1500 + 37/2, 1500, 1500}   

        ==========
        error2 = 9.0
        kp = 4
        ki = 0.1  
        control_effort = kp * error + ki * (error + 9.0)
                        = 4.0 * 9.0 + 0.2 * 18.0 
                        = 39.6   
        expected_pwm = {1500 + 37/2, 1500 + 37/2, 1500, 1500} 

        ===========
        error3 = 9.0
        control_effort = kp * error + ki * (error + 9.0 + 9.0)
                        = 4.0 * 9.0 + 0.2 * 27.0 
                        = 41.4  
    */
    Msg::PWM expected_pwm1{1518, 1518, 1500, 1500};
    assertPWMEqual(pwm1, expected_pwm1);


    Msg::PWM pwm2 = d.CalculateControlEffort(depth2);
    Msg::PWM expected_pwm2{1519, 1519, 1500, 1500};
    assertPWMEqual(pwm2, expected_pwm2);

    Msg::PWM pwm3 = d.CalculateControlEffort(depth3);
    Msg::PWM expected_pwm3{1520, 1520, 1500, 1500};
    assertPWMEqual(pwm3, expected_pwm3);

}

test(down_and_back_up){
    DepthController d(4, 0.2, 0, 10);
    meters desired_depth{20.0};
    d.SetDesiredDepth(desired_depth);
    meters depth1{0.0}, depth2{1.0}, depth3{3.0}, depth4{10.0}, depth5{15.0};

    auto pwm1 = d.CalculateControlEffort(depth1);
    Msg::PWM expectedpwm1{1521, 1521, 1500, 1500};
    assertPWMEqual(pwm1, expectedpwm1);
    /*
      kp = 4
      ki = 0.2
      error = 10
      effort = 4 * 10 + 0.2 * 10 = 42
      expected1 = {1521 1521 1500 1500}
    */

    auto pwm2 = d.CalculateControlEffort(depth2);
    Msg::PWM expectedpwm2{1519, 1519, 1500, 1500};
    assertPWMEqual(pwm2, expectedpwm2);
    /*
      kp = 4
      ki = 0.2
      error = 9
      effort = 4 * 9 + 0.2 * (10 + 9) = 
      expected1 = {1519 1519 1500 1500}
    */

    auto pwm3 = d.CalculateControlEffort(depth3);
    Msg::PWM expectedpwm3{1516, 1516, 1500, 1500};
    assertPWMEqual(pwm3, expectedpwm3);
    /*
      kp = 4
      ki = 0.2
      error = 7
      effort = 4 * 7 + 0.2 * (10 + 9 + 7) = 
      expected1 = {1516 1516 1500 1500}

    */
    auto pwm4 = d.CalculateControlEffort(depth4);
    Msg::PWM expectedpwm4{1502, 1502, 1500, 1500};
    assertPWMEqual(pwm3, expectedpwm3);
    /*
      kp = 4
      ki = 0.2
      error = 0
      effort = 4 * 0 + 0.2 * (10 + 9 + 7 + 0) = 
      expected1 = {1502 1502 1500 1500}

    */

    auto pwm5 = d.CalculateControlEffort(depth5);
    Msg::PWM expectedpwm5{1493, 1493, 1500, 1500};
    assertPWMEqual(pwm5, expectedpwm5);
    /*
      kp = 4
      ki = 0.2
      error = -5
      effort = 4 * -5 + 0.2 * (10 + 9 + 7 + -5) = 
      expected1 = {1493 1493 1500 1500}

    */


}

