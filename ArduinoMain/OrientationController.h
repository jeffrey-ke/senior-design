#ifndef O_CONTROLLER_H
#define O_CONTROLLER_H
#include "Controller.h"
#include "Msgs.h"
#include "IMUDriver.h"

// ONLY controls ___ (insert whatever axis we're controlling)
class OrientationController {
    public: //aliases
        enum class AXES {X, Y, Z};
        static constexpr auto YAW = AXES::X;
        static constexpr auto ROLL = AXES::Y;
        static constexpr auto PITCH = AXES::Z;
    public: //methods
        OrientationController(): con_x_(0, 0, 0), con_y_(0, 0, 0), con_z_(0, 0, 0){}
        void SetDesiredToVertical();
        void SetDesiredToHorizontal();
        void SetDesiredOrientation(degrees desired, AXES axis);
        Msg::PWM CalculateControlEffort(Msg::RPY current_orientation); 
        void SetGains(double kp, double ki, double kd, AXES axis);

        bool IsAngleAchieved(degrees current, degrees margin, AXES axis);
        degrees GetAngleError(degrees desired, degrees current);

        bool IsVertical(Msg::RPY current_orientation, degrees margin=5);

 
    public: //getters
        degrees GetDesiredOrientation(AXES axis);

    private: //members
        Controller con_x_;
        Controller con_y_;
        Controller con_z_;
};


#endif