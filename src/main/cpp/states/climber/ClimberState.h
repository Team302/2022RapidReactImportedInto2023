//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================


#pragma once

//Team 302 Includes
#include <controllers/ControlData.h>
#include <controllers/DragonPID.h>
#include <controllers/MechanismTargetData.h>
#include <states/Mech2MotorState.h>
#include <subsys/Climber.h>

class ControlData;


class ClimberState : public Mech2MotorState
{
    public:
        ClimberState
        (
            ControlData*                    controlData,
            ControlData*                    controlData2,
            double                          target1,
            double                          target2,
            double                          robotPitch
        );

        ClimberState() = delete;
        ~ClimberState() = default;
        void Init() override;
        void Run() override;
        bool AtTarget() const override;

        double GetRobotPitch() const { return m_robotPitch; };

    private:
        double GetLiftHeight() const;
        double GetRotateAngle() const;

        bool LiftTargetReached() const;
        bool RotateTargetReached() const;

        Climber*                            m_climber;
        ControlData*                        m_liftControlData;
        ControlData*                        m_rotateControlData;
        double                              m_liftTarget; //Target position of climber lift in inches
        double                              m_rotateTarget; //Target position of rotating arm in degrees
        double                              m_robotPitch;
        DragonPID*                          m_liftController;
        DragonPID*                          m_rotateController;
        std::shared_ptr<IDragonMotorController>  m_liftMotor;
        std::shared_ptr<IDragonMotorController>  m_rotateMotor;
};