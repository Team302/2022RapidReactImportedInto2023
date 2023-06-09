
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

// C++ Includes
#include <map>

// FRC includes

// Team 302 includes
#include <gamepad/TeleopControl.h>
#include <states/intake/LeftIntakeStateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>

// Third Party Includes

using namespace std;


LeftIntakeStateMgr* LeftIntakeStateMgr::m_instance = nullptr;
LeftIntakeStateMgr* LeftIntakeStateMgr::GetInstance()
{
	if ( LeftIntakeStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto intake = mechFactory->GetLeftIntake();
        if (intake != nullptr)
        {
		    LeftIntakeStateMgr::m_instance = new LeftIntakeStateMgr();
        }
	}
	return LeftIntakeStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
LeftIntakeStateMgr::LeftIntakeStateMgr() : IntakeStateMgr()
{
    map<string, StateStruc> stateMap;
    stateMap[m_intakeOffXmlString]      = m_offState;
    stateMap[m_intakeIntakeXmlString]   = m_intakeState;
    stateMap[m_intakeExpelXmlString]    = m_expelState;
    stateMap[m_intakeRetractXmlString]  = m_retractState;

    Init(MechanismFactory::GetMechanismFactory()->GetLeftIntake(), stateMap);
}   

Intake* LeftIntakeStateMgr::GetIntake() const 
{
    return MechanismFactory::GetMechanismFactory()->GetLeftIntake();
}
bool LeftIntakeStateMgr::IsIntakePressed() const 
{
    auto controller = TeleopControl::GetInstance();
    return controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) : false;
}
bool LeftIntakeStateMgr::IsExpelPressed() const 
{
    auto controller = TeleopControl::GetInstance();
    return controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXPEL_LEFT) : false;
}
bool LeftIntakeStateMgr::IsRetractSelected() const
{
    auto controller = TeleopControl::GetInstance();
    return controller != nullptr ? controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_RETRACT_LEFT) > 0.1 : false;
}
