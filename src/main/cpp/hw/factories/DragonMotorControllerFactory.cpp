
#include <map>
#include <string>

#include <hw/factories/DragonMotorControllerFactory.h>        
#include <hw/usages/MotorControllerUsage.h>
#include <hw/DragonTalon.h>
#include <hw/DragonFalcon.h>
#include <utils/Logger.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/FeedbackDevice.h>

using namespace std;
using namespace ctre::phoenix::motorcontrol;

DragonMotorControllerFactory* DragonMotorControllerFactory::m_instance = nullptr;


DragonMotorControllerFactory* DragonMotorControllerFactory::GetInstance()
{
    if ( DragonMotorControllerFactory::m_instance == nullptr )
    {
        DragonMotorControllerFactory::m_instance = new DragonMotorControllerFactory();
    }
    return DragonMotorControllerFactory::m_instance;
}

DragonMotorControllerFactory::DragonMotorControllerFactory() 
{
	for ( auto inx=0; inx<63; ++inx )
	{
		m_canControllers[inx] = nullptr;
	}
    MotorControllerUsage::GetInstance();
    CreateTypeMap();
}

//=======================================================================================
// Method:          CreateMotorController
// Description:     Create a motor controller from the inputs
// Returns:         Void
//=======================================================================================
shared_ptr<IDragonMotorController> DragonMotorControllerFactory::CreateMotorController
(
	string		                                    mtype,
    int 											canID,
	int 											pdpID,
    string                                          usage,
    bool 											inverted, 
    bool 											sensorInverted,
    FeedbackDevice  	                            feedbackDevice,
    int 											countsPerRev,
    double                                          countsPerInch,
    double                                          countsPerDegree,
    float 											gearRatio,
    bool 											brakeMode,
    int 											followMotor,
    int 											peakCurrentDuration,
    int 											continuousCurrentLimit,
    int 											peakCurrentLimit,
    bool 											enableCurrentLimit,
    bool											forwardLimitSwitch,
    bool											forwardLimitSwitchNormallyOpen,
    bool											reverseLimitSwitch,
    bool											reverseLimitSwitchNormallyOpen,
    double											voltageCompensationSaturation,
    bool											enableVoltageCompensation,
    IDragonMotorController::MOTOR_TYPE                                                          motorType

)
{
    shared_ptr<IDragonMotorController> controller;

    auto hasError = false;
    
    auto type = m_typeMap.find(mtype)->second;
    if ( type == MOTOR_TYPE::TALONSRX )
    {
        auto talon = new DragonTalon( MotorControllerUsage::GetInstance()->GetUsage(usage), canID, pdpID, countsPerRev, gearRatio, countsPerInch, countsPerDegree, motorType);
        talon->EnableBrakeMode( brakeMode );
        talon->Invert( inverted );
        talon->SetSensorInverted( sensorInverted );
        talon->ConfigSelectedFeedbackSensor( feedbackDevice, 0, 50 );
        talon->ConfigSelectedFeedbackSensor( feedbackDevice, 1, 50 );

        talon->ConfigPeakCurrentLimit( peakCurrentLimit, 50 );
        talon->ConfigPeakCurrentDuration( peakCurrentDuration, 50 );
        talon->ConfigContinuousCurrentLimit( continuousCurrentLimit, 50 );
        talon->EnableCurrentLimiting( enableCurrentLimit );
        if ( forwardLimitSwitch )
        {
            talon->SetForwardLimitSwitch(forwardLimitSwitchNormallyOpen);
        }        
        if ( reverseLimitSwitch )
        {
            talon->SetReverseLimitSwitch(reverseLimitSwitchNormallyOpen);
        }

        if ( followMotor > -1 )
        {
            talon->SetAsFollowerMotor( followMotor );
        }
        controller.reset( talon );

        if (enableVoltageCompensation)
        {
            talon->EnableVoltageCompensation(voltageCompensationSaturation);
        }
    }
    else if ( type == MOTOR_TYPE::FALCON )
    {
        auto talon = new DragonFalcon( MotorControllerUsage::GetInstance()->GetUsage(usage), canID, pdpID, countsPerRev, gearRatio, countsPerInch, countsPerDegree, motorType);
        talon->EnableBrakeMode( brakeMode );
        talon->Invert( inverted );
        /**
        talon->SetSensorInverted( sensorInverted );
        **/
        talon->ConfigSelectedFeedbackSensor( feedbackDevice, 0, 50 );
        talon->ConfigSelectedFeedbackSensor( feedbackDevice, 1, 50 );

        if ( forwardLimitSwitch )
        {
            talon->SetForwardLimitSwitch(forwardLimitSwitchNormallyOpen);
        }        
        if ( reverseLimitSwitch )
        {
            talon->SetReverseLimitSwitch(reverseLimitSwitchNormallyOpen);
        }
        
        if ( followMotor > -1 )
        {
            talon->SetAsFollowerMotor( followMotor );
        }

        talon->ConfigPeakCurrentLimit( peakCurrentLimit, 50 );
        talon->ConfigPeakCurrentDuration( peakCurrentDuration, 50 );
        talon->ConfigContinuousCurrentLimit( continuousCurrentLimit, 50 );
        talon->EnableCurrentLimiting( enableCurrentLimit );

        if (enableVoltageCompensation)
        {
            talon->EnableVoltageCompensation(voltageCompensationSaturation);
        }

        /** **/
        controller.reset( talon );
    }
    else
    {
        hasError = true;
    }

    if ( !hasError )
    {
        m_canControllers[ canID ] = controller;
    }
	return controller;
}



//=======================================================================================
// Method:          GetController
// Description:     return motor controller
// Returns:         IDragonMotorController* 	may be nullptr if there isn't a controller
//												with this CAN ID.
//=======================================================================================
shared_ptr<IDragonMotorController> DragonMotorControllerFactory::GetController
(
	int							canID		/// Motor controller CAN ID
) const
{
	shared_ptr<IDragonMotorController> controller;
	if ( canID > -1 && canID < 63 )
	{
		controller = m_canControllers[ canID ];
	}
	else
	{
	    string msg = "invalid CAN ID ";
	    msg += to_string( canID );
        Logger::GetLogger()->LogError( "DragonMotorControllerFactory::GetController", msg );
	}
	return controller;
}

void DragonMotorControllerFactory::CreateTypeMap()
{
    m_typeMap["TALONSRX"] = DragonMotorControllerFactory::MOTOR_TYPE::TALONSRX;
    m_typeMap["FALCON"] = DragonMotorControllerFactory::MOTOR_TYPE::FALCON;
}
