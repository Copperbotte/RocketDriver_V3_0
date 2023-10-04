#include "TankPressControllerClass.h"
#include <Arduino.h>

TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setVentFailsafePressure_Default, bool setVentFailsafeArm = false, bool setIsSystemBang, bool setNodeIDCheck) 
//    : Controller{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, ventFailsafePressure_Default{setVentFailsafePressure_Default}, ventFailsafeArm{setVentFailsafeArm}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
    : Controller{setControllerID, setControllerNodeID, setNodeIDCheck},
    PID{},
    primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent},
    tankVent{*setTankVent},
    ventFailsafePressure_Default{setVentFailsafePressure_Default},
    ventFailsafeArm{setVentFailsafeArm},
    isSystemBang{setIsSystemBang}
{
    // Instantiation stuff?

    // The compiler is whining at me if these are a constructor - Joe 2023 Sept 29
    PIDSensor1.name = "bang1";
    PIDSensor2.name = "bang2";
    PIDSensor3.name = "bang3";
}
TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setTargetPcValue_Default, float setTankToChamberDp_Default, float setVentFailsafePressure_Default, float set_K_p_Default, float set_K_i_Default, float set_K_d_Default, float setControllerThreshold_Default, bool setVentFailsafeArm = false, bool setIsSystemBang, bool setNodeIDCheck) 
//    : Controller{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, targetPcValue_Default{setTargetPcValue_Default}, tankToChamberDp_Default{setTankToChamberDp_Default}, ventFailsafePressure_Default{setVentFailsafePressure_Default}, K_p_Default{set_K_p_Default}, K_i_Default{set_K_i_Default}, K_d_Default{set_K_d_Default}, controllerThreshold_Default{setControllerThreshold_Default}, ventFailsafeArm{setVentFailsafeArm}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
    : Controller{setControllerID, setControllerNodeID, setNodeIDCheck},
      PID{set_K_p_Default, set_K_i_Default, set_K_d_Default, setTargetPcValue_Default + setTankToChamberDp_Default},
      primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent},
      tankVent{*setTankVent}, targetPcValue_Default{setTargetPcValue_Default},
      tankToChamberDp_Default{setTankToChamberDp_Default},
      ventFailsafePressure_Default{setVentFailsafePressure_Default},
//K_p_Default{set_K_p_Default}, K_i_Default{set_K_i_Default},
//K_d_Default{set_K_d_Default},
      controllerThreshold_Default{setControllerThreshold_Default},
      ventFailsafeArm{setVentFailsafeArm}, isSystemBang{setIsSystemBang}
{
    // Instantiate operational values from the default values given
    // Allows a reset to defaults after having changed settings via config messages
    targetPcValue = targetPcValue_Default;
    tankToChamberDp = tankToChamberDp_Default;
    //_setTargetValue(targetValue_Default);
    //_setTargetValue(targetPcValue + tankToChamberDp);

//K_p = K_p_Default;
//K_i = K_i_Default;
//K_d = K_d_Default;
    ventFailsafePressure = ventFailsafePressure_Default;
    controllerThreshold = controllerThreshold_Default;
    valveMinimumDeenergizeTime = valveMinimumDeenergizeTime_Default;
    valveMinimumEnergizeTime = valveMinimumEnergizeTime_Default;
    
//// force K_i = 0 at instantiation - should move this now with new defaults setup
//K_i_run = K_i;  //stashes K_i value for later
//K_i = 0;
////TankPressController::setK_i(0);

    // The compiler is whining at me if these are a constructor - Joe 2023 Sept 29
    PIDSensor1.name = "bang1";
    PIDSensor2.name = "bang2";
    PIDSensor3.name = "bang3";
}

void TankPressController::begin()
{
    if (nodeIDCheck)
    {
        // setup stuff?
    }
}

void TankPressController::ventPressureCheck()
{
    if (ventFailsafeArm)
    {
    // tank sensors still have bang names, but they're really just the tank pressure readings
    if (bangSensorWeightedEMA >= ventFailsafePressure)
    {
        if (ventFailsafeFlag)
        {
        tankVent.setState(ValveState::OpenCommanded);
        }
        ventFailsafeFlag = true;
        //Serial.print(PIDSensor1.getEMA()); //bangSensor1EMA);
        //Serial.print(" : ");
        //Serial.println(ventFailsafePressure);
    }
    else ventFailsafeFlag = false;
    }
}

void TankPressController::stateOperations()
{
    //run the PID calculation each time state operations runs
    //timer use real timestep here?
    // Each time this is run, set controllerUpdate to true
    controllerUpdate = true;

    if (isSystemBang)
    {
        // Hold the reset integral calc bool true each cycle while in T minus to prevent windup
        if (currentAutosequenceTime <= 0)
        {
            resetIntegralCalcBool = true;
        }
        // Run PID math on BangBang Tank Controllers
        
        //bangPIDoutput = PIDmath();
        PIDmath();
    }

    ////////////////////////////////////////////////////////////////////////////
    //     I'm breaking the following switch statement into a set of switch 
    // statements to rearrange state logic with state selection.  The underlying
    // logic should be invariant under this change, but if this change makes it
    // harder to read, please revert the change.
    // - Joe Kessler, 2023 September 29

    TankPressControllerState CurrentState = getState();

    // Handle pre state check flags
    switch (CurrentState)
    {
    case TankPressControllerState::Passive:
        testPass = false;
        //set abortFlag false when going to passive;
        abortFlag = false;
        break;
    case TankPressControllerState::Standby:
        testPass = false;
        //set abortFlag false when going to passive;
        abortFlag = false;
        break;
    case TankPressControllerState::RegPressActive:
        testPass = false;
        break;
    case TankPressControllerState::Armed:
        testPass = false;
        break;
    case TankPressControllerState::PropTankVent:
        testPass = false;
        //set abortFlag false going into Vent to be able to vent out of an Abort from abortFlag
        abortFlag = false;
        break;
    case TankPressControllerState::HiVent:
        testPass = false;
        //set abortFlag false going into Vent to be able to vent out of an Abort from abortFlag
        abortFlag = false;
        break;
    case TankPressControllerState::Abort:
        testPass = false;
        break;
    case TankPressControllerState::HiPressPassthroughVent:  //uhh what the fuck was this, the valve states are ???
        testPass = false;
        break;
    case TankPressControllerState::TestPassthrough:
        testPass = true;
        break;
    case TankPressControllerState::OffNominalPassthrough:
        testPass = true;
        break;
    case TankPressControllerState::AutosequenceCommanded:
        testPass = false;
        break;
    case TankPressControllerState::BangBangActive:
        testPass = false;
        break;
    default:
        break;
    }

    // Handle whether the state has changed
    if(getPriorState() != CurrentState)
    {
        switch (CurrentState)
        {
        case TankPressControllerState::Standby:
            //don't do shit
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::CloseCommanded);
            tankVent.setState(         ValveState::CloseCommanded);
            sensorState = SensorState::Slow;
            break;
        case TankPressControllerState::RegPressActive:
            //do shit
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::OpenCommanded);
            pressLineVent.setState(    ValveState::CloseCommanded);
            tankVent.setState(         ValveState::CloseCommanded);
            ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
            break;
        case TankPressControllerState::Armed:
            // Arming turns sensor read rates up to operational levels before opening valves
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::CloseCommanded);
            tankVent.setState(         ValveState::CloseCommanded);
            break;
        case TankPressControllerState::PropTankVent:
            //Serial.println("dis u? ");
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::OpenCommanded);
            tankVent.setState(         ValveState::OpenCommanded);
            break;
        case TankPressControllerState::HiVent:
            //Serial.println("dis u? ");
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::OpenCommanded);
            tankVent.setState(         ValveState::OpenCommanded);
            break;
        case TankPressControllerState::Abort:
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::CloseCommanded);
            tankVent.setState(         ValveState::CloseCommanded);
            ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
            break;
        case TankPressControllerState::HiPressPassthroughVent:  //uhh what the fuck was this, the valve states are ???
            sensorState = SensorState::Fast;
            primaryPressValve.setState(ValveState::CloseCommanded);
            pressLineVent.setState(    ValveState::OpenCommanded);
            tankVent.setState(         ValveState::CloseCommanded);
            ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
            break;
        default:
            break;
        }
    }

    // Previous state irrelevant
    switch (CurrentState)
    {
    case TankPressControllerState::Passive:
        //don't do shit
        //if (priorState != TankPressControllerState::Passive)
        //{
        //primaryPressValve.setState(ValveState::CloseCommanded);
        //pressLineVent.setState(ValveState::CloseCommanded);
        //tankVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        //}
        break;
    case TankPressControllerState::Armed:
        // Armed had this function outside its state check.  Should all states do this? - Joe, 2023 September 29
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::TestPassthrough:
        sensorState = SensorState::Slow;
        break;
    case TankPressControllerState::OffNominalPassthrough:
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::AutosequenceCommanded:
        // If specific press routine is on autosequence, include a state switch on timer in here
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::BangBangActive:
        //minimum bang time lockouts, once they are up the valves go to plain Open/Closed states which unlocks them to be commanded again
        if (primaryPressValve.getState() == ValveState::BangingOpen || primaryPressValve.getState() == ValveState::BangOpenProcess)
        {
            if (bangtimer >= valveMinimumEnergizeTime)    // X ms opening/closing time
            {
            primaryPressValve.setState(ValveState::Open);
            }
        }
        if (primaryPressValve.getState() == ValveState::BangingClosed)
        {
            if (bangtimer >= valveMinimumDeenergizeTime)    // X ms opening/closing time
            {
            primaryPressValve.setState(ValveState::Closed);
            }
        }
        // Update ValveState if Open/Closed based on PID controller output
        if (bangPIDoutput > (controllerThreshold))
        {
            //open valve
            if (primaryPressValve.getState() == ValveState::Closed)
            {
            primaryPressValve.setState(ValveState::BangOpenCommanded);
            bangtimer = 0;
            }
            
        }
        if (bangPIDoutput < ((-1)*controllerThreshold))
        {
            //close valve
            if (primaryPressValve.getState() == ValveState::Open)
            {
            primaryPressValve.setState(ValveState::BangCloseCommanded);
            bangtimer = 0;
            }
        }
        sensorState = SensorState::Fast;
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    default:
        break;
    }

    // Controller State switch case
//    switch (CurrentState)
//    {
//    case TankPressControllerState::Passive:
//        testPass = false;
//        //set abortFlag false when going to passive;
//        abortFlag = false;
//        //don't do shit
//        //if (priorState != TankPressControllerState::Passive)
//        //{
//        //primaryPressValve.setState(ValveState::CloseCommanded);
//        //pressLineVent.setState(ValveState::CloseCommanded);
//        //tankVent.setState(ValveState::CloseCommanded);
//        sensorState = SensorState::Slow;
//        //}
//        break;
//    case TankPressControllerState::Standby:
//        testPass = false;
//        //set abortFlag false when going to passive;
//        abortFlag = false;
//        //don't do shit
//        if (getPriorState() != TankPressControllerState::Standby)
//        {
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::CloseCommanded);
//        tankVent.setState(ValveState::CloseCommanded);
//        sensorState = SensorState::Slow;
//        }
//        break;
//    case TankPressControllerState::RegPressActive:
//        testPass = false;
//        //do shit
//        if (getPriorState() != TankPressControllerState::RegPressActive)
//        {
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::OpenCommanded);
//        pressLineVent.setState(ValveState::CloseCommanded);
//        tankVent.setState(ValveState::CloseCommanded);
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        }
//        break;
//    case TankPressControllerState::Armed:
//        testPass = false;
//        if (getPriorState() != TankPressControllerState::Armed)
//        {
//        // Arming turns sensor read rates up to operational levels before opening valves
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::CloseCommanded);
//        tankVent.setState(ValveState::CloseCommanded);
//        }
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        break;
//    case TankPressControllerState::PropTankVent:
//        testPass = false;
//        //set abortFlag false going into Vent to be able to vent out of an Abort from abortFlag
//        abortFlag = false;
//        if (getPriorState() != TankPressControllerState::PropTankVent)
//        {
//            //Serial.println("dis u? ");
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::OpenCommanded);
//        tankVent.setState(ValveState::OpenCommanded);
//        }
//        break;
//    case TankPressControllerState::HiVent:
//        testPass = false;
//        //set abortFlag false going into Vent to be able to vent out of an Abort from abortFlag
//        abortFlag = false;
//        if (getPriorState() != TankPressControllerState::HiVent)
//        {
//            //Serial.println("dis u? ");
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::OpenCommanded);
//        tankVent.setState(ValveState::OpenCommanded);
//        }
//        break;
//    case TankPressControllerState::Abort:
//        testPass = false;
//        if (getPriorState() != TankPressControllerState::Abort)
//        {
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::CloseCommanded);
//        tankVent.setState(ValveState::CloseCommanded);
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        }
//        break;
//    case TankPressControllerState::HiPressPassthroughVent:  //uhh what the fuck was this, the valve states are ???
//        testPass = false;
//        if (getPriorState() != TankPressControllerState::HiPressPassthroughVent)
//        {
//        sensorState = SensorState::Fast;
//        primaryPressValve.setState(ValveState::CloseCommanded);
//        pressLineVent.setState(ValveState::OpenCommanded);
//        tankVent.setState(ValveState::CloseCommanded);
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        }
//        break;
//    case TankPressControllerState::TestPassthrough:
//        sensorState = SensorState::Slow;
//        //
//        testPass = true;
//        break;
//    case TankPressControllerState::OffNominalPassthrough:
//        //
//        testPass = true;
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        break;
//    case TankPressControllerState::AutosequenceCommanded:
//        testPass = false;
//        // If specific press routine is on autosequence, include a state switch on timer in here
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        break;
//    case TankPressControllerState::BangBangActive:
//        testPass = false;
//        //minimum bang time lockouts, once they are up the valves go to plain Open/Closed states which unlocks them to be commanded again
//        if (primaryPressValve.getState() == ValveState::BangingOpen || primaryPressValve.getState() == ValveState::BangOpenProcess)
//        {
//            if (bangtimer >= valveMinimumEnergizeTime)    // X ms opening/closing time
//            {
//            primaryPressValve.setState(ValveState::Open);
//            }
//        }
//        if (primaryPressValve.getState() == ValveState::BangingClosed)
//        {
//            if (bangtimer >= valveMinimumDeenergizeTime)    // X ms opening/closing time
//            {
//            primaryPressValve.setState(ValveState::Closed);
//            }
//        }
//        // Update ValveState if Open/Closed based on PID controller output
//        if (bangPIDoutput > (controllerThreshold))
//        {
//            //open valve
//            if (primaryPressValve.getState() == ValveState::Closed)
//            {
//            primaryPressValve.setState(ValveState::BangOpenCommanded);
//            bangtimer = 0;
//            }
//            
//        }
//        if (bangPIDoutput < ((-1)*controllerThreshold))
//        {
//            //close valve
//            if (primaryPressValve.getState() == ValveState::Open)
//            {
//            primaryPressValve.setState(ValveState::BangCloseCommanded);
//            bangtimer = 0;
//            }
//        }
//        sensorState = SensorState::Fast;
//        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
//        break;
//    default:
//        break;
//    }

//External bangbang vent line logic
/* if (pressLineVentStateBang1 != pressLineVentStateBang2)
{
    //logic for when they don't agree - I should explicitly define all possible states but that's tomorrow me's problem
    //Most important one, allows the vent line opened if either bang tank controller commands it
    if (pressLineVentStateBang1 == ValveState::OpenCommanded || pressLineVentStateBang2 == ValveState::OpenCommanded)
    {
        pressLineVent.setState(ValveState::OpenCommanded);
    }

}
else
{
    pressLineVent.setState(pressLineVentStateBang1);
} */

/* // controller Valve object stateOperations
    primaryPressValve.controllerStateOperations();
    pressLineVent.controllerStateOperations();
    tankVent.controllerStateOperations(); */
}

void bangSensorPID::setInput(float proportionalValue, float integralValue, float derivativeValue)
{
    if (nodeIDCheck)
    {
        bangSensorEMA = proportionalValue;
        bangSensorIntegral = integralValue;
        bangSensorDerivative = derivativeValue;
        
        //Serial.print(name);
        //Serial.print(" ins: ");
        //Serial.print(bangSensorEMA);
        //Serial.print(" : ");
        //Serial.print(bangSensorIntegral);
        //Serial.print(" : ");
        //Serial.println(bangSensorDerivative);
    }
}

//void TankPressController::setPIDSensorInput1(float proportionalValue, float integralValue, float derivativeValue)
//{
//    PIDSensor1.setInput(proportionalValue, integralValue, derivativeValue);
//
//    if (nodeIDCheck)
//    {
//    bangSensor1EMA = proportionalValue;
//    bangSensor1Integral = integralValue;
//    bangSensor1Derivative = derivativeValue;
//    }
//}
//void TankPressController::setPIDSensorInput2(float proportionalValue, float integralValue, float derivativeValue)
//{
//    if (nodeIDCheck)
//    {
//    bangSensor2EMA = proportionalValue;
//    bangSensor2Integral = integralValue;
//    bangSensor2Derivative = derivativeValue;
//    }
//}
//void TankPressController::setPIDSensorInput3(float proportionalValue, float integralValue, float derivativeValue)
//{
//    if (nodeIDCheck)
//    {
//    bangSensor3EMA = proportionalValue;
//    bangSensor3Integral = integralValue;
//    bangSensor3Derivative = derivativeValue;
///*     Serial.print("bang3 ins: ");
//    Serial.print(bangSensor3EMA);
//    Serial.print(" : ");
//    Serial.print(bangSensor3Integral);
//    Serial.print(" : ");
//    Serial.println(bangSensor3Derivative); */
//    }
//}

////////////////////////////////////////////////////////////////////////////////
// Logic for which input source to use for PID values.
//     The existence of this function kinda breaks how a PID typically works, so
// I can't entirely separate the two. - Joe Kessler 2023 October 3
void TankPressController::PIDinputSetting()
{
    if (trustBangSensor1)
    {
        bangSensorWeightedEMA = PIDSensor1.getEMA(); //bangSensor1EMA;
        e_p = getTargetValue() - PIDSensor1.getEMA(); //bangSensor1EMA;
        e_i = PIDSensor1.getIntegral(); //bangSensor1Integral;
        e_d = PIDSensor1.getDerivative(); //bangSensor1Derivative;
    }
    else if (trustBangSensor2)
    {
        bangSensorWeightedEMA = PIDSensor2.getEMA(); //bangSensor2EMA;
        e_p = getTargetValue() - PIDSensor2.getEMA(); //bangSensor2EMA;
        e_i = PIDSensor2.getIntegral(); //bangSensor2Integral;
        e_d = PIDSensor2.getDerivative(); //bangSensor2Derivative;
    }
    else if (trustBangSensor3)
    {
        bangSensorWeightedEMA = PIDSensor3.getEMA(); //bangSensor3EMA;
        e_p = getTargetValue() - PIDSensor3.getEMA(); //bangSensor3EMA;
        e_i = PIDSensor3.getIntegral(); //bangSensor3Integral;
        e_d = PIDSensor3.getDerivative(); //bangSensor3Derivative;
    }
    else
    {
        // if controller doesn't trust ANY tank sensor, even the sim, trigger an abort
        abortFlag = true;
    }
}

////////////////////////////////////////////////////////////////////////////////
//     This appears to be a PID controller that's been partially disassembled.
// I'm not going to repair it much, instead I'm going to package this into its 
// own class and include it with this controller.  Its unlikely we need this
// here, instead of being with the collection of classes used in Sensors.hpp.
// 
//     The K terms appear to be the PID constants.  I believe the P terms are 
// the individual outputs, before being summed as funcOutput.  The E terms 
// appear to be the PID inputs, selected through PIDinputSetting().
// 
// - Joe Kessler 2023 October 1
////////////////////////////////////////////////////////////////////////////////


//     This function is a temporary wrapper as the PID controller is seperated 
// from the tank controller. - Joe 2023 October 3
float TankPressController::PIDmath()
{
    PIDinputSetting();
    bangPIDoutput = _PIDmath();
    PIDprint();
    return bangPIDoutput;
}

//float PIDmath(float inputArrayPID[], float controllerSetPoint, float timeStepPIDMath, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d)
float TankPressController::PIDmath_old()
{
    PIDinputSetting();
    //timeStepPIDMath = 1;
    funcOutput = 0;
    p_rollingAve = 0;
//P_p = 0;
//P_i = 0;
//P_d = 0;
    /*   e_p = 0;
    //e_i = 0;
    e_d = 0; */
    //arrayMostRecentPositionPID = static_cast<int>(inputArrayPID[1]+0.5);

    // PID function calculations - new integral differenced int style
    /*   e_p = controllerSetPoint - p_rollingAve;    // proportional offset calculation
    e_i += accumulatedI_PID_float(inputArrayPID, timeStepPIDMath, controllerSetPoint)* timeStepPIDMath;
    //e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
    e_d = linearRegressionLeastSquared_PID(inputArrayPID, 5, timeStepPIDMath);    // derivative function calculation */

    //
    P_p = K_p*(e_p);
    P_i = K_i*(e_i);
    P_d = K_d*(e_d * controllerTimeStep);

    if (isnan(P_p)) P_p = 0;
    if (isnan(P_i)) P_i = 0;
    if (isnan(P_d)) P_d = 0;
    
    funcOutput = (P_p) - (P_i) - (P_d); //still not 100% sure on signs, particularly the d

    // normalizes units to be in PSI
    //funcOutput = (K_p*(e_p)) - (K_i*(e_i/integrationSteps)) - (K_d*(e_d * timeStep)); //still not 100% sure on signs, particularly the d

    if (PIDmathPrintFlag)
    {
        Serial.println("insidePID: ");
        Serial.print(K_p);
        Serial.print(" : ");
        Serial.print(e_p);
        Serial.print(" : ");
        Serial.println(K_p*(e_p));
        Serial.print(K_i);
        Serial.print(" : ");
        Serial.print(e_i);
        Serial.print(" : ");
        Serial.println(K_i*(e_i));
        Serial.print(K_d);  
        Serial.print(" : ");
        Serial.print(e_d);  
        Serial.print(" : ");
        Serial.println(K_d*(e_d * timeStepPIDMath));  
        Serial.println(funcOutput);
    }
        //Serial.print("insidePID p_rollingAve,");
        //Serial.print(p_rollingAve);
        //Serial.println(",");

    return funcOutput;
}


// Guarded setter for targetValue, does an implicit unit conversion. - Joe 2023 October 3
// Not the same as setting targetValue directly.
void TankPressController::setPcTarget(float PcTargetIn)
{
    if (PcTargetIn <= 600 && PcTargetIn >= 200) // MAGIC NUMBERS - Joe 2023 October 3
    {
        // Only continue if the new target is different than current target
        if (targetPcValue != PcTargetIn)
        {
            resetIntegralCalcBool = true;
            targetPcValue = PcTargetIn;
            // set target point based on dP, make the math function later
            //_setTargetValue(targetPcValue + tankToChamberDp);
            if (!isWaterFlowSetup)
            {
                _setTargetValue(targetPcValue*1.25); //very crude dP approx
            }
            else 
            {
                _setTargetValue(targetPcValue*0.25); //very crude dP approx
            }
        }
    }
}


void TankPressController::resetAll()
{
    ventFailsafePressure = ventFailsafePressure_Default;
    targetPcValue = targetPcValue_Default;
    tankToChamberDp = tankToChamberDp_Default;

    controllerThreshold = controllerThreshold_Default;
    valveMinimumDeenergizeTime = valveMinimumDeenergizeTime_Default;
    valveMinimumEnergizeTime = valveMinimumEnergizeTime_Default;
    
//K_p = K_d_Default;
//K_i = K_i_Default;
//K_d = K_d_Default;
//// force K_i = 0 at instantiation - should move this now with new defaults setup
//K_i_run = K_i;  //stashes K_i value for later
//K_i = 0;
//_setTargetValue(targetValue_Default);
//_setTargetValue(targetPcValue + tankToChamberDp);

    PIDresetAll();

}