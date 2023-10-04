#ifndef TANKPRESSCONTROLLERCLASS_H
#define TANKPRESSCONTROLLERCLASS_H

#include <Arduino.h>
#include "./States/ControllerStates.h"
#include "./States/SensorStates.hpp"
#include "./ValveClass.h"
#include "./Base_Classes/Timer.hpp"
#include "./base_classes/state_machine.hpp"
#include "./Base_Classes/Controller.hpp"
#include <string>

class bangSensorPID
{
private:
    float bangSensorEMA = 0; // This is the P term of this struct.
    float bangSensorIntegral = 0;
    float bangSensorDerivative = 0;

    bool nodeIDCheck = false; // Does this need a nodeIDCheck? I can't remember why I put this here. - Joe 2023 September 29

public:

    String name; // Debug string for Serial.prints in setInput.  Likely unnecessary. - Joe 2023 Sept 29 

    void setInput(float proportionalValue, float integralValue, float derivativeValue);
    float getEMA(){return bangSensorEMA;}
    float getIntegral(){return bangSensorIntegral;}
    float getDerivative(){return bangSensorDerivative;}
};

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
//     This class is not the same as the above bang sensor PID, which is used as
// args for this class, I think. - Joe Kessler, 2023 October 1
//
//     All setters for this class return true or false, depending on if the set 
// successfully wrote data or not.  Used with controllerConfigUpdate.  The 
// functions now have a _prefix to differentiate them from wrappers in the 
// controller.
// 
//     This controller doesn't appear to be in use anywhere.  It also explains
// why the sensors all have linearRegressionLeastSquared_PID and
// accumulatedI_PID_float despite seemingly doing nothing.
// 
// - Joe Kessler, 2023 October 3

class PID
{
public: // These should be private.
    // PID Constants.
    float K_p_Default = 1;
    float K_p = 1;

    float K_i_Default = 0; //initial Ki, KEEP 0 for tank press
    float K_i = 0; //initial Ki, KEEP 0 for tank press

    // I'm not sure what the run constant does? - Joe 2023 October 1
    float K_i_run_Default = 0; //bang run Ki
    float K_i_run = 0; //bang run Ki

    float K_d_Default = 0;
    float K_d = 0; 

    float targetValue_Default = 0;
    float targetValue = 0;

    float getKp(){return K_p;}
    float getKi(){return K_i;}
    float getKd(){return K_d;}
    float getTargetValue(){return targetValue;}

    bool _setK_p(float K_pin)
    {
        if (K_pin <= 1000 && K_pin >= -1000) // MAGIC NUMBERS >:(
        {
            K_p = K_pin;
            return true; // controllerConfigUpdate = true;
        }
        return false;
    }
    
    bool _setK_i(float K_iin)
    {
        if (K_iin <= 1000 && K_iin >= -1000) // MAGIC NUMBERS >:(
        {
            K_i = K_iin;
            return true;//controllerConfigUpdate = true;
        }
        return false;
    }

    bool _setK_i()   //empty input args means reset K_i to K_i_run
    {
        K_i = K_i_run;
        return false;
    }

    bool _setK_d(float K_din)
    {
        if (K_din <= 1000 && K_din >= -1000)
        {
            K_d = K_din;
            return true; // controllerConfigUpdate = true;
        }
        return false;
    }

    // Does this set controllerConfigUpdate? - Joe 2023 October 3
    void _setTargetValue(float controllerSetPointIn)
    {
        targetValue = controllerSetPointIn;
    } 

    // These variables are set nowhere, and only used in PIDmath. - Joe Kessler, 2023 October 3
    bool PIDmathPrintFlag = false;
    float timeStepPIDMath = 0;
    float controllerTimeStep = 0.01; //default to 100Hz assumption for controller refresh
    float p_rollingAve = 0;

    // Unused variables associated with the integral and derivative terms.
    int arrayMostRecentPositionPID = 0; 

    // PID function inputs.  Used with PIDmath.
    float e_p = 0;
    float e_i = 0;
    float e_d = 0;

    float getPfunc(){return e_p;}
    float getIfunc(){return e_i;}
    float getDfunc(){return e_d;}

    // PID function intermediate outputs. Used with PIDmath.
    float P_p = 0;
    float P_i = 0;
    float P_d = 0;
    float funcOutput = 0;

    float getP_p(){return P_p;}
    float getP_i(){return P_i;}
    float getP_d(){return P_d;}

    float _PIDmath()
    {
//PIDinputSetting();
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

        ////////////////////////////////////////////////////////////////////////
        //     This explains why the sensors all have 
        // linearRegressionLeastSquared_PID and accumulatedI_PID_float despite
        // seemingly doing nothing.  They're used here!  They seemed like too 
        // useful tools to abandon.  Perhaps its useful to generalize this 
        // controller too. For now, I'm going to keep it disrepaired.
        // - Joe 2023 October 3
        
        // PID function calculations - new integral differenced int style
        /*   e_p = controllerSetPoint - p_rollingAve;    // proportional offset calculation
        e_i += accumulatedI_PID_float(inputArrayPID, timeStepPIDMath, controllerSetPoint)* timeStepPIDMath;
        //e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
        e_d = linearRegressionLeastSquared_PID(inputArrayPID, 5, timeStepPIDMath);    // derivative function calculation */

        //
        P_p = K_p*(e_p);
        P_i = K_i*(e_i);
        P_d = K_d*(e_d * controllerTimeStep);

        // Numerical error guards
        if (isnan(P_p)) P_p = 0;
        if (isnan(P_i)) P_i = 0;
        if (isnan(P_d)) P_d = 0;
        
        funcOutput = (P_p) - (P_i) - (P_d); //still not 100% sure on signs, particularly the d

        // Should this commented line below use LinearMap? - Joe 2023 October 3
        // normalizes units to be in PSI
        //funcOutput = (K_p*(e_p)) - (K_i*(e_i/integrationSteps)) - (K_d*(e_d * timeStep)); //still not 100% sure on signs, particularly the d

        return funcOutput;
    }

    // Prints the results of PIDMath if PIDmathPrintFlag is true.
    void PIDprint()
    {
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
    }

    PID()
    : K_p_Default{1}, K_i_Default{0}, K_i_run_Default{0}, K_d_Default{0}, targetValue_Default{0},
      K_p{        1}, K_i{        0}, K_i_run{        0}, K_d{        0}, targetValue{        0}
    {
        // force K_i = 0 at instantiation - should move this now with new defaults setup
        K_i_run = K_i;  //stashes K_i value for later
        K_i = 0;
        //TankPressController::setK_i(0);
    }

    PID(float _K_p_Default, float _K_i_Default, float _K_d_Default, float _targetValue_Default)
    : K_p_Default{_K_p_Default}, K_i_Default{_K_i_Default}, K_d_Default{_K_d_Default}, targetValue_Default{_targetValue_Default},
      K_p{        _K_p_Default}, K_i{        _K_i_Default}, K_d{        _K_d_Default}, targetValue{        _targetValue_Default}
    {
        // force K_i = 0 at instantiation - should move this now with new defaults setup
        K_i_run = K_i;  //stashes K_i value for later
        K_i = 0;
        //TankPressController::setK_i(0);
    }

    void PIDresetAll()
    {
        K_p = K_p_Default;
        K_i = K_i_Default;
        K_i_run = K_i_run_Default;
        K_d = K_d_Default;
        targetValue = targetValue_Default;

        // force K_i = 0 at instantiation - should move this now with new defaults setup
        K_i_run = K_i;  //stashes K_i value for later
        K_i = 0;
        //TankPressController::setK_i(0);
    }

};

class TankPressController : public Controller<TankPressControllerState>, public Timer, public PID
{
private:
//const uint32_t controllerID;                          // Controller ID number 
//const uint8_t controllerNodeID;
//bool nodeIDCheck;                           // Whether this object should operate on this node
        bool isSystemBang;
//bool testPass = false;
//TankPressControllerState state;
//TankPressControllerState priorState;
//int64_t currentAutosequenceTime;
//SensorState sensorState;                    // Use one sensor state inside here to toggle all sensors on controller
        //elapsedMicros bangtimer;                        // timer for the valve, used for changing duty cycles, in MICROS
        ValveState pressLineVentStateBang1;
        ValveState pressLineVentStateBang2;
        Valve &primaryPressValve;
        Valve &pressLineVent;
        Valve &tankVent;
        //Valve MainValve{};
        bool abortFlag = false;         //controller can trigger an abort by flipping this flag true
        bool ventFailsafeArm = false;   // for setting whether a tank controller has the vent failsafe armed
        bool ventFailsafeFlag = false;  //for making vent failsafe require successive controller loops to open vents

        float ventFailsafePressure_Default;
        float ventFailsafePressure;

        float targetPcValue_Default;
        float targetPcValue;

        float tankToChamberDp_Default;
        float tankToChamberDp;

//float targetValue_Default;
//float targetValue;

//float K_p_Default = 1;
//float K_p = 1;
//
//float K_i_Default = 0; //initial Ki, KEEP 0 for tank press
//float K_i = 0; //initial Ki, KEEP 0 for tank press
//
//float K_i_run_Default = 0; //bang run Ki
//float K_i_run = 0; //bang run Ki
//
//float K_d_Default = 0;
//float K_d = 0;

        float controllerThreshold_Default = 1;
        float controllerThreshold = 1;
//float bangPIDoutput;

//float controllerTimeStep = 0.01; //default to 100Hz assumption for controller refresh
        float sensorIntervalTimeStep = 0.01; // Is this unused? It was next to controllerTimeStep intially, with the same value. - Joe 2023 October 3

//bool trustBangSensor1 = true;
//bool trustBangSensor2 = true;
//bool trustBangSensor3 = false;
//float bangSensor1EMA = 0;   //primary PT
//float bangSensor1Integral = 0;   //primary PT
//float bangSensor1Derivative = 0;   //primary PT
//
//float bangSensor2EMA = 0;   //secondary PT
//float bangSensor2Integral = 0;   //secondary PT
//float bangSensor2Derivative = 0;   //secondary PT
//
//float bangSensor3EMA = 0;   //simulated PT
//float bangSensor3Integral = 0;   //simulated PT
//float bangSensor3Derivative = 0;   //simulated PT

//bangSensorPID PIDSensor1, PIDSensor2, PIDSensor3;

//float bangSensorWeightedEMA = 0;   //weighted ave/trusted value - currently crude use
        
// Are these 3 ever used? It looks like old permanent input variables for before the KPE variables appeared. - Joe 2023 October 2
//float proportionalValue = 0;
//float integralValue = 0;
//float derivativeValue = 0;

        //do i need to create a float array for sensors here or can I pass a reference/similar?
//float funcOutput = 0;
//float p_rollingAve = 0;
//float P_p = 0;
//float P_i = 0;
//float P_d = 0;
//float e_p = 0;
//float e_i = 0;
//float e_d = 0;
//int arrayMostRecentPositionPID = 0;
//bool PIDmathPrintFlag = false;
//float timeStepPIDMath = 0;

        bool resetIntegralCalcBool = false;

        bool controllerUpdate = false;
//bool controllerConfigUpdate = false;
        elapsedMillis quasistaticUpdateTimer;
        bool quasistaticControllerUpdate = false;
        bool isWaterFlowSetup = true;

public:
        elapsedMicros bangtimer;                        // timer for the valve, used for changing duty cycles, in MICROS
        uint32_t valveMinimumEnergizeTime_Default = 75000;      // in MICROS
        uint32_t valveMinimumEnergizeTime;      // in MICROS

        uint32_t valveMinimumDeenergizeTime_Default = 50000;    // in MICROS
        uint32_t valveMinimumDeenergizeTime;    // in MICROS

    // constructor - hipress
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID,
        Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent,
        float setVentFailsafePressure_Default, bool setVentFailsafeArm = false,
         bool setIsSystemBang = false, bool setNodeIDCheck = false);
    // constructor - tank bangers
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID,
         Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent,
         float setTargetPcValue_Default, float setTankToChamberDp_Default,
         float setVentFailsafePressure_Default, float set_K_p_Default,
         float set_K_i_Default, float set_K_d_Default,
         float setControllerThreshold_Default, bool setVentFailsafeArm = false,
         bool isSystemBang = false, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
void begin();

    void resetAll(); // reset all configurable settings to defaults

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
void stateOperations();

    // access functions defined in place
    float getControllerThreshold(){return controllerThreshold;}
    void setControllerThreshold(float controllerThresholdIn)
    {
        if (controllerThresholdIn <= 100 && controllerThresholdIn >= 0)
        {
            controllerThreshold = controllerThresholdIn;
            controllerConfigUpdate = true;
        }
    }

    // get functions, return the current value of that variable
//uint32_t getControllerID(){return controllerID;}
//uint8_t getControllerNodeID(){return controllerNodeID;}
//bool getNodeIDCheck(){return nodeIDCheck;}
        bool getIsBang(){return isSystemBang;}
//float getTargetValue(){return targetValue;}
//float getControllerThreshold(){return controllerThreshold;}
//TankPressControllerState getState(){return state;}
//TankPressControllerState getPriorState(){return priorState;}
//SensorState getControllerSensorState(){return sensorState;}
        ValveState getPrimaryPressValveState(){return primaryPressValve.getState();}
        ValveState getPressLineVentState(){return pressLineVent.getState();}
        ValveState getTankVentState(){return tankVent.getState();}
        uint32_t getValveMinEnergizeTime(){return valveMinimumEnergizeTime;}
        uint32_t getValveMinDeEnergizeTime(){return valveMinimumDeenergizeTime;}
        float getVentFailsafePressure(){return ventFailsafePressure;}
        bool getVentFailsafeArm(){return ventFailsafeArm;}

        bool getControllerUpdate(){return controllerUpdate;}
//bool getControllerConfigUpdate(){return controllerConfigUpdate;}
        bool getQuasistaticControllerUpdate(){return quasistaticControllerUpdate;}
        void setQuasistaticControllerUpdate(bool quasistaticControllerUpdateIn){quasistaticControllerUpdate = quasistaticControllerUpdateIn;}
        elapsedMillis getQuasistaticUpdateTimer(){return quasistaticUpdateTimer;}
bool getAbortFlag(){return abortFlag;}

        bool getResetIntegralCalcBool()
            {
                bool tempBoolContainer;
                tempBoolContainer = resetIntegralCalcBool;
                resetIntegralCalcBool = false;              //default resets it to false
                return tempBoolContainer;
            }
        void resetIntegralCalc(bool resetIntIn){resetIntegralCalcBool = resetIntIn;}
        

    // set the Node ID Check bool function
void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    // controller state set function
// void setState(TankPressControllerState newState)
// {
//     if (newState != state)
//     {
//         priorState = state;
//         controllerConfigUpdate = true;
//     }
//     state = newState;
// }        
    // vent line setting - for bang bang with two tank controllers sharing vent line control
        void setPressVentLineStateBang1(ValveState ventLineSetIn) {pressLineVentStateBang1 = ventLineSetIn;}
    // vent line setting - for bang bang with two tank controllers sharing vent line control
        void setPressVentLineStateBang2(ValveState ventLineSetIn) {pressLineVentStateBang2 = ventLineSetIn;}

        void setPrimaryPressValveState(ValveState primaryPressValveStateIn) {if (primaryPressValveStateIn != ValveState::NullReturn) {primaryPressValve.setState(primaryPressValveStateIn);}}
        void setPressLineVentState(ValveState pressLineVentStateIn) {if (pressLineVentStateIn != ValveState::NullReturn) {pressLineVent.setState(pressLineVentStateIn);}}
        void setTankVentState(ValveState tankVentStateIn) {if (tankVentStateIn != ValveState::NullReturn) {tankVent.setState(tankVentStateIn);}}
        //test state set functions
        void testSetPrimaryPressValveState(ValveState primaryPressValveStateIn) {if(testPass) {primaryPressValve.setState(primaryPressValveStateIn);}}
        void testSetPressLineVentState(ValveState pressLineVentStateIn) {if(testPass) {pressLineVent.setState(pressLineVentStateIn);}}
        void testSetTankVentState(ValveState tankVentStateIn) {if(testPass) {tankVent.setState(tankVentStateIn);}}
    //setting functions - have all inputs bounded to catch nonsense CAN config msg inputs
        void setVentFailsafePressure(float ventFailsafePressureIn){if (ventFailsafePressureIn <= 10000 && ventFailsafePressureIn >= 0) {ventFailsafePressure = ventFailsafePressureIn;controllerConfigUpdate = true;}}
        void setVentFailsafeArm(bool armIn){ventFailsafeArm = armIn;}

        
        void setValveMinimumEnergizeTime(uint32_t valveMinimumEnergizeTimeIn){if(valveMinimumEnergizeTimeIn >= 0 && valveMinimumEnergizeTimeIn <= 10000){valveMinimumEnergizeTime = valveMinimumEnergizeTimeIn;controllerConfigUpdate = true;}}
        void setValveMinimumDeenergizeTime(uint32_t valveMinimumDeenergizeTimeIn){if(valveMinimumDeenergizeTimeIn >= 0 && valveMinimumDeenergizeTimeIn <= 10000){valveMinimumDeenergizeTime = valveMinimumDeenergizeTimeIn;controllerConfigUpdate = true;}}

//void setPcTarget(float PcTargetIn);
    
//// reset all configurable settings to defaults
//void resetAll();

    // autosequence get function
void setCurrentAutosequenceTime(int64_t countdownIn) {currentAutosequenceTime = countdownIn;}

    // set function for controllerUpdateBool that indicates if a new controller calc has been run
        void setControllerUpdate(bool controllerUpdateIn){controllerUpdate = controllerUpdateIn;}
//void setControllerConfigUpdate(bool controllerConfigUpdateIn){controllerConfigUpdate = controllerConfigUpdateIn;}
        void ventPressureCheck();


////////////////////////////////////////////////////////////////////////////////
// PID Controller Interface Variables and Functions
// Do NOT leave this here.  This is relocated for ease of refactoring.
// - Joe Kessler, 2023 October 3

    // BAD MOVE THIS NO DO NOT KEEP THIS HERE THIS IS TEMPORARY NO BAD
    // BE ORGANIZED THAT'S THE WHOLE POINT OF THIS REFACTOR - Joe
    bool trustBangSensor1 = true;
    bool trustBangSensor2 = true;
    bool trustBangSensor3 = false;
    bangSensorPID PIDSensor1; //primary PT
    bangSensorPID PIDSensor2; //secondary PT
    bangSensorPID PIDSensor3; //simulated PT

//float getPfunc(){return e_p;}
//float getIfunc(){return e_i;}
//float getDfunc(){return e_d;}
//float getP_p(){return P_p;}
//float getP_i(){return P_i;}
//float getP_d(){return P_d;}

// set functions, allows the setting of a variable
//void setPIDSensorInput1(float proportionalValue, float integralValue, float derivativeValue);
//void setPIDSensorInput2(float proportionalValue, float integralValue, float derivativeValue);
//void setPIDSensorInput3(float proportionalValue, float integralValue, float derivativeValue);

//float getKp(){return K_p;}
//float getKi(){return K_i;}
//float getKd(){return K_d;}

//void setK_p(float K_pin){if (K_pin <= 1000 && K_pin >= -1000) {K_p = K_pin;controllerConfigUpdate = true;}}
//void setK_i(float K_iin){if (K_iin <= 1000 && K_iin >= -1000) {K_i = K_iin;controllerConfigUpdate = true;}}
//void setK_i(){K_i = K_i_run;}   //empty input args means reset K_i to K_i_run
//void setK_d(float K_din){if (K_din <= 1000 && K_din >= -1000) {K_d = K_din;controllerConfigUpdate = true;}}

    // Wrappers to pass a PID controller setting update. - Joe 2023 October 3
    void setK_p(float K_pin){controllerConfigUpdate = controllerConfigUpdate || _setK_i(K_pin);}
    void setK_i(float K_iin){controllerConfigUpdate = controllerConfigUpdate || _setK_i(K_iin);}
    void setK_i()           {controllerConfigUpdate = controllerConfigUpdate || _setK_i();}
    void setK_d(float K_din){controllerConfigUpdate = controllerConfigUpdate || _setK_i(K_din);}

    // PID math wrapper variables and functions
    float PIDmath_old();
    float bangPIDoutput;
    float bangSensorWeightedEMA = 0;   //weighted ave/trusted value - currently crude use. This is only used with PIDInputSetting, and ventPressureCheck. - Joe 2023 October 3
    void PIDinputSetting(); //logic for which input source to use for PID values
    float PIDmath(); // Wrapper function for the PID controller. This used to be called PIDmath. - Joe 2023 October 3
    float getPIDoutput(){return bangPIDoutput;}

    // controller set point function
    //float getTargetValue(){return targetValue;}
    //void setControllerTargetValue(float controllerSetPointIn){targetValue = controllerSetPointIn;} 
    void setPcTarget(float PcTargetIn); // Guarded setter for targetValue, does an implicit unit conversion. - Joe 2023 October 3
};


#endif