
#ifndef BASECLASS_STATEMACHINE_H_
#define BASECLASS_STATEMACHINE_H_

// This class defines the Valve Object that will be used to represent and actuate the valves
// Run begin to set the pins

// 2023 Feb 10
// Current Authors: 
//     Joseph Kessler (joseph.b.kessler@gmail.com)
// 
////////////////////////////////////////////////////////////////////////////////
//     This is a template state machine class that requires an enum of states.
// It is inherited by a class using class Class : StateMachine<StateType> with a
// generic typename.
// 
template<typename StateType>
class StateMachine
{
private:
    StateType _state;
    StateType _priorState;                           // Tracks the valve state
    int64_t _fireSequenceActuation;              // time on autosequence to actuate IF FireCommanded is used
    int64_t _currentAutoSequenceTime;              // time of autosequence for comparison under FireCommanded

protected:
    void _setInitialValues(StateType state, StateType priorState)
    {
        _state = state;
        _priorState = priorState;
        //_fireSequenceActuation = fireSequenceActuation; // This is unused typically? odd.
    }

    // Reverts the current state to the previous state.
    //     Assumes the previous state is legal. Be careful, calling this twice 
    // does nothing!
    void _revertState()
    {
        _state = _priorState;
    }

public:
    StateType getState(){return _state;}
    StateType getPriorState(){return _priorState;}
    int64_t getFireTime(){return _fireSequenceActuation;}
    int64_t getCurrentAutoSequenceTime(){return _currentAutoSequenceTime;}

    void setState(StateType newState, int64_t fireTimeIn) 
    {
        if (newState != _state)
        {
            _priorState = _state;
            _state = newState;

            _fireSequenceActuation = fireTimeIn; // This is the only place fireSequenceActuaion is set.
            // Generally, this version of the function is only called in EngineController.
        }
    }

    //     Convinience overloaded function. Should compile away.  If it doesn't,
    // then copy and paste the above code and write it manually without
    // fireSequenceActuation.
    void setState(StateType newState) 
    {
        setState(newState, _fireSequenceActuation);
    }

    void setFireTime(int64_t fireSequenceActuation)
    {
        _fireSequenceActuation = fireSequenceActuation;
    }

    void setCurrentAutoSequenceTime(int64_t timeSetIn)
    {
        _currentAutoSequenceTime = timeSetIn;
    }
};

#endif