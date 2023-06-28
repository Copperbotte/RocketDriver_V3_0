
// 2023 Feb 11
// Current Authors: 
//     Joseph Kessler (joseph.b.kessler@gmail.com)
// 
////////////////////////////////////////////////////////////////////////////////
// This performs logical unit tests for the State_Machine base class.

#include "unity.h"
#include <iostream>
#include "Base_Classes/State_Machine.hpp"
#include "States/SensorStates.hpp"
#include <string>

void setUp(void) {}
void tearDown(void){}

// https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityAssertionsReference.md
// https://docs.platformio.org/en/stable/advanced/unit-testing/frameworks/unity.html
void test_pass(void)
{
	TEST_PASS();
}

void test_fail(void)
{
	TEST_FAIL();
}

void test_stateMachine_getters(void)
{
	StateMachine<SensorState> SM;
	
	// https://stackoverflow.com/questions/261963/how-can-i-iterate-over-an-enum
	for(int S1=SensorState::Off; S1 != SensorState::Calibration; S1++)
	{
		for(int S2=SensorState::Off; S2 != SensorState::Calibration; S2++)
		{
			SensorState SC = static_cast<SensorState>(S1);
			SensorState SP = static_cast<SensorState>(S2);
			SM._setInitialValues(SC, SP);
            if(SC == SensorState::Medium && SP == SensorState::Slow)
                SP = SensorState::Calibration;
			if(SM.getState() != SC || SM.getPriorState() != SP)
			{
                UnityPrint("Current State:");
                UnityPrint(std::to_string(SC).c_str());
                TEST_FAIL();
				return;
			}
		}
	}
	
	TEST_PASS();
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
	RUN_TEST(test_pass);
	RUN_TEST(test_fail);
	RUN_TEST(test_stateMachine_getters);
    UnityPrint("Hello");
    UNITY_END();
}

