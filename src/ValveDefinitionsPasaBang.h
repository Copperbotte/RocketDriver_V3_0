#ifndef VALVEDEFINITIONSPASABANG_H
#define VALVEDEFINITIONSPASABANG_H

#include "ValveClass.h"
#include <array>
#include "ALARApinDefines.h"
#pragma once

// Define number of valves here
#define NUM_VALVES 7

// Declare all Valve Objects here using ValveClass, and add them to the valveArray
// On Pasafire Node
Valve HiPressVent{17, 8, NormalClosed, 6, 25000, false};
Valve LoxMV{24, 8, NormalClosed, 8, 25000, false, 50};
Valve FuelMV{25, 8, NormalClosed, 5, 25000, false, 50};
Valve LoxVent{18, 8, NormalOpen, 1, 100000, false, 50};
Valve LoxBang{19, 8, NormalClosed, 2, 25000, false};
Valve FuelVent{21, 8, NormalClosed, 4, 25000, false};
Valve FuelBang{22, 8, NormalClosed, 3, 25000, false};

// ADD VALVES TO THIS VALVE ARRAY IN THE FORM: &VALVE
std::array<Valve*, NUM_VALVES> valveArray{&HiPressVent, &LoxMV, &FuelMV, &LoxVent, &LoxBang, &FuelVent, &FuelBang};

#endif