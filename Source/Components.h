#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Components/BaseComponent.h"
#include "Components/LinearResistor.h"
#include "Components/Capacitor.h"
#include "Components/Inductor.h"
#include "Components/CurrentSource.h"
#include "Components/VoltageSourceWithResistance.h"
#include "Components/InterfacedInductor.h"
#include "Components/SynchronGeneratorEMT.h"
#include "Components/SynchronGenerator.h"

enum class SynchGenStateType { perUnit, statorReferred };
enum class SynchGenParamType { perUnit, statorReferred };

#endif // !COMPONENTS_H