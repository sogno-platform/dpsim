#include "TwoWindingTransformer.h"

using namespace DPsim;

TwoWindingTransformer::TwoWindingTransformer(std::string name, int node1, int node2, Real resistance, Real inductance) : RxLine(name, node1, node2, resistance, inductance) {
}
