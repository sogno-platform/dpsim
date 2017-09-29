#ifndef TWOWINDINGTRANSFORMER_H
#define TWOWINDINGTRANSFORMER_H

#include "RxLine.h"

namespace DPsim {
	// TODO: currently just modeled as an RxLine, possibly use more complex model?
	class TwoWindingTransformer : public RxLine {
	public:
		TwoWindingTransformer(std::string name, int node1, int node2, Real resistance, Real inductance);
	};
};

#endif
