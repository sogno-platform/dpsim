// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;
using namespace std;

int main(int argc, char* argv[]) {

	Attribute<Real>::Ptr attr = AttributeStatic<Real>::make(0.001);

	Real read1 = **attr; //read1 = 0.001
	cout << "attr value: " << read1 << endl;

	**attr = 0.002;
	Real read2 = **attr; //read2 = 0.002
	cout << "attr value: " << read2 << endl;
		
	attr->set(0.003);
	Real read3 = **attr; //read3 = 0.003
	cout << "attr value: " << read3 << endl;

	return 0;
}
