#ifdef __linux__
#ifndef SHMEMTEST_H
#define SHMEMTEST_H

namespace DPsim {

	void shmemExample();
	void shmemRTExample();
	void shmemDistributed(int argc, char *argv[]);
	void shmemDistributedDirect(int argc, char *argv[]);
	void shmemDistributedRef();
}

#endif
#endif
