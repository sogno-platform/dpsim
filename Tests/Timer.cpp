#include <dpsim/Timer.h>
#include <dpsim/Utils.h>

using namespace DPsim;

int main(int argc, char *argv[]) {
	Timer t;

	CommandLineArgs args(argc, argv);

	t.setStartTime(args.startTime);
	t.setInterval(args.timeStep);

	if (args.startTime == Timer::StartTimePoint())
		args.startTime = Timer::StartClock::now();

	std::cout << "Start clock at: " << args.startTime << "(in " << args.startTime - Timer::StartClock::now() << ")" << std::endl;
	std::cout << "Timer interval: " << t.interval() << std::endl;

	t.start();

	while (t.ticks() * args.timeStep < args.duration) {
		t.sleep();

		std::cout << ".";
		std::cout.flush();
	}

	t.stop();
}
