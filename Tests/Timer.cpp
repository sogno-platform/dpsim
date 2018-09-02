#include <dpsim/Timer.h>
#include <dpsim/Utils.h>

using namespace DPsim;

int main(int argc, char *argv[]) {
	Timer t;

	CommandLineArgs args(argc, argv);

	Timer::Ticks dt = Timer::Ticks(static_cast<Timer::Ticks::rep>(args.timeStep * 1e9));

	t.setStartTime(args.startTime);
	t.setInterval(dt);

	if (args.startTime > Timer::StartTimePoint())
		std::cout << "Start clock at: " << args.startTime << std::endl;
	else
		std::cout << "Start clock immeadiatly" << std::endl;

	std::cout << "Timer interval: " << dt << std::endl;

	t.start();

	while (t.ticks() * args.timeStep < args.duration) {
		t.sleep();

		std::cout << ".";
		std::cout.flush();
	}

	t.stop();
}
