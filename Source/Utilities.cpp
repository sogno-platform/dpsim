#include "Utilities.h"

#include <iostream>

void updateProgressBar(double time, double finalTime) {

	if (time / finalTime <= 0.1) {
		std::cout << "                      (0%)\r";
	}
	else if (time / finalTime > 0.1 && time / finalTime <= 0.2) {
		std::cout << "##                    (10%)\r";
	}
	else if (time / finalTime > 0.2 && time / finalTime <= 0.3) {
		std::cout << "####                  (20%)\r";
	}
	else if (time / finalTime > 0.3 && time / finalTime <= 0.4) {
		std::cout << "######                (30%)\r";
	}
	else if (time / finalTime > 0.4 && time / finalTime <= 0.5) {
		std::cout << "########              (40%)\r";
	}
	else if (time / finalTime > 0.5 && time / finalTime <= 0.6) {
		std::cout << "##########            (50%)\r";
	}
	else if (time / finalTime > 0.6 && time / finalTime <= 0.7) {
		std::cout << "############          (60%)\r";
	}
	else if (time / finalTime > 0.7 && time / finalTime <= 0.8) {
		std::cout << "##############        (70%)\r";
	}
	else if (time / finalTime > 0.8 && time / finalTime <= 0.9) {
		std::cout << "################      (80%)\r";
	}
	else if (time / finalTime > 0.9 && time / finalTime < 1) {
		std::cout << "##################    (90%)\r";
	}
	else {
		std::cout << "####################  (100%)";
		std::cout << std::endl;
	}
}
