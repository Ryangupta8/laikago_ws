#include "laikago_controller/joint_controller.h"
#include "laikago_controller/laikago_control_tool.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace laikago_controller;

int main(int argc, char *argv[])
{
	LaikagoJointController controller;
	double p, i, d, i_max, i_min;

	p = 5; i = 1; d = 5; i_max = 5; i_min = 0.01;

	controller.setGains(p, i, d, i_max, i_min);
	// controller.getGains();

}
