#include "ExampleClass.h"

using namespace example_ns;

int main(int argc, char **argv) {
	ExampleClass example(argc, argv);
	example.run();
	ros::spin();
	return 0;
}
