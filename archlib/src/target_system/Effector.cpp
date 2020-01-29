#include "archlib/target_system/Effector.hpp"

namespace arch {
	namespace target_system {

		Effector::Effector(int &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name) {}
		Effector::~Effector() {}

        void Effector::setUp() {
			double freq;
			handle.getParam("frequency", freq);
			rosComponentDescriptor.setFreq(freq);
		}
		
        void Effector::tearDown() {}
	}
}