#ifndef RELIABILITYENGINE_HPP
#define RELIABILITYENGINE_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <algorithm>

#include "ros/ros.h"
#include "ros/package.h"

#include "libbsn/goalmodel/Node.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"
#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/GoalTree.hpp"
#include "libbsn/model/Formula.hpp"
#include "libbsn/utils/utils.hpp"

#include "lepton/Lepton.h"

#include "archlib/DataAccessRequest.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/EnergyStatus.h"
#include "archlib/ROSComponent.hpp"
#include "archlib/EngineRequest.h"

#include "engine/Engine.hpp"

class ReliabilityEngine : public Engine {
	
	public: 
		ReliabilityEngine(int &argc, char **argv, std::string name);
    	virtual ~ReliabilityEngine();

    private:
      	ReliabilityEngine(const ReliabilityEngine &);
    	ReliabilityEngine &operator=(const ReliabilityEngine &);

  	public:
        void setUp();
    	void tearDown();
        
		std::string get_prefix();

		std::map<std::string, int> initialize_priority(std::vector<std::string>);
		std::map<std::string, double> initialize_strategy(std::vector<std::string>);

		void monitor();
    	void analyze();
		void plan();
    	void execute();

    private: 
        double setpoint;
		double offset;
		double gain;
        double tolerance;

        int cycles;

		std::string prefix;

		ros::Publisher enact;
};

#endif 