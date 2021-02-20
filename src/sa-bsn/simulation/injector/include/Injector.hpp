#ifndef INJECTOR_HPP
#define INJECTOR_HPP

#include "ros/ros.h"
#include <ros/package.h>

#include <time.h>
#include <map>
#include <vector>

#include "archlib/Uncertainty.h"
#include "archlib/ROSComponent.hpp"

#include "libbsn/utils/utils.hpp"

class Injector : public arch::ROSComponent {

	public:
    	Injector(int &argc, char **argv, const std::string &name);
    	virtual ~Injector();

    private:
      	Injector(const Injector &);
    	Injector &operator=(const Injector &);

		int64_t seconds_in_cycles(const double &seconds);
		int64_t cycles_in_seconds(const double &cycles);
		double gen_noise(const std::string &component, double &noise, int &duration, double &amplitude, std::string &type);

	public:
		virtual void setUp();
		virtual void tearDown();
		virtual void body();

		void inject(const std::string &component, const std::string &content);

	private:
		ros::NodeHandle handle;
		int cycles;

		std::map<std::string, ros::Publisher> uncertainty_pub;

		std::vector<std::string> components;

		std::map<std::string, double> noise_factor;
		std::map<std::string, int> duration;
		std::map<std::string, double> frequency;
		std::map<std::string, double> amplitude;
		std::map<std::string, int> begin;
		std::map<std::string, int> end;
		std::map<std::string, std::string> type;



		ros::Publisher log_uncertainty;
};

#endif 