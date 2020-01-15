#ifndef ROSCOMPONENT_DESCRIPTOR_H
#define ROSCOMPONENT_DESCRIPTOR_H

#include <string>

#include "ros/ros.h"

namespace arch {
	class ROSComponentDescriptor {

		public:
			ROSComponentDescriptor();
			virtual ~ROSComponentDescriptor();

		private:
			ROSComponentDescriptor(const ROSComponentDescriptor &);
			ROSComponentDescriptor &operator=(const ROSComponentDescriptor &);

		public:
			void setName(const std::string &name);
			std::string getName() const;

			void setFreq(const double &freq);
			double getFreq() const;

		private:
			std::string name;
			double freq;
	};
}


#endif