#ifndef CENTRALHUB_HPP
#define CENTRALHUB_HPP

#include <stdio.h> 
#include <string>
#include <numeric>

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"  
#include "messages/SensorData.h"

#include "libbsn/resource/Battery.hpp"
#include "libbsn/utils/utils.hpp"


class CentralHub : public arch::target_system::Component {

    public:
		CentralHub(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery);
    	~CentralHub();

	private:
    	CentralHub &operator=(const CentralHub &);

  	public:
        virtual void setUp() = 0;
    	virtual void tearDown() = 0;
        virtual int32_t run();
		void body();
        void apply_noise();

        void reconfigure(const archlib::AdaptationCommand::ConstPtr& msg);

        virtual void collect(const messages::SensorData::ConstPtr& sensor_data) = 0;
        virtual void process() = 0;
        virtual void transfer() = 0;

    private:
        bool isActive();
        void turnOn();
        void turnOff();
        void recharge();

    protected:
		bool active;
        int max_size;
        int total_buffer_size;
        std::array<int, 6> buffer_size;
		bsn::resource::Battery battery;
        std::vector<std::list<double>> data_buffer;
};

#endif 