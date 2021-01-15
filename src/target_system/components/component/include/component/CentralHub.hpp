#ifndef CENTRALHUB_HPP
#define CENTRALHUB_HPP

#include <stdio.h> 
#include <string>
#include <numeric>
#include <fstream>

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"
#include "messages/SensorData.h"
#include "messages/DiagnosticsData.h"
#include "messages/CentralhubDiagnostics.h"

#include "bsn/resource/Battery.hpp"
#include "bsn/utils/utils.hpp"

#include <ros/package.h>
#include "boost/date_time/posix_time/posix_time.hpp"

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
        void flushData(messages::CentralhubDiagnostics);

    private:
        bool isActive();
        void turnOn();
        void turnOff();
        void recharge();

    protected:
		bool active;
        int max_size;
        int total_buffer_size;
        std::array<int, 5> buffer_size;
		bsn::resource::Battery battery;
        std::vector<std::list<double>> data_buffer;

        ros::Publisher statusPub;
        std::string timestamp;

        std::fstream fp;
        std::string filepath;
        std::string foldername;

};

#endif 