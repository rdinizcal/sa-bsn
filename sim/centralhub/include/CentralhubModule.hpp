#ifndef CENTRALHUB_MODULE_HPP
#define CENTRALHUB_MODULE_HPP

#include <fstream>
#include <chrono>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/base/FIFOQueue.h"

#include "bsn/processor/Processor.hpp"
#include "bsn/communication/TCPSend.hpp"

#include "bsn/msg/info/TaskInfo.hpp"
#include "bsn/msg/info/ContextInfo.hpp"
#include "bsn/msg/info/MonitorTaskInfo.hpp"
#include "bsn/msg/data/SensorData.h"
#include "bsn/msg/control/CentralHubControlCommand.hpp"



class CentralhubModule : public odcore::base::module::TimeTriggeredConferenceClientModule{
    private:
        CentralhubModule(const CentralhubModule & /*obj*/);
        CentralhubModule &operator=(const CentralhubModule & /*obj*/);
        virtual void setUp();
        virtual void tearDown();   

		void sendTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		
        void sendMonitorTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);

    public:
        CentralhubModule(const int32_t &argc, char **argv);
        virtual ~CentralhubModule();

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

    private:
        odcore::base::FIFOQueue buffer;
        odcore::base::FIFOQueue localQueue;

        bool active;
		std::map<std::string,double> params;

        uint32_t connect;
        uint32_t port;
        std::string ip;
        
        uint32_t persist;
        std::ofstream fp;
        std::string path;
};

#endif 