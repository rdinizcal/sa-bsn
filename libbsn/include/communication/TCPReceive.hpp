#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <deque>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/io/tcp/TCPAcceptor.h>
#include <opendavinci/odcore/io/tcp/TCPFactory.h>
#include <opendavinci/odcore/io/ConnectionListener.h>
#include <opendavinci/odcore/io/StringListener.h>
#include <opendavinci/odcore/io/tcp/TCPAcceptorListener.h>
#include <opendavinci/odcore/io/tcp/TCPConnection.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include "bsn/operation/Operation.hpp"

namespace bsn {
    namespace communication {

        class TCPReceive: 
            public odcore::io::ConnectionListener,
            public odcore::io::StringListener,
            public odcore::io::tcp::TCPAcceptorListener {
            
            public:
              TCPReceive();
              TCPReceive(int32_t p);
              TCPReceive(const TCPReceive &);
              TCPReceive &operator=(const TCPReceive & /*obj*/);

            private:
                int32_t port;
                std::mutex bufferLock; 
                std::atomic_bool shouldRun;
                std::shared_ptr<odcore::io::tcp::TCPConnection> thisConnection;
                // std::shared_ptr<odcore::io::tcp::TCPAcceptor> tcpacceptor;

                virtual void nextString(const std::string &s);

                void push(std::string);

                virtual void onNewConnection(std::shared_ptr<odcore::io::tcp::TCPConnection> connection);

                virtual void handleConnectionError();

            public:
                void initialize();

                void set_port(const int32_t p);
                int32_t get_port() const;

                std::string get_package();

                void start_connection();
                void stop_connection();

                void print_buffer();

                const std::string toString() const;
            
        };

    }
}

