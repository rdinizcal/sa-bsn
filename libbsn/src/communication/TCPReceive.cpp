#include "bsn/communication/TCPReceive.hpp"

using namespace std;

using namespace odcore;
using namespace odcore::wrapper;
using namespace odcore::io;
using namespace odcore::io::tcp;
using namespace bsn::operation;

namespace bsn {
    namespace communication {

        deque<string> buffer;
        
        TCPReceive::TCPReceive(int32_t p) :
            port(p),
            bufferLock(),
            shouldRun(false),
            thisConnection(nullptr) {}

        TCPReceive::TCPReceive(const TCPReceive &obj) : 
            port(obj.get_port()),
            bufferLock(),
            shouldRun(false),
            thisConnection(nullptr) {}

        TCPReceive& TCPReceive::operator=(const TCPReceive &obj) {
            port = obj.get_port();        
            return (*this);
        }

        void TCPReceive::set_port(const int32_t p) {
            port = p;
        }

        int32_t TCPReceive::get_port() const {
            return port;
        }

        void TCPReceive::start_connection(){
            thread listener(&TCPReceive::initialize,this);
            listener.detach();
        }

        void TCPReceive::handleConnectionError() {

            thisConnection->stop();
            cout << "Connection terminated\n";
        }

        string TCPReceive::get_package() {
            string p;
            bufferLock.lock();
                if(buffer.size() > 0) {
                    p = buffer.front();
                    buffer.pop_front();
                }
                else {
                    p = "";
                }
                
            bufferLock.unlock();
            return p;
        }

        void TCPReceive::print_buffer() {
            bufferLock.lock();
            cout << "\nBuffer " << buffer.size() << ':';
            for(auto s : buffer)
                cout << s << ',';
            bufferLock.unlock();
        }

        void TCPReceive::nextString(const std::string &receivedString) {
            // Realiza split pelo caracter separador escolhido. No caso '*'
            Operation op;
            vector<string> vet = op.split(receivedString,'*');
            bufferLock.lock();
                for (auto str : vet) {
                    buffer.push_back(str);
                }        
            bufferLock.unlock();

        }

        void TCPReceive::onNewConnection(std::shared_ptr<odcore::io::tcp::TCPConnection> connection) {
            if (connection.get()) {
                cout << "Handle a new connection." << endl;
            
                connection->setRaw(true);	
                connection->setStringListener(this);
                connection->setConnectionListener(this);
                connection->start();        
                
                thisConnection = connection;        
            }
        }

        void TCPReceive::stop_connection(){
            shouldRun = false;
        }

        void TCPReceive::initialize(){
            cout << "Server listening on port " << get_port() << endl;    
            shouldRun = true;
            try {
                std::shared_ptr<TCPAcceptor>
                    tcpacceptor(TCPFactory::createTCPAcceptor(port));
                TCPReceive handler(-1);
                tcpacceptor->setAcceptorListener(&handler);
                tcpacceptor->start();
                        
                while(shouldRun);        
                tcpacceptor.reset();
                cout << "Stopped the connection\n";
            }
            catch(string &exception) {
                cerr << "Error while creating TCP receiver: " << exception << endl;
            }
        }

        const string TCPReceive::toString() const {
            stringstream sstr;

            sstr << "TCPReceive" << endl;

            return sstr.str();
        }
    }
}
