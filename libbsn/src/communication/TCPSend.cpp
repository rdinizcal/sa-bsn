#include "communication/TCPSend.hpp"

using namespace std;
using namespace odcore;
using namespace odcore::io::tcp;

namespace bsn {
    namespace communication {
        TCPSend::TCPSend() : port(6060), ip("localhost"), is_connected(false), thisConnection(nullptr) {}

        TCPSend::TCPSend(int32_t thisPort) : port(thisPort), ip("localhost"), is_connected(false), thisConnection(nullptr) {}

        TCPSend::TCPSend(std::string thisIp, int32_t thisPort) : port(thisPort), ip(thisIp), is_connected(false), thisConnection(nullptr) {}

        TCPSend::TCPSend(const TCPSend &obj) : 
            port(obj.get_port()), 
            ip(obj.getIP()),
            is_connected(false),
            thisConnection(nullptr) {}

        TCPSend& TCPSend::operator=(const TCPSend &obj) {
            port = obj.get_port();
            ip = obj.getIP();
            is_connected = false;
            thisConnection = nullptr;
            return (*this);
        }

        bool TCPSend::connect(){
            try{                
                std::shared_ptr<TCPConnection> connection(TCPFactory::createTCPConnectionTo(ip, port));                
                connection->setRaw(true);        
                thisConnection = connection;                                
                cout << "Sender connected on " << get_port() << endl;
                is_connected = true;
                return true;
            }
            catch(string &exception) {
                cout << "Sender couldn't connect on " << get_port() << '(' << exception << ')' << endl;
                is_connected = false;
                return false;
            }   
        }

        void TCPSend::setIP(const string vIP) {
            this->ip = vIP;
        }   

        string TCPSend::getIP() const {
            return ip;
        }

        void TCPSend::set_port(const int32_t p) {
            port = p;
        }

        int32_t TCPSend::get_port() const {
            return port;
        }

        void TCPSend::disconnect() {
            // Desaloca o ponteiro pra conexão
            thisConnection.reset();
        }

        void TCPSend::send(string package) {    
            if(is_connected){
                try {
                    // Adiciona caracter separador
                    package += '*';
                    thisConnection->send(package);        
                }
                catch(string &exception) {
                    cerr << "Data could not be sent: " << exception << endl;
                }
            }
        }

        const std::string TCPSend::toString() const {
            stringstream sstr;

            sstr << "TCPSend " << ip << " " << port << endl;

            return sstr.str();
        }

    }
}

