#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/io/tcp/TCPConnection.h>
#include <opendavinci/odcore/io/tcp/TCPFactory.h>

namespace bsn {
    namespace communication {

        class TCPSend {
            public:
                TCPSend();

                TCPSend(int);

                // Construtor recebe uma string com o ip e uma int32_t porta        
                TCPSend(std::string, int32_t);

                TCPSend(const TCPSend & /*obj*/);
                TCPSend &operator=(const TCPSend & /*obj*/);

            private:

                int32_t port;
                std::string ip;
                bool is_connected;


                std::shared_ptr<odcore::io::tcp::TCPConnection> thisConnection;
            
            public:
                void disconnect();

                // Conecta à porta desejada e 
                // retorna true se foi feita com sucesso e false caso nao
                bool connect(); 

                // Envia um pacote por rede
                void send(std::string package);

                // Retorna a porta à qual tenta-se conectar        
                int32_t get_port() const;
                void set_port(const int32_t p);

                std::string getIP() const;
                void setIP(const std::string vIP);

                const std::string toString() const;
        };

    }
}