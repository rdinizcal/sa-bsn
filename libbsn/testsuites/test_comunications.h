#include <cxxtest/TestSuite.h>
#include "bsn/communication/TCPReceive.hpp"
#include "bsn/communication/TCPSend.hpp"

using namespace std;
using namespace bsn::communication;

class test_comunications : public CxxTest::TestSuite {
public:    
    void test_initialization_receiver(void) {
        cout << "\n\nTesting receiver constructor...\n";
        TCPReceive server(8080);
        TS_ASSERT_EQUALS(8080,server.get_port());        
        server.stop_connection();
    }

    void test_initialization_sender(void) {
        cout << "\n\nTesting sender constructor...\n";
        TCPSend sender("localhost",8080);
        TS_ASSERT_EQUALS(8080,sender.get_port());        
        sender.disconnect();
    }

    // void test_false_connection(void) {
    //     cout << "\n\nTesting sender connection false condition...\n";
    //     TCPSend sender("localhost",8080);
    //     // Como ninguem está escutando deve retornar false
    //     TS_ASSERT_EQUALS(false,sender.connect());
    //     sender.disconnect();
    // }

    void test_simple_send(void) {        
        cout << "\n\nTesting Simple message sending...\n";                        
        TCPSend sender("localhost",1234);      
        TCPReceive server(1234);  
        
        server.start_connection();    
        
        sender.connect();
        
        sender.send("Test message");
        //Wait for package
        sleep(1);
        string pack = server.get_package();
        
        sender.disconnect();
        server.stop_connection();
        
        TS_ASSERT_EQUALS(pack,"Test message");
        
    }
};
