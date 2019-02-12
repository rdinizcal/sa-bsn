#include "bsn/operation/Operation.hpp"

using namespace std;

namespace bsn {
    namespace operation {

        Operation::Operation() {};
        Operation::~Operation() {};

        const vector<string> Operation::split(const string& s, const char& c) {    
	        string buff{""};
	        vector<string> v;
	
	        for(auto n:s) {
	    	    if(n != c) 
                    buff+=n; 
                else
		            if(n == c && buff != "") { 
                        v.push_back(buff); 
                        buff = ""; 
                    }
	        }
	        if(buff != "") 
                v.push_back(buff);
	
	        return v;
        }

    }
}

