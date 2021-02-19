#include "libbsn/utils/utils.hpp"

using namespace std;

namespace bsn {
    namespace utils {
        
        const vector<string> split(const string& s, const char& c) {    
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

