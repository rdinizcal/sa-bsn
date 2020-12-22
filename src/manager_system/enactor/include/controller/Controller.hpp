#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "enactor/Enactor.hpp"

class Controller : public Enactor {
    public:
        Controller(int &argc, char **argv, std::string name);
        virtual ~Controller();

    private:
      	Controller(const Controller &);
    	Controller &operator=(const Controller &);

    public: 
        virtual void setUp();

        virtual void apply_strategy(const std::string &component);      
};

#endif