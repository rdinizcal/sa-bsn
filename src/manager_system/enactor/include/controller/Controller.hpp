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

        virtual void apply_reli_strategy(const std::string &component);
        virtual void apply_cost_strategy(const std::string &component);
        virtual void receiveEvent(const archlib::Event::ConstPtr& msg);

    private:
        std::string adaptation_parameter;
        double KP;
        std::map<std::string, double> kp;

};

#endif