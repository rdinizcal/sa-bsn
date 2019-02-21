#ifndef GOALMODEL_TASK_HPP
#define GOALMODEL_TASK_HPP

#include <string>

#include "goalmodel/Context.hpp"
#include "goalmodel/Property.hpp"

namespace bsn {
    namespace goalmodel {

        class Task {

            public:
                Task(const std::string &/*id*/, const std::string &/*description*/, const goalmodel::Context &/*context*/, const goalmodel::Property &/*cost*/, const goalmodel::Property &/*reliability*/, const goalmodel::Property &/*frequency*/);

                Task();
                ~Task();

                Task(const Task &);
                Task &operator=(const Task &);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;
                
                void setContext(const goalmodel::Context &/*cost*/);
                goalmodel::Context getContext() const;

                void setCost(const goalmodel::Property &/*cost*/);
                goalmodel::Property getCost() const;

                void setReliability(const goalmodel::Property &/*reliability*/);
                goalmodel::Property getReliability() const;

                void setFrequency(const goalmodel::Property &/*frequency*/);
                goalmodel::Property getFrequency() const;

            private:
                std::string id;
                std::string description;
                goalmodel::Context context;
                goalmodel::Property cost;
                goalmodel::Property reliability;
                goalmodel::Property frequency;
        };
    }  
}

#endif