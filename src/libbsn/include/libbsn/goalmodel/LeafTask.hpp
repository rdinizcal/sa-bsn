#ifndef GOALMODEL_LEAFTASK_HPP
#define GOALMODEL_LEAFTASK_HPP

#include <string>
#include <vector>

#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {

        class LeafTask : public Task {

            public:
                LeafTask(const std::string &/*id*/, const std::string &/*description*/, const goalmodel::Context &/*context*/, const goalmodel::Property &/*cost*/, const goalmodel::Property &/*reliability*/, const goalmodel::Property &/*frequency*/);
                LeafTask(const std::string &/*id*/, const std::string &/*description*/, const goalmodel::Property &/*cost*/, const goalmodel::Property &/*reliability*/, const goalmodel::Property &/*frequency*/);

                LeafTask();
                ~LeafTask();

                LeafTask(const LeafTask &);
                LeafTask &operator=(const LeafTask &);

                void setContext(const goalmodel::Context &/*cost*/);
                goalmodel::Context getContext() const;

                void setCost(const goalmodel::Property &/*cost*/);
                goalmodel::Property getCost() const;

                void setReliability(const goalmodel::Property &/*reliability*/);
                goalmodel::Property getReliability() const;

                void setFrequency(const goalmodel::Property &/*frequency*/);
                goalmodel::Property getFrequency() const;

                void addChild(std::shared_ptr<Node> /*node*/);

            private:
                goalmodel::Context context;
                goalmodel::Property cost;
                goalmodel::Property reliability;
                goalmodel::Property frequency;
        };
    }  
}

#endif