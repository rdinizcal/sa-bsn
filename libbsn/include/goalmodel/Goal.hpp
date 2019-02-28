#ifndef GOALMODEL_GOAL_HPP
#define GOALMODEL_GOAL_HPP

#include <string>
#include <vector>
#include <stdexcept> 

namespace bsn {
    namespace goalmodel {

        class Goal {

            public:
                Goal(const std::string &/*id*/, const std::string &/*description*/);

                Goal();
                ~Goal();

                Goal(const Goal &);
                Goal &operator=(const Goal &);
                bool operator==(const Goal &rhs);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                std::vector<Goal> getChildren() const;

                void addChild(const Goal &/*goal*/);
                void removeChild(const std::string &/*id*/);
                Goal getChild(const std::string &/*id*/);

            private:
                int findChild(const std::string &/*id*/);

            protected:
                std::string id;
                std::string description;
                std::vector<Goal> children;
        };
    }  
}

#endif