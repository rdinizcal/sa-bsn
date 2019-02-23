#ifndef GOALMODEL_TASK_HPP
#define GOALMODEL_TASK_HPP

#include <string>
#include <vector>

#include "goalmodel/Task.hpp"
#include "goalmodel/Property.hpp"

namespace bsn {
    namespace goalmodel {

        class Task {

            public:
                Task(const std::string &/*id*/, const std::string &/*description*/);

                Task();
                ~Task();

                Task(const Task &);
                Task &operator=(const Task &);
                bool operator==(const Task &rhs);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                std::vector<Task> getChildren() const;

                void addChild(const Task &/*task*/);
                void removeChild(const std::string &/*id*/);
                //Task* getChild(const std::string &/*id*/);

            private:
                int findChild(const std::string &/*id*/);

            protected:
                std::string id;
                std::string description;

            private:
                std::vector<Task> children;
        };
    }  
}

#endif