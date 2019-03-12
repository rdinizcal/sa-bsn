#ifndef GOALMODEL_NODE_HPP
#define GOALMODEL_NODE_HPP

#include <string>
#include <vector>
#include <stdexcept> 

namespace bsn {
    namespace goalmodel {

        class Node {

            public:
                Node(const std::string &/*id*/, const std::string &/*description*/);
                Node();
                ~Node();

                Node(const Node &);
                Node &operator=(const Node &);
                bool operator==(const Node &rhs);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                bool hasChildren() const;
                std::vector<Node> getChildren() const;

                void addChild(const Node &/*goal*/);
                void removeChild(const std::string &/*id*/);
                Node getChild(const std::string &/*id*/);

            private:
                int findChild(const std::string &/*id*/);

            protected:
                std::string id;
                std::string description;
                std::vector<Node> children;
        };
    }  
}

#endif