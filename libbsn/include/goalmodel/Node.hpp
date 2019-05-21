#ifndef GOALMODEL_NODE_HPP
#define GOALMODEL_NODE_HPP

#include <string>
#include <vector>
#include <stdexcept> 
#include <memory>

namespace bsn {
    namespace goalmodel {

        class Node {

            public:
                Node(const std::string &/*id*/, const std::string &/*description*/);
                Node();
                virtual ~Node();

                Node(const Node &);
                Node &operator=(const Node &);
                bool operator==(const Node &rhs) const;
                friend std::ostream& operator<<(std::ostream &/*stream*/, const Node &/*node*/);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                bool hasChildren() const;
                std::vector<std::shared_ptr<Node>> getChildren() const;

                void addChild(std::shared_ptr<Node> /*goal*/);
                void removeChild(const std::string &/*id*/);
                std::shared_ptr<Node> getChild(const std::string &/*id*/);


            private:
                int findChild(const std::string &/*id*/);

            protected:
                std::string id;
                std::string description;
                std::vector<std::shared_ptr<Node>> children;
        };
    }  
}

#endif