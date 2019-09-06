#include "data_access_node/DataAccessNode.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "repository");
    DataAccessNode dan(argc, argv);

    dan.setUp();
    dan.run();

    return 0;
}
