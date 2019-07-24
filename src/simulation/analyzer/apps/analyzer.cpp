#include "Analyzer.hpp"

int main(int argc, char **argv) {
    
    Analyzer analyzer(argc, argv, "analyzer");

    ros::init(argc, argv, "analyzer");

    analyzer.setUp();
    analyzer.run();

    return 0;
}