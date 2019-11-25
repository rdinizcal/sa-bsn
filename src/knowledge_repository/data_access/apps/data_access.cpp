#include "data_access/DataAccess.hpp"

int32_t main(int argc, char **argv) {
    DataAccess access(argc, argv, "data_access");
    return access.run();
}
