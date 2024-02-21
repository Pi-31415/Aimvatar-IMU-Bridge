#include "ximu_cpp_library/DataBaseClass.h"

int main() {
    try {
        DataBaseClass dataBase("LoggedData_CalBattAndTherm.csv");
        dataBase.ImportCSVnumeric("file_prefix_");
        std::cout << "Number of packets: " << dataBase.GetNumPackets() << std::endl;
        // Additional usage
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
