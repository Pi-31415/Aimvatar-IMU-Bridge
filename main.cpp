#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>


// Data Structure to hold sensor data
struct SensorData {
    float orientation[3];
    float angularVelocity[3];
    float linearAcceleration[3];
    float magnetometer[3];
    float accelerometer[3];
    float gravity[3];
    int8_t temperature;
    uint8_t calibration[4];
};

void parseData(const std::string& dataString, SensorData& sensorData);
void printSensorData(const SensorData& data);
void printSensorDataRow(const std::string& sensorName, const float data[3], int nameWidth, int valueWidth);
// Declare a clearscreen function
void clearScreen() {
    std::cout << "\033[2J\033[1;1H";
}
std::string getLocalIP();

int main(int argc, char *argv[]) {
    int sockfd;
    struct sockaddr_in serveraddr, clientaddr;
    char buffer[1024];
    bool verbose = false;
    SensorData sensorData;
    // Check if verbose mode is enabled
    if (argc > 1 && std::string(argv[1]) == "--verbose") {
        verbose = true;
    }

    // Create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error opening socket" << std::endl;
        return 1;
    }

    // Get local IP address
    std::string localIP = getLocalIP();

    // Configure server address
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(12345); // Arbitrary port
    serveraddr.sin_addr.s_addr = inet_addr(localIP.c_str());

    // Bind socket
    if (bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        return 1;
    }

    if (verbose) {
        std::cout << "Server started at " << localIP << ":" << ntohs(serveraddr.sin_port) << std::endl;
    }

    // Main loop
    while (true) {
        socklen_t len = sizeof(clientaddr);
        int n = recvfrom(sockfd, buffer, 1024, 0, (struct sockaddr *)&clientaddr, &len);

        if (n < 0) {
            std::cerr << "Error receiving data" << std::endl;
            continue;
        }

        buffer[n] = '\0';
        parseData(std::string(buffer), sensorData);
        if (verbose) {
            printSensorData(sensorData);
        }

        // Respond to the client
        const char *msg = "Message received";
        sendto(sockfd, msg, strlen(msg), 0, (struct sockaddr *)&clientaddr, len);

        if (std::string(buffer) == "exit") {
            break;
        }
    }

    // Close socket
    close(sockfd);
    return 0;
}



// Function to automatically get the local IP address
std::string getLocalIP() {
    char hostbuffer[256];
    char *IPbuffer;
    struct hostent *host_entry;
    int hostname;

    // Retrieve the hostname
    hostname = gethostname(hostbuffer, sizeof(hostbuffer));
    host_entry = gethostbyname(hostbuffer);
    IPbuffer = inet_ntoa(*((struct in_addr*) host_entry->h_addr_list[0]));

    return std::string(IPbuffer);
}


void parseData(const std::string& dataString, SensorData& sensorData) {
    std::istringstream iss(dataString);
    std::string token;
    std::vector<float> values;

    while (std::getline(iss, token, ',')) {
        values.push_back(std::stof(token));
    }

    if (values.size() >= 22) {
        std::copy(values.begin(), values.begin() + 3, sensorData.orientation);
        std::copy(values.begin() + 3, values.begin() + 6, sensorData.angularVelocity);
        std::copy(values.begin() + 6, values.begin() + 9, sensorData.linearAcceleration);
        std::copy(values.begin() + 9, values.begin() + 12, sensorData.magnetometer);
        std::copy(values.begin() + 12, values.begin() + 15, sensorData.accelerometer);
        std::copy(values.begin() + 15, values.begin() + 18, sensorData.gravity);
        sensorData.temperature = static_cast<int8_t>(values[18]);
        sensorData.calibration[0] = static_cast<uint8_t>(values[19]);
        sensorData.calibration[1] = static_cast<uint8_t>(values[20]);
        sensorData.calibration[2] = static_cast<uint8_t>(values[21]);
        sensorData.calibration[3] = static_cast<uint8_t>(values[22]);
    }
}

void printSensorData(const SensorData& data) {
    // Set the width for each column
    int nameWidth = 20;
    int valueWidth = 12;

    // Clear the screen
    clearScreen();
    // Print table headers
    std::cout << std::left
              << std::setw(nameWidth) << "Sensor"
              << std::setw(valueWidth) << "X"
              << std::setw(valueWidth) << "Y"
              << std::setw(valueWidth) << "Z"
              << "\n";

    // Print a line separator
    std::cout << std::string(nameWidth + 3 * valueWidth, '-') << "\n";

    // Print sensor data
    printSensorDataRow("Orientation", data.orientation, nameWidth, valueWidth);
    printSensorDataRow("Angular Velocity", data.angularVelocity, nameWidth, valueWidth);
    printSensorDataRow("Linear Accel", data.linearAcceleration, nameWidth, valueWidth);
    printSensorDataRow("Magnetometer", data.magnetometer, nameWidth, valueWidth);
    printSensorDataRow("Accelerometer", data.accelerometer, nameWidth, valueWidth);
    printSensorDataRow("Gravity", data.gravity, nameWidth, valueWidth);
    
    // Print temperature and calibration
    std::cout << std::setw(nameWidth) << "Temperature"
              << std::setw(valueWidth) << static_cast<int>(data.temperature) << "\n";
    std::cout << std::setw(nameWidth) << "Calibration"
              << std::setw(valueWidth) << "Sys=" << static_cast<int>(data.calibration[0])
              << " Gyro=" << static_cast<int>(data.calibration[1])
              << " Accel=" << static_cast<int>(data.calibration[2])
              << " Mag=" << static_cast<int>(data.calibration[3]) << "\n";
}

void printSensorDataRow(const std::string& sensorName, const float data[3], int nameWidth, int valueWidth) {
    std::cout << std::setw(nameWidth) << sensorName
              << std::setw(valueWidth) << data[0]
              << std::setw(valueWidth) << data[1]
              << std::setw(valueWidth) << data[2]
              << "\n";
}
