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
#include "lib/MahonyAHRS.h"
#include <Eigen/Dense>
#include <memory>
#include "filter_common.h"
#include "filter_includes.h"
#include <cmath>
// Probably need to sample for a longer
 
using Eigen::Vector3f;
using Eigen::Matrix3f;

// Define the number of samples to accumulate before integration
const int integrationInterval = 5; // For example, 10 samples

// Variables for accumulating sensor data
std::vector<Vector3f> accBuffer;
int sampleCounter = 0;

// Add global variables for the high-pass filter coefficients
const int hpFilterOrder = 1;
const double hpFilterCutoff = 10;
// Sample frequency
const double sampleFreq = 100.0;
// 100Hz - The BNO055 advertises a 100Hz accelerometer output data rate, so we'll use that as our sample rate
const double samplePeriod = 1.0 / sampleFreq;


class HighPassFilter {
public:
    HighPassFilter(double cutoffFreq, double sampleRate) {
        // Calculate filter coefficients
        double RC = 1.0 / (cutoffFreq * 2 * M_PI);
        double dt = 1.0 / sampleRate;
        alpha = dt / (RC + dt);

        prevInput = Vector3f::Zero();
        prevOutput = Vector3f::Zero();
    }

    Vector3f apply(const Vector3f& input) {
        Vector3f output = alpha * (prevOutput + input - prevInput);
        prevInput = input;
        prevOutput = output;
        return output;
    }

private:
    double alpha;
    Vector3f prevInput;
    Vector3f prevOutput;
};

// Declare a global filter instance
HighPassFilter hpFilter(hpFilterCutoff, sampleFreq); // Example: 0.1 Hz cutoff frequency, 256 Hz sample rate

// Function to apply high-pass filter
Vector3f applyHighPassFilter(const Vector3f& accel) {
    return hpFilter.apply(accel);
}

// Implemenation of filtfilt (Zero-phase digital filtering) function
class FilterImpl {
public:
    FilterImpl(const std::vector<double>& bCoeff, const std::vector<double>& aCoeff)
        : b(bCoeff), a(aCoeff) {
        if (a.empty()) a.push_back(1.0); // Ensure a[0] is present
    }

    std::vector<Vector3f> filter(const std::vector<Vector3f>& input) {
        std::vector<Vector3f> output(input.size(), Vector3f::Zero());
        for (size_t i = 0; i < input.size(); ++i) {
            for (size_t j = 0; j < b.size() && j <= i; ++j) {
                output[i] += b[j] * input[i - j];
            }
            for (size_t j = 1; j < a.size() && j <= i; ++j) {
                output[i] -= a[j] * output[i - j];
            }
            output[i] /= a[0];
        }
        return output;
    }

private:
    std::vector<double> b, a;
};

// Define a function to convert quaternion to rotation matrix (since Eigen's quaternion does not directly apply to our Mahony filter's output)
Matrix3f quaternionToRotationMatrix(float q0, float q1, float q2, float q3) {
    Matrix3f R;
    R << 1 - 2*q2*q2 - 2*q3*q3,     2*q1*q2 - 2*q0*q3,     2*q1*q3 + 2*q0*q2,
         2*q1*q2 + 2*q0*q3,     1 - 2*q1*q1 - 2*q3*q3,     2*q2*q3 - 2*q0*q1,
         2*q1*q3 - 2*q0*q2,     2*q2*q3 + 2*q0*q1,     1 - 2*q1*q1 - 2*q2*q2;
    return R;
}

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
    float roll, pitch, yaw;
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
    
    // How to use butterworth
    // std::unique_ptr<SO_BUTTERWORTH_HPF> filterHighPass (new SO_BUTTERWORTH_HPF);

    // auto coeffs = filterHighPass->calculate_coeffs(1.0, 5000);
    // auto yn = filterHighPass->process(0.303);

    // std::cout << "Coeffs: " << std::endl;
    // std::cout << "a0: " << coeffs.a0 << std::endl;
    // std::cout << "a1: " << coeffs.a1 << std::endl;
    // std::cout << "a2: " << coeffs.a2 << std::endl;
    // std::cout << "b1: " << coeffs.b1 << std::endl;
    // std::cout << "b2: " << coeffs.b2 << std::endl;
    // std::cout << "yn: " << yn << std::endl;

    int sockfd;
    struct sockaddr_in serveraddr, clientaddr;
    char buffer[1024];
    bool verbose = false;
    SensorData sensorData;
    Mahony filter;
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

    // Variables for position calculation
    Vector3f velocity = Vector3f::Zero(); // Initialize velocity
    Vector3f position = Vector3f::Zero(); // Initialize position

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

        // Apply the filter
         // Update Mahony filter with received sensor data
        filter.updateIMU(sensorData.angularVelocity[0], sensorData.angularVelocity[1], sensorData.angularVelocity[2],
                      sensorData.accelerometer[0], sensorData.accelerometer[1], sensorData.accelerometer[2]);

        // Extract roll, pitch, yaw from the filter
        sensorData.roll = filter.getRoll();
        sensorData.pitch = filter.getPitch();
        sensorData.yaw = filter.getYaw();

        if (verbose) {
            printSensorData(sensorData);
        }

        // ----------------- Position calculation -----------------
          // Accumulate sensor data
    Vector3f accel(sensorData.linearAcceleration[0],
                   sensorData.linearAcceleration[1],
                   sensorData.linearAcceleration[2]);
    accBuffer.push_back(accel);

    // Increase sample counter
    sampleCounter++;

    // Perform integration after every 'integrationInterval' samples
    if (sampleCounter >= integrationInterval) {
        Vector3f accumulatedAccel = Vector3f::Zero();
        for (const auto& acc : accBuffer) {
            accumulatedAccel += acc;
        }
        accumulatedAccel /= integrationInterval; // Average the accumulated acceleration

        // Apply high-pass filter
        Vector3f filteredAccel = applyHighPassFilter(accumulatedAccel);

        // Integrate acceleration to get velocity
        velocity += filteredAccel * samplePeriod * integrationInterval;

        // Integrate velocity to get position
        position += velocity * samplePeriod * integrationInterval;

        // Reset buffer and counter
        accBuffer.clear();
        sampleCounter = 0;
    }

        // Output the calculated position
        // std::cout << "Position: X=" << position[0] 
        //           << ", Y=" << position[1] 
        //           << ", Z=" << position[2] << std::endl;
        // Output position in cm
        std::cout << "Position (cm): X=" << position[0] * 100
                  << ", Y=" << position[1] * 100
                  << ", Z=" << position[2] * 100 << std::endl;

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
    // Print filtered orientation data
    std::cout << std::setw(nameWidth) << "Roll"
              << std::setw(valueWidth) << data.roll
              << "\n";
    std::cout << std::setw(nameWidth) << "Pitch"
              << std::setw(valueWidth) << data.pitch
              << "\n";
    std::cout << std::setw(nameWidth) << "Yaw"
              << std::setw(valueWidth) << data.yaw
              << "\n";
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
