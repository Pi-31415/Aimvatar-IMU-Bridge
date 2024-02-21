#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

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

int main(int argc, char *argv[]) {
    int sockfd;
    struct sockaddr_in serveraddr, clientaddr;
    char buffer[1024];
    bool verbose = false;

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

        if (verbose) {
            std::cout << "Received: " << buffer << std::endl;
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
