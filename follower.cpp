
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

// for UDP socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 49186
#define BUFFER_SIZE 2048

std::mutex state_mutex;
std::atomic<bool> running{true};



void pubThread () {

}



void subThread (const YAML::Node& config) {
    //socket params
    int sockSub = socket(AF_INET, SOCK_DGRAM, 0);
    char buffer[BUFFER_SIZE];
    struct sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(49185); // port for leader publisher
    recv_addr.sin_addr.s_addr = INADDR_ANY; // listen on all addresses

    const int sub_freq = 100; //Hz

    if (bind(sockSub, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
        perror("Subscriber socket bind failed");
        close(sockSub);
    }

    std::cout << "Subscriber lsitening on port: 49185" << std::endl;

    while(running.load()) {

        ssize_t n = recvfrom(sockSub, buffer, sizeof(buffer), 0, nullptr, nullptr);

        buffer[n] = '\0';

        std::cout << "Received : " << buffer << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/sub_freq));

    }

    close(sockSub);

}



int main () {

    try {

        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");

        // start sub thread
        std::thread sub_thread(subThread, std::cref(config));

        while (running.load()) {

            std::this_thread::sleep_for(std::chrono::seconds(3));

        }

        // stop thread
        running.store(false);
        sub_thread.join();

    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;

}