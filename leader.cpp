
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <termios.h>
#include <yaml-cpp/yaml.h>

// for UDP socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


#define PORT 49185
#define BUFFER_SIZE 2048


std::mutex state_mutex;
std::atomic<bool> running{true};


void pubThread (const YAML::Node& config) {
     
    // socket params
    int sockPub = socket(AF_INET, SOCK_DGRAM, 0);
    char buffer[BUFFER_SIZE];
    struct sockaddr_in send_addr{};
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &send_addr.sin_addr);

    const int pub_freq = 100; //Hz

    // test message
    std::string msg = "Hello from server!";

    while (running.load()) {

        sendto(sockPub, msg.c_str(), msg.size(), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));

        std::cout << "Publishing..." << std::endl;

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/pub_freq));
    }

    close(sockPub);

}

void subThread() {

}


int main () {
    
    try {

        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");
        
        // start publisher thread
        std::thread pub_thread(pubThread, std::cref(config));

        while (running.load()) {

            std::cout << "Main............." << std::endl;
            //simulate a task
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // stop thread
        running.store(false);
        pub_thread.join();

    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;
    
}