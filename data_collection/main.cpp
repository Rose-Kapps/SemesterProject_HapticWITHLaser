#include "data_collection.h"
#include <thread>

int main() {
    // Start data collection in a separate thread
    std::thread dataThread(collectData);

    // Simulate running for 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));
    simulationRunning = false;

    // Wait for the data collection thread to finish
    dataThread.join();

    // Save the collected data to a file
    saveDataToFile("collected_data.txt");

    return 0;
}
