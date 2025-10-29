
#include <windows.h>




// Struct to hold data points
struct DataPoint {
    int x;
    int y;
    int z;
    int voltage;
};

// Global variables
std::vector<DataPoint> data;

// Function to collect data
void collectData() {
    while (simulationRunning) {
        DataPoint point;
        point.x = getPositionX();
        point.y = getPositionY();
        point.z = getPositionZ();
        point.voltage = getVoltage();
        data.push_back(point);

        // Sleep for 100ms
        #if defined(_WIN32) || defined(_WIN64)
        Sleep(100);  // Sleep for 100 milliseconds on Windows
        #else
        usleep(100000);  // Sleep for 100,000 microseconds (100ms) on UNIX-like systems
        #endif
    }
}

// Function to save data to a file
void saveDataToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (const auto& point : data) {
        file << point.x << " " << point.y << " " << point.z << " " << point.voltage << std::endl;
    }

    file.close();
}

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
