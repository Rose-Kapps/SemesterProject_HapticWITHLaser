#include "data_collection.h"

std::vector<DataPoint> data;
//bool simulationRunning = true;



void collectData() {
    while (simulationRunning) {
        DataPoint point;
        point.x = robotPosDes.x();
        point.y = robotPosDes.y();
        point.z = robotPosDes.z();
        point.voltage = voltageLevel;
        data.push_back(point);

        // Sleep for 100ms
       // #if defined(_WIN32) || defined(_WIN64)
        //Sleep(100);  // Sleep for 100 milliseconds on Windows
        //#else
        //usleep(100000);  // Sleep for 100,000 microseconds (100ms) on UNIX-like systems
        //#endif
    }
}

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
