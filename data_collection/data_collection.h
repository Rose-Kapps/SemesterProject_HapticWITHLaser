#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <unistd.h>
#endif

// Mock functions to simulate data retrieval
//int getPositionX();
//int getPositionY();
//int getPositionZ();
//int getVoltage();

struct DataPoint {
    double x;
    double y;
    double z;
    double voltage;
};

extern std::vector<DataPoint> data;
// extern bool simulationRunning;

void collectData();
void saveDataToFile(const std::string& filename);

#endif // DATA_COLLECTION_H
