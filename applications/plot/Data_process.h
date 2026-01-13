#pragma once

#include <vector>
#include <deque>
#include "chai3d.h"
#include "cbw.h"
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <stdexcept>

using namespace chai3d;
using namespace std;

// Aliases Eigen
using Vec3 = Eigen::Vector3d;
using MatX = Eigen::MatrixXd;

#define signal_threshold   0.1 //V
#define max_signal 5 // V
#define microns 0.000001 // used to convert meter to microns 

// -----------------------------------------------------------------------------
// Step Smoother class (to reduce ADC quantization noise)
// -----------------------------------------------------------------------------
class StepSmoother {
public:
    StepSmoother(int tolerance = 1, int persistence = 3);
    double apply(double newValue);

private:
    int tolerance;
    int persistence;
    double lastValue;
    double stableValue;
    int counter;
    bool initialized;
};



struct signal3d
{
    cVector3d RobotPos;
    double signal_value;
    signal3d(cVector3d position, double value) {
        RobotPos = position;
        signal_value = value;
    }
};

class GaussianFilter {
public:
    /**
     * Constructor
     * @param window_size: Number of past measurements to consider
     * @param sigma: Standard deviation for Gaussian weights
     */
    GaussianFilter(int window_size, double sigma);

    /**
     * Apply the Gaussian filter to a new measurement
     * @param new_measurement: The new input value to be filtered
     * @return: The smoothed value
     */
    double applyFilter(double new_measurement);

private:
    std::vector<double> buffer;     // Circular buffer to store measurements
    std::vector<double> weights;    // Gaussian weights
    int window_size;                // Size of the filter window
    int index;                      // Current insert index
    bool filled;                    // Whether buffer has been fully filled at least once

    /**
     * Precomputes Gaussian weights based on sigma
     */
    void calculateWeights(double sigma);
};

class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 0.1);
    double apply(double x);

private:
    double alpha;
    bool initialized;
    double y;
};

double updateMax(cVector3d position,double voltage,cVector3d& maxPos, bool reset) ;

cVector3d computeGradient(cVector3d currentRobotPosition, double current_voltage);

cVector3d computeSignalDif(cVector3d currentRobotPosition, double current_voltage);

// ============================
// Plane fitting & geometry
// ============================

// Fit a plane using SVD
void fitPlaneSVD(const std::vector<Vec3>& points,
    Vec3& centroid,
    Vec3& normal,
    Vec3& singularValues);

// Signed distance point → plane
double signedDistanceToPlane(const Vec3& point,
    const Vec3& centroid,
    const Vec3& normal);

// Projection point → plane
Vec3 projectPointToPlane(const Vec3& point,
    const Vec3& centroid,
    const Vec3& normal);

// Plane basis
void planeBasis(const Vec3& normal,
    Vec3& u,
    Vec3& v);

// ============================
// Line geometry
// ============================

Vec3 projectPointToLine(const Vec3& P,
    const Vec3& A,
    const Vec3& B);

Vec3 projectVectorToLine(const Vec3& vector,
    const Vec3& A,
    const Vec3& B);

double distanceToLine(const Vec3& P,
    const Vec3& A,
    const Vec3& B);

Vec3 vectorToLine(const Vec3& P,
    const Vec3& A,
    const Vec3& B);

Vec3 perpendicularDirectionToLine(const Vec3& P,
    const Vec3& A,
    const Vec3& B);

double computeT(const Vec3& P,
    const Vec3& A,
    const Vec3& B);

// ============================
// Utility
// ============================

bool isFarEnough(const std::vector<Vec3>& points,
    const Vec3& newPoint,
    double threshold);