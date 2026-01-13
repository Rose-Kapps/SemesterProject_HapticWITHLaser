#include "Data_process.h"

#include <numeric>


// -----------------------------------------------------------------------------
// StepSmoother implementation
// -----------------------------------------------------------------------------
StepSmoother::StepSmoother(int tolerance, int persistence)
    : tolerance(tolerance), persistence(persistence),
    lastValue(0.0), stableValue(0.0), counter(0), initialized(false) {}

double StepSmoother::apply(double newValue) {
    if (!initialized) {
        lastValue = stableValue = newValue;
        initialized = true;
        return stableValue;
    }

    if (fabs(newValue - stableValue) <= tolerance) {
        counter = 0; // No real change
    }
    else {
        if (fabs(newValue - stableValue) > tolerance) {
            counter++;
            if (counter >= persistence) {
                stableValue = newValue;
                counter = 0;
            }
        }
    }

    lastValue = newValue;
    return stableValue;
}

GaussianFilter::GaussianFilter(int window_size, double sigma)
    : window_size(window_size), index(0), filled(false) {
    buffer.resize(window_size, 0.0);
    weights.resize(window_size);
    calculateWeights(sigma);
}

void GaussianFilter::calculateWeights(double sigma) {
    int mid = window_size / 2;
    double sum = 0.0;

    for (int i = 0; i < window_size; ++i) {
        int x = i - mid;
        weights[i] = std::exp(-0.5 * (x * x) / (sigma * sigma));
        sum += weights[i];
    }

    // Normalize weights
    for (int i = 0; i < window_size; ++i) {
        weights[i] /= sum;
    }
}

double GaussianFilter::applyFilter(double new_measurement) {
    buffer[index] = new_measurement;
    index = (index + 1) % window_size;

    int count = filled ? window_size : index;
    double filtered_value = 0.0;

    for (int i = 0; i < count; ++i) {
        int weight_index = (i - index + window_size) % window_size;
        filtered_value += buffer[i] * weights[weight_index];
    }

    if (!filled && index == 0) filled = true;

    return filtered_value;
}


double updateMax(cVector3d position, double voltage,cVector3d& maxPos,bool reset) {
    static std::deque<cVector3d> lastPositions;
    static std::deque<double> lastIntensities;

    lastPositions.push_back(position);
    lastIntensities.push_back(voltage);

    auto it = std::max_element(lastIntensities.begin(), lastIntensities.end());
    double maxVoltage = *it;  // Get the value
    double maxIndex = std::distance(lastIntensities.begin(), it); // Get the index
    maxPos = lastPositions[maxIndex];
    cVector3d temporary_max = position;
    if (reset == true) {
        lastIntensities.clear();
        lastPositions.clear();
        maxPos = temporary_max;
        maxVoltage = 0;
        return 0;
    }
    
    return maxVoltage;
}



cVector3d computeGradient(cVector3d currentRobotPosition,double current_voltage ) {
    static std::deque<signal3d> last_values;
    signal3d tmp(currentRobotPosition, current_voltage);
    
    // Check if currentRobotPosition is already in last_values
    bool isValid = true;
    double delta = 10*microns; //m
    double voltagNoiseThreshold = 0.1; //V
    for (const auto& entry : last_values) {
        if (entry.RobotPos.equals(currentRobotPosition) || entry.RobotPos.distance(currentRobotPosition) < delta)
        {
            isValid = false;
            break;
        }
    }
    
    if (last_values.size() < 3) 
    {
        if (isValid)   last_values.push_back(tmp);
        return cVector3d(0, 0, 0);
    }
    else 
    {
        // Sample voltages at the three points
        double V_p0 = current_voltage;
        double V_p1 = last_values[0].signal_value;
        double V_p2 = last_values[1].signal_value;
        double V_p3 = last_values[2].signal_value;

        // Set up the matrix A with the displacement vectors
        Eigen::Matrix3d A;
        A.row(0) = Eigen::Vector3d(last_values[0].RobotPos.x() - currentRobotPosition.x(),
            last_values[0].RobotPos.y() - currentRobotPosition.y(),
            last_values[0].RobotPos.z() - currentRobotPosition.z()).transpose();

        A.row(1) = Eigen::Vector3d(last_values[1].RobotPos.x() - currentRobotPosition.x(),
            last_values[1].RobotPos.y() - currentRobotPosition.y(),
            last_values[1].RobotPos.z() - currentRobotPosition.z()).transpose();

        A.row(2) = Eigen::Vector3d(last_values[2].RobotPos.x() - currentRobotPosition.x(),
            last_values[2].RobotPos.y() - currentRobotPosition.y(),
            last_values[2].RobotPos.z() - currentRobotPosition.z()).transpose();

        // Set up the vector b with the voltage differences
        Eigen::Vector3d b;
        b << V_p1 - V_p0, V_p2 - V_p0, V_p3 - V_p0;

        // Solve the system of equations A * grad = b for the gradient (grad)
        Eigen::Vector3d grad = A.colPivHouseholderQr().solve(b);

        // Convert the gradient to a CHAI3D vector (cVector3d)
        cVector3d gradient(grad[0] / 100, grad[1] / 100, grad[2] / 100);

        // Add current position and voltage and remove the last one
        if (isValid) {
            last_values.push_back(tmp);
            last_values.pop_front();
        }

        return gradient;
    
    }

}

cVector3d computeSignalDif(cVector3d currentRobotPosition, double current_voltage) {
    static signal3d prev_value(currentRobotPosition , current_voltage);

    //If displacement is too small, ignore it 
    if ((currentRobotPosition - prev_value.RobotPos).length() < 10 * microns) {
        return cVector3d(0, 0, 0);
    }

    //Do not compute spring force if signal is bellow threshold
    if (current_voltage < signal_threshold) {
        prev_value.signal_value = 0;
        return cVector3d(0, 0, 0);
    }

    double signal_diff;
    cVector3d position_diff;
    
    //Define position difference vector such that its   
    //magnitude is unit and all components are positive
    position_diff = cVector3d(  abs(currentRobotPosition.x() - prev_value.RobotPos.x()),
                                abs(currentRobotPosition.y() - prev_value.RobotPos.y()),
                                abs(currentRobotPosition.z() - prev_value.RobotPos.z()));
    
    if(position_diff.length()!=0) position_diff.normalize();


    //If signal at current position is smaller then previous -> move the user back to where they were
    if (current_voltage < 0.5*prev_value.signal_value) {
        signal_diff = (current_voltage - prev_value.signal_value);
    }
    else {
        signal_diff = 0;
    }

    //Multiply the displacement vector by the signal delta
    position_diff.normalize();
    position_diff *= cClamp((exp(signal_diff)), 0.0, 5.0);


    //Replace previous values with actual values
    prev_value.RobotPos = currentRobotPosition;
    prev_value.signal_value = current_voltage;

    //Set vector components to 0 if they are too small
    for (int i = 0; i<2 ; i++)
    {
        if (position_diff(i) < 0.2) {
            position_diff(i) = 0;
        }
    }
    return position_diff;
}

// ============================
// LowPassFilter
// ============================

LowPassFilter::LowPassFilter(double alpha)
    : alpha(alpha), initialized(false), y(0.0) {
}

double LowPassFilter::apply(double x) {
    if (!initialized) {
        y = x;
        initialized = true;
    }
    else {
        y = alpha * x + (1.0 - alpha) * y;
    }
    return y;
}


// ============================
// Plane fitting
// ============================

void fitPlaneSVD(const std::vector<Vec3>& points,
    Vec3& centroid,
    Vec3& normal,
    Vec3& singularValues)
{
    if (points.size() < 3) {
        throw std::runtime_error("at least 3 points needed");
    }

    centroid.setZero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    MatX X(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        Vec3 v = points[i] - centroid;
        X.row(i) = v.transpose();
    }

    Eigen::JacobiSVD<MatX> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    singularValues = svd.singularValues();

    Eigen::Matrix3d V = svd.matrixV();
    normal = V.col(2);
    normal.normalize();
}

double signedDistanceToPlane(const Vec3& point,
    const Vec3& centroid,
    const Vec3& normal)
{
    double d = normal.dot(point - centroid);
    return d;
}

Vec3 projectPointToPlane(const Vec3& point,
    const Vec3& centroid,
    const Vec3& normal)
{
    double d = signedDistanceToPlane(point, centroid, normal);
    Vec3 projectedPoint = point - d * normal;
    return projectedPoint;
}

void planeBasis(const Vec3& normal,
    Vec3& u,
    Vec3& v)
{
    Vec3 n = normal.normalized();
    Vec3 tmp;

    if (std::abs(n.x()) < 0.9) {
        tmp = Vec3(1, 0, 0);
    }
    else {
        tmp = Vec3(0, 1, 0);
    }

    u = n.cross(tmp);
    u.normalize();

    v = n.cross(u);
    v.normalize();
}

// ============================
// Utility
// ============================

bool isFarEnough(const std::vector<Vec3>& points,
    const Vec3& newPoint,
    double threshold)
{
    for (const auto& p : points) {
        double dist = (p - newPoint).norm();
        if (dist < threshold) {
            return false;
        }
    }
    return true;
}