#ifndef _HTCVIVETRACKER_HH
#define _HTCVIVETRACKER_HH

#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <iomanip>
#include <vector>
#include <map>
#include <openvr.h>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>
#include <igl/bbw.h>


// Reference Code: https://github.com/laiaaa0/htc_vive_tracker

using namespace std;
using namespace Eigen;

enum PRINT {
    AllDevices,
    AllTrackers, 
    RefTracker,
    SubTrackers,
    BaseStations
};

enum FITTING {
    CIRCLE_X,
    CIRCLE_Y,
    CIRCLE_Z,
    SPHERE,
    SPHERE_1135Z
};

enum COORD
{
    Absolute,
    ReferenceTracker
};


class HTCViveTracker
{
public:
    HTCViveTracker();
    ~HTCViveTracker();

    bool InitializeVR();
    bool ShutDownVR();

    // Device
    vector<string> GetAllDeviceNames();
    void PrintAllDetectedDevices();
    void PrintDevicesInformation(enum PRINT);
    bool GetDevicePose(const string& device_name, Affine3d& pose);
    bool UpdateDevicePose(const int device_id);
    bool IsDeviceDetected(const string& device_name);

    bool StackingPose(COORD coord, const Affine3d& pose, int skip_frame_num, int stack_num, const Affine3d& refPosInv = Affine3d::Identity());
    Affine3d AveragingPose();
    void ClearStackingData();

    void StackingPosition(Vector3d& xyz, FITTING opt);
    void Regression(FITTING opt);
    void WriteStackingPositionData(FITTING opt, bool isCosphere = false);
    void WriteRegressionResults(string fileName);
    void CoSphereRegression(const MatrixXd& V1, const MatrixXd& V2, double fixZ);
    void CoSphereRegression(const MatrixXd& V1, const MatrixXd& V2);
    MatrixXd GetStackingPosition() { 
        MatrixXd stack;
        stack.resize(xyzVec.size(), xyzVec[0].rows());
        for (size_t i = 0; i < xyzVec.size(); i++) {
            stack.row(i) = xyzVec[i].transpose();
        }
        return stack;
    }
    
    // Tracker
    int GetRefTrackerIndex() { return ref_tracker_index;  }
    void SetReferenceTracker(int id) { 
        ref_tracker_index = id; 
        ref_tracker_serial = devices_serial[id]; 
    }
    void SetSubTracker(int id) { 
        sub_tracker_index = id; 
        sub_tracker_serial = devices_serial[id]; 
    }
    
    string GetDeviceSerial(const int device_id) { return devices_serial[device_id]; }
    string GetDeviceName(const int device_id) { return devices_names[device_id]; }
    vector<int> GetTrackerIDs() { return trackerIDs; }
    
    
    //vector<int> GetSubTrackerIDs() { return subTrackerIDs; }
    

private:
    string GetDeviceClass(const int device_id);
    string SetDeviceName(const int device_id);
    Vector4d averaging_quaternions(vector<Vector4d> quaternions);
    
private:
    vr::IVRSystem* vr_system; //IVR System interface (main)
    vr::IVRChaperone* vr_chaperone; //IVR Chaperone (to get boundaries and size)
    vr::TrackedDevicePose_t device_poses[vr::k_unMaxTrackedDeviceCount]; //array that contains the positions of all the detected devices

    map<string, uint32_t> devices_id; // map to obtain the device id from the device string
    vector<string> devices_names;     // vector to obtain the device name from the device id
    vector<string> devices_serial;

    const uint32_t max_devices = vr::k_unMaxTrackedDeviceCount; // max number of devices = 64
    int hmd_counts = 1, controller_counts = 1, tracker_counts = 1, track_reference_counts = 1, null_counts = 1;

    const char* NAME_HMD = "HMD";
    const char* NAME_TRACKER = "TRACKER";
    const char* NAME_TREFERENCE = "TREFERENCE";
    const char* NAME_NULL = "INVALID";

    int ref_tracker_index;
    string ref_tracker_serial;
    Affine3d ref_tracker_pose;
    int sub_tracker_index;
    string sub_tracker_serial;


    vector<string> list_of_devices;
    vector<int> devicesIDs;
    vector<int> trackerIDs;
    vector<int> stationIDs;

    int frameNo;
    vector<Vector4d> init_qVec, cum_qVec;
    Vector4d avgInit_q;
    vector<Vector3d> cum_tVec;
    Affine3d pose;

    int columnSize;
    MatrixXd A;
    VectorXd b, ans;
    vector<Vector3d> xyzVec;
    Vector3d abc;
    double radius, R2;
    int cosphere;
};






#endif