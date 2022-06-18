#include "HTCViveTracker.hh"
#include <conio.h>
#include <fstream>

void PrintOptions() {
	cout << "=====================================================" << endl;
	cout << "  Options" << endl;
	cout << "  1. Setting Reference Tracker " << endl;
	cout << "  2. Setting Sub-tracker Position " << endl;
	cout << "  3. Sub-tracker Regression " << endl;
	cout << "  q. Exit" << endl;
	cout << "=====================================================" << endl;
}

Affine3d GetReferenceTrackerPose()
{
	ifstream ifs("referenceTracker.pose");
	if (!ifs.is_open()) { cerr << "referenceTracker.pose was not opend" << endl; exit(1); }
	string dump;
	double x, y, z, w;
	ifs >> dump >> x >> y >> z;      Vector3d trans(x, y, z);
	ifs >> dump >> x >> y >> z >> w; Vector4d quat(x, y, z, w);
	Affine3d pose;
	pose.translation() = trans;
	pose.linear() = Quaterniond(quat).matrix();
	return pose;
}

void SETTING_REFERENCE_TRACKER(HTCViveTracker& hvt);
void SETTING_SUBTRACKER_POSITION(HTCViveTracker& hvt);
void SUBTRACKER_REGRESSION(HTCViveTracker& hvt);

int main(int argc, char** argv)
{
	cout << "=====================================================" << endl;
	cout << "HTC VIVE Tracker for McDCIR initial position settings" << endl;
	cout << "Author: Sungho Moon" << endl;
	cout << "Contributor: Haegin Han, Sungho Moon, Gahee Son" << endl;
	cout << "=====================================================" << endl;
	HTCViveTracker hvt;
	if (hvt.InitializeVR()) {
		if (hvt.GetAllDeviceNames().size() == 1) {
			cerr << "No devices detected. Check link box are connected and devices are paired" << endl;
			return EXIT_FAILURE;
		}
		hvt.PrintAllDetectedDevices();
	}
	else cerr << ">> VR Initialization is failed" << endl;

	char opt;
	bool isExit(false);
	while (true) {
		PrintOptions();
		cout << "Enter the option number: "; cin >> opt;
		switch (opt) {
		case '1': SETTING_REFERENCE_TRACKER(hvt);	break;
		case '2': SETTING_SUBTRACKER_POSITION(hvt);	break;
		case '3': SUBTRACKER_REGRESSION(hvt);	    break;
		case 'q': isExit = true;  break;
		}
		if (isExit) break;
	}
	cout << "EXIT_SUCCESS" << endl;
	return EXIT_SUCCESS;
}

void SETTING_REFERENCE_TRACKER(HTCViveTracker& hvt)
{
	cout << "1. Setting Reference Tracker" << endl;
	cout << "Tracker's list: " << endl;
	int ref_tracker_index(-1);
	vector<int> trackerIDs = hvt.GetTrackerIDs();
	for (auto itr : trackerIDs) {
		cout << "ID:" << itr << ", " << hvt.GetDeviceSerial(itr) << endl;
	} 
	cout << "Select the reference tracker ID: "; cin >> ref_tracker_index;
	while (true) {
		auto it = find(trackerIDs.begin(), trackerIDs.end(), ref_tracker_index);
		if (it != trackerIDs.end()) break;
		else { cout << "Try again: "; cin >> ref_tracker_index; }
	}
	hvt.SetReferenceTracker(ref_tracker_index);
	string ref_tracker_name = hvt.GetDeviceName(ref_tracker_index);
	string ref_tracker_serial = hvt.GetDeviceSerial(ref_tracker_index);
	cout << ref_tracker_name + " (" + ref_tracker_serial + ") is the reference tracker" << endl;
	int skip_frame_num(10);
	int stack_num(1000);
	Affine3d pose, avg_pose;
	while(true) {
		if (!hvt.IsDeviceDetected(ref_tracker_name)) continue;
		hvt.UpdateDevicePose(ref_tracker_index);
		hvt.GetDevicePose(ref_tracker_name, pose);
		if (hvt.StackingPose(COORD::Absolute, pose, skip_frame_num, stack_num)) {
			avg_pose = hvt.AveragingPose();
			hvt.ClearStackingData();
			break;
		}
	}
	cout << endl;
	ofstream ofs("referenceTracker.pose");
	cout << "Reference Tracker Averaing Pose" << endl;
	cout << "Translation(xyz,m): " << avg_pose.translation().transpose() << endl;
	cout << "Quaternion(xyzw): "   << Quaterniond(avg_pose.linear()).coeffs().transpose() << endl;
	ofs << "Translation(xyz,m): "  << avg_pose.translation().transpose() << endl;
	ofs << "Quaternion(xyzw): "    << Quaterniond(avg_pose.linear()).coeffs().transpose() << endl;
}

void SETTING_SUBTRACKER_POSITION(HTCViveTracker& hvt)
{
	cout << "2. Setting Sub-tracker Position" << endl;
	Affine3d refPoseInv = GetReferenceTrackerPose().inverse();
	cout << "Tracker's list: " << endl;
	vector<int> trackerIDs = hvt.GetTrackerIDs();
	int sub_tracker_index(-1);
	for (auto itr : trackerIDs) {
		cout << "ID:" << itr << ", " << hvt.GetDeviceSerial(itr) << endl;
	}
	cout << "Select the sub tracker ID: "; cin >> sub_tracker_index;
	while (true) {
		auto it = find(trackerIDs.begin(), trackerIDs.end(), sub_tracker_index);
		if (it != trackerIDs.end()) break;
		else { cout << "Try again: "; cin >> sub_tracker_index; }
	}
	hvt.SetSubTracker(sub_tracker_index);
	string sub_tracker_name = hvt.GetDeviceName(sub_tracker_index);
	string sub_tracker_serial = hvt.GetDeviceSerial(sub_tracker_index);
	cout << sub_tracker_name + " (" + sub_tracker_serial + ") is the sub-tracker" << endl;
	
	Affine3d pose, avg_pose;
	int skip_frame_num(10);
	int stack_num(1000);
	while (true) {
		if (!hvt.IsDeviceDetected(sub_tracker_name)) continue;
		hvt.UpdateDevicePose(sub_tracker_index);
		hvt.GetDevicePose(sub_tracker_name, pose);
		if (hvt.StackingPose(COORD::ReferenceTracker, pose, skip_frame_num, stack_num, refPoseInv)) {
			avg_pose = hvt.AveragingPose();
			hvt.ClearStackingData();
			break;
		}
	}
	cout << endl;
	string fileName;
	cout << "Enter the file name: "; cin >> fileName;
	ofstream ofs(fileName);
	cout << "Sub-tracker Position" << endl;
	cout << "Translation(xyz,cm): " << avg_pose.translation().transpose() * 100 << endl;
	ofs  << "Translation(xyz,cm): " << avg_pose.translation().transpose() * 100 << endl;
}

void SUBTRACKER_REGRESSION(HTCViveTracker& hvt)
{
	char key;
	cout << "3. Tracker Regression" << endl;
	cout << "[Regression Options]" << endl;
	cout << "1. Circle Fitting - Fixing X " << endl;
	cout << "2. Circle Fitting - Fixing Y " << endl;
	cout << "3. Circle Fitting - Fixing Z " << endl;
	cout << "4. Sphere Fitting" << endl;
	cout << "5. Sphere Fitting - Z=113.5 cm (isocenter)" << endl;
	cout << "6. CoSphere Fitting" << endl;
	cout << "7. CoSphere Fitting - Z=113.5 cm (isocenter)" << endl;
	cout << "q. Exit" << endl;
	cout << "Enter the fitting option: "; cin >> key;
	if (key == 'q') return;
	FITTING opt; bool isCosphere(false); bool isFix(false);
	switch (key) {
	case '1': opt = FITTING::CIRCLE_X; break;
	case '2': opt = FITTING::CIRCLE_Y; break;
	case '3': opt = FITTING::CIRCLE_Z; break;
	case '4': opt = FITTING::SPHERE;   break;
	case '5': opt = FITTING::SPHERE_1135Z; break;
	case '6': opt = FITTING::SPHERE; isCosphere = true;  break;
	case '7': opt = FITTING::SPHERE; isCosphere = true; isFix = true; break;
	case 'q': return;
	}

	Affine3d refPoseInv = GetReferenceTrackerPose().inverse();
	cout << "Tracker's list: " << endl;
	vector<int> trackerIDs = hvt.GetTrackerIDs();
	int sub_tracker_index(-1);
	for (auto itr : trackerIDs) {
		cout << "ID:" << itr << ", " << hvt.GetDeviceSerial(itr) << endl;
	}
	cout << "Select the sub tracker ID: "; cin >> sub_tracker_index;
	while (true) {
		auto it = find(trackerIDs.begin(), trackerIDs.end(), sub_tracker_index);
		if (it != trackerIDs.end()) break;
		else { cout << "Try again: "; cin >> sub_tracker_index; }
	}
	hvt.SetSubTracker(sub_tracker_index);
	string sub_tracker_name = hvt.GetDeviceName(sub_tracker_index);
	string sub_tracker_serial = hvt.GetDeviceSerial(sub_tracker_index);
	cout << sub_tracker_name + " (" + sub_tracker_serial + ") is the sub-tracker" << endl;
	cout << "Press space bar serveral times to get tracker pose and press 'q' to solve matrix" << endl;

	Affine3d pose;
	int count(0); int cosphere(0); MatrixXd V1, V2;
	while (true) {
		int key = _getch();
		if (key == 32) {
			if (!hvt.IsDeviceDetected(sub_tracker_name)) continue;
			hvt.UpdateDevicePose(sub_tracker_index);
			hvt.GetDevicePose(sub_tracker_name, pose);
			Vector3d xyz = (refPoseInv * pose).translation();
			cout << count++ << " -> " << xyz.transpose() << endl;
			hvt.StackingPosition(xyz, opt);
		}
		else if (key == (int)'q') {
			if (isCosphere && cosphere == 0) {
				cout << "First sphere fitting" << endl;
				V1 = hvt.GetStackingPosition();
				cosphere++;
				hvt.WriteStackingPositionData(opt, isCosphere);
				hvt.ClearStackingData();
				cout << "Press space bar serveral times again and press 'q' to solve matrix" << endl;
			}
			else if (isCosphere && cosphere == 1) {
				cout << "Second sphere fitting" << endl;
				V2 = hvt.GetStackingPosition();
				hvt.WriteStackingPositionData(opt, isCosphere);
				if (isFix) hvt.CoSphereRegression(V1, V2, 1.135);
				else       hvt.CoSphereRegression(V1, V2);
				break;
			}
			else {
				hvt.Regression(opt);
				hvt.WriteStackingPositionData(opt);
				break;
			}
		}
	}
	hvt.ClearStackingData();
}