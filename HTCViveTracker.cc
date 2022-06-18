#include "HTCViveTracker.hh"

HTCViveTracker::HTCViveTracker()
{
	ref_tracker_index = -1;
	//ref_tracker_serial = "LHR-FEA44485";

	frameNo = 0;
	cosphere = 0;
}

HTCViveTracker::~HTCViveTracker()
{
	ShutDownVR();
}

bool HTCViveTracker::InitializeVR()
{
	cout << ">> VR Initialization " << endl;
	bool runtime_ok = vr::VR_IsRuntimeInstalled();
	bool hmd_present = vr::VR_IsHmdPresent();
	vr::EVRInitError err;
	vr_system = vr::VR_Init(&err, vr::VRApplication_Background);
	string init_error = vr::VR_GetVRInitErrorAsSymbol(err);

	cout << "  VR is runtime installed: " << runtime_ok << endl;
	cout << "  VR is HMD present: " << hmd_present << endl;
	cout << "  VR init error: " << init_error << endl;

	if (runtime_ok && hmd_present && err == vr::VRInitError_None) {
		for (int i = 0; i < max_devices; ++i) {
			devices_names.push_back("");
		}
		vr_chaperone = (vr::IVRChaperone*)vr::VR_GetGenericInterface(vr::IVRChaperone_Version, &err);
		if (err == 0) cout << "  Chaperone initialized correctly" << endl << endl;
		else    	  cout << "  Problem initializing Chaperone" << endl << endl;

		vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, device_poses, max_devices);
		for (int i = 0; i < max_devices; i++) {
			if (device_poses[i].bDeviceIsConnected && device_poses[i].bPoseIsValid) {
				string device_name = SetDeviceName(i);
				devices_id[device_name] = i;
				devices_names[i] = device_name;
				char serial_number[1024];
				vr::VRSystem()->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, serial_number, sizeof(serial_number));
				devices_serial.push_back(string(serial_number));

				if (device_name.find(NAME_TRACKER) != string::npos) trackerIDs.push_back(i);
				else if (device_name.find(NAME_TREFERENCE) != string::npos) stationIDs.push_back(i);

				//if (string(serial_number) == ref_tracker_serial) {
				//	ref_tracker_index = i;
				//}
				//else {
				//	if (device_name.find(NAME_TRACKER) != string::npos) subTrackerIDs.push_back(i);
				//}			
			}
		}
	}

	list_of_devices = GetAllDeviceNames();
}

bool HTCViveTracker::ShutDownVR()
{
	if (vr_system) {
		cout << ">> VR is shut down." << endl;
		vr::VR_Shutdown();
		return true;
	}
	else false;
}

bool HTCViveTracker::UpdateDevicePose(const int device_id)
{
	vr::ETrackedDeviceClass device_class = vr_system->GetTrackedDeviceClass(device_id);
	vr::TrackedDevicePose_t tracked_device_pose;
	vr::VRControllerState_t controller_state;
	vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, device_poses, max_devices);
	if (device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller
		or device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker) {
		vr_system->GetControllerStateWithPose(vr::TrackingUniverseStanding, device_id, &controller_state, sizeof(controller_state), &tracked_device_pose);
		device_poses[device_id] = tracked_device_pose;
	}
	else if (device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid) return false;

	return true;
}

bool HTCViveTracker::GetDevicePose(const string& device_name, Affine3d& pose)
{
	if (devices_id.find(device_name) == devices_id.end()) return false;
	int device_index = devices_id[device_name];
	if (UpdateDevicePose(device_index)) {
		vr::TrackedDevicePose_t current_device_pose = device_poses[device_index];
		if (current_device_pose.bDeviceIsConnected && current_device_pose.bPoseIsValid) {
			vr::HmdMatrix34_t matrix = current_device_pose.mDeviceToAbsoluteTracking;
			Vector3d trans(matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]);
			Matrix3d rot;
			rot << matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
				   matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
				   matrix.m[2][0], matrix.m[2][1], matrix.m[2][2];
			Quaterniond quat(rot);

			pose.translation() = trans;
			pose.linear() = quat.matrix();
			
			Affine3d rotateY = Affine3d::Identity();
			rotateY.linear() = AngleAxisd(M_PI, Vector3d::UnitY()).matrix();
			pose = pose * rotateY;
			
			if (device_index == ref_tracker_index) {
				ref_tracker_pose = pose;
			}
			return true;
		}
	}
	return false;
}

void HTCViveTracker::PrintDevicesInformation(PRINT opt)
{
	vector<int> printIDs;
	bool isSub(false);
	switch (opt) {
	case PRINT::AllDevices:   printIDs = devicesIDs; break;
	case PRINT::AllTrackers:  printIDs = trackerIDs; break;
	case PRINT::RefTracker:   printIDs.push_back(ref_tracker_index); break;
	case PRINT::SubTrackers:  printIDs = trackerIDs; isSub = true; break;
	case PRINT::BaseStations: printIDs = stationIDs; break;
	}

	for (int i: printIDs) {
		Affine3d pose = Affine3d::Identity();
		if (IsDeviceDetected(list_of_devices[i])) {
			bool success = GetDevicePose(list_of_devices[i], pose);
			cout << fixed;
			cout.precision(5);

			if (success) {
				if (isSub && i == ref_tracker_index) continue;
				Affine3d pose_to_ref = ref_tracker_pose.inverse() * pose;
				cout << setw(15) << list_of_devices[i] << " Position (xyz,cm): " << pose_to_ref.translation().transpose() * 100 << endl;
				//cout << setw(15) << list_of_devices[i] << " Quaternion (xyzw): " << Quaterniond(pose_to_ref.linear()).coeffs().transpose() << endl;
			}
		}
	}

}

bool HTCViveTracker::IsDeviceDetected(const string& device_name) 
{
	if (devices_id.find(device_name) == devices_id.end()) 
		return false;
	uint32_t device_index = devices_id[device_name];
	if (device_index < max_devices)
		return vr_system->IsTrackedDeviceConnected(device_index);
	else return false;
}

vector<string> HTCViveTracker::GetAllDeviceNames()
{
	vector<string> non_empty_device_names;
	for (size_t i = 0; i < devices_names.size(); ++i) {
		if (devices_names[i] != "")
			non_empty_device_names.push_back(devices_names[i]);
	}
	return non_empty_device_names;
}

void HTCViveTracker::PrintAllDetectedDevices() 
{
	cout << ">> Detected devices list: " << endl;
	for (vr::TrackedDeviceIndex_t device_index = vr::k_unTrackedDeviceIndex_Hmd; device_index < max_devices; ++device_index) {
		if (device_poses[device_index].bDeviceIsConnected && device_poses[device_index].bPoseIsValid) {
			string device_name = devices_names[device_index];
			string device_serial = devices_serial[device_index];
			cout << "  device[" + to_string(device_index) + "]: ";
			//if (device_serial == ref_tracker_serial)
			//	cout << setw(15) << device_name << setw(25) << " (" + device_serial + ")" << ", Refernece Tracker" << endl;
			//else if (device_name.find(NAME_TRACKER) != string::npos && device_serial != ref_tracker_serial)
			//	cout << setw(15) << device_name << setw(25) << " (" + device_serial + ")" << ", Sub-Tracker" << endl;
			//else
				cout << setw(15) << device_name << setw(25) << " (" + device_serial + ")" << endl;
		}
	}
	cout << endl;
}

string HTCViveTracker::GetDeviceClass(const int device_id) 
{
	vr::ETrackedDeviceClass tracked_device_class = vr_system->GetTrackedDeviceClass(device_id);
	if      (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD)               return NAME_HMD;
	else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) return NAME_TREFERENCE;
	else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker)    return NAME_TRACKER;
	else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid)    		return NAME_NULL;
	else return NAME_NULL;
}

string HTCViveTracker::SetDeviceName(const int device_id)
{
	string class_name = GetDeviceClass(device_id);
	string device_name;

	if (class_name == NAME_HMD) {
		device_name = class_name + "_" + std::to_string(hmd_counts);
		hmd_counts++;
	}
	else if (class_name == NAME_TREFERENCE) {
		device_name = class_name + "_" + std::to_string(track_reference_counts);
		track_reference_counts++;
	}
	else if (class_name == NAME_TRACKER) {
		device_name = class_name + "_" + std::to_string(tracker_counts);
		tracker_counts++;
	}
	else if (class_name == NAME_NULL) {
		device_name = class_name + "_" + std::to_string(null_counts);
		null_counts++;
	}
	else device_name = NAME_NULL;

	return device_name;

}

bool HTCViveTracker::StackingPose(COORD coord, const Affine3d& pose, int skip_frame_num, int stack_num, const Affine3d& refPosInv)
{
	Affine3d tracker_pose;
	if (coord == COORD::Absolute) {
	tracker_pose = pose;
	}
	else if (coord == COORD::ReferenceTracker) {
		tracker_pose = refPosInv * pose;
	}

	bool isStack(false);
	Vector3d trans(tracker_pose.translation());
	Vector4d quat(Quaterniond(tracker_pose.linear()).coeffs());

	if (frameNo < 10) {
		init_qVec.push_back(quat);
		frameNo++;
	}
	else if (frameNo == skip_frame_num) {
		avgInit_q = averaging_quaternions(init_qVec);
		frameNo++;
	}
	else {
		Vector3d axisX0 = Quaterniond(avgInit_q).matrix() * Vector3d::UnitX();
		Vector3d axisY0 = Quaterniond(avgInit_q).matrix() * Vector3d::UnitY();
		Vector3d axisZ0 = Quaterniond(avgInit_q).matrix() * Vector3d::UnitZ();
		Vector3d axisX1 = Quaterniond(quat).matrix() * Vector3d::UnitX();
		Vector3d axisY1 = Quaterniond(quat).matrix() * Vector3d::UnitY();
		Vector3d axisZ1 = Quaterniond(quat).matrix() * Vector3d::UnitZ();

		double dotX = axisX0.dot(axisX1);
		double dotY = axisY0.dot(axisY1);
		double dotZ = axisZ0.dot(axisZ1);

		if (dotX > 0 && dotY > 0 && dotZ > 0) {
			frameNo++;
			isStack = true;
		}
	}

	if (isStack) {
		cum_qVec.push_back(quat);
		cum_tVec.push_back(trans);
		cout << "\rStacking: " << cum_qVec.size() << " / " << stack_num << flush;
	}

	if (cum_qVec.size() == stack_num)
		return true;
	else
		return false;
}

Affine3d HTCViveTracker::AveragingPose()
{
	Vector3d avg_t;
	Vector4d avg_q;
	Affine3d avg_pose;
	for (const auto& t : cum_tVec)
		avg_t += t;
	avg_t /= cum_tVec.size();
	avg_q = averaging_quaternions(cum_qVec);
	
	avg_pose.translation() = avg_t;
	avg_pose.linear() = Quaterniond(avg_q).matrix();
	return avg_pose;
}

void HTCViveTracker::ClearStackingData()
{
	frameNo = 0;
	init_qVec.clear();
	cum_qVec.clear();
	cum_tVec.clear();
	avgInit_q = Vector4d::Zero();
	pose = Affine3d::Identity();
	A.resize(0, 0);
	b.resize(0);
	xyzVec.clear();
	columnSize = 0;
	radius = 0.;
	R2 = 0.;
}

void HTCViveTracker::StackingPosition(Vector3d& xyz, FITTING opt)
{
	xyzVec.push_back(xyz);
	Vector3d xyz_ = xyz;
	double bValue;
	switch (opt) {
	case FITTING::CIRCLE_X:
		columnSize = 3;
		xyz_ = Vector3d(xyz.y(), xyz.z(), 1.);
		bValue = xyz.y() * xyz.y() + xyz.z() * xyz.z();
		break;
	case FITTING::CIRCLE_Y:
		columnSize = 3;
		xyz_ = Vector3d(xyz.x(), xyz.z(), 1.);
		bValue = xyz.x() * xyz.x() + xyz.z() * xyz.z();
		break;
	case FITTING::CIRCLE_Z:
		columnSize = 3;
		xyz_ = Vector3d(xyz.x(), xyz.y(), 1.);
		bValue = xyz.x() * xyz.x() + xyz.y() * xyz.y();
		break;
	case FITTING::SPHERE:
		columnSize = 4;
		xyz_ = Vector3d(xyz.x(), xyz.y(), xyz.z());
		bValue = xyz.x() * xyz.x() + xyz.y() * xyz.y() + xyz.z() * xyz.z();
		break;
	case FITTING::SPHERE_1135Z:
		columnSize = 3;
		xyz_ = Vector3d(xyz.x(), xyz.y(), 1.);
		bValue = xyz.x() * xyz.x() + xyz.y() * xyz.y() + xyz.z() * xyz.z() + 1.135 * 1.135 - 2 * 1.135 * xyz.z();
		break;
	}
	ans.resize(columnSize);
	A.conservativeResize(frameNo + 1, columnSize);
	b.conservativeResize(frameNo + 1);

	if (opt == FITTING::SPHERE) A.row(frameNo) = RowVector4d(xyz.x(), xyz.y(), xyz.z(), 1.);
	else                        A.row(frameNo) = xyz_.transpose();
	b(frameNo) = bValue;	
	frameNo++;
}

void HTCViveTracker::WriteStackingPositionData(FITTING opt, bool isCosphere)
{
	string prefix;
	switch (opt) {
	case FITTING::CIRCLE_X: prefix = "circleX_"; break;
	case FITTING::CIRCLE_Y: prefix = "circleY_"; break;
	case FITTING::CIRCLE_Z: prefix = "circleZ_"; break;
	case FITTING::SPHERE:   prefix = "sphere_"; break;
	case FITTING::SPHERE_1135Z: prefix = "sphereZ1135_"; break;
	}
	if (isCosphere)	prefix = "cosphere_";

	string fileName;
	cout << "Enter the file name: "; cin >> fileName;
	fileName = prefix + fileName;
	ofstream ofs(fileName + ".obj");
	ofs << fixed;
	ofs.precision(10);
	for (size_t i = 0; i < xyzVec.size(); i++) {
		ofs << "v " << xyzVec[i].transpose() * 100 << endl; // cm
	} ofs.close();
	WriteRegressionResults(fileName);
}

void HTCViveTracker::WriteRegressionResults(string fileName)
{
	ofstream ofs(fileName + ".out");
	ofs << "Center(cm): " << abc.transpose() * 100 << endl;
	ofs << "Radius(cm): " << radius * 100 << endl;
	ofs << "R^2       : " << R2 << endl;
	ofs.close();
}

void HTCViveTracker::Regression(FITTING opt)
{
	ans = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
	double aa, bb, cc;
	switch (opt) {
	case FITTING::CIRCLE_X:
		bb = ans(0) * 0.5;
		cc = ans(1) * 0.5;
		aa = 1.;
		radius = sqrt(ans(2) + bb * bb + cc * cc);
		break;
	case FITTING::CIRCLE_Y:
		aa = ans(0) * 0.5;
		cc = ans(1) * 0.5;
		bb = 1.;
		radius = sqrt(ans(2) + aa * aa + cc * cc);
		break;
	case FITTING::CIRCLE_Z:
		aa = ans(0) * 0.5;
		bb = ans(1) * 0.5;
		cc = 1.;
		radius = sqrt(ans(2) + aa * aa + bb * bb);
		break;
	case FITTING::SPHERE:
		aa = ans(0) * 0.5;
		bb = ans(1) * 0.5;
		cc = ans(2) * 0.5;
		radius = sqrt(ans(3) + aa * aa + bb * bb + cc * cc);
		break;
	case FITTING::SPHERE_1135Z:
		aa = ans(0) * 0.5;
		bb = ans(1) * 0.5;
		cc = 1.135;
		radius = sqrt(ans(2) + aa * aa + bb * bb);
		break;
	}
	
	abc = Vector3d(aa, bb, cc);
	R2 = ((A.rowwise() - abc.transpose()).rowwise().squaredNorm().array() - radius * radius).sum() / (double)b.rows();
	cout << "center(cm): " << abc.transpose() * 100 << endl;
	cout << "radius(cm): " << radius * 100 << endl;
	cout << "r^2: " << R2 << endl;
}

void HTCViveTracker::CoSphereRegression(const MatrixXd& V1, const MatrixXd &V2, double fixZ)
{
	cout << "[COSPHERE regression - with fixed Z : " << fixZ << "]" << endl;

	//generation of A and b
	MatrixXd A = MatrixXd::Ones(V1.rows() + V2.rows(), 4);
	A.block(0, 0, V1.rows(), 2) = V1.leftCols(2);
	A.block(V1.rows(), 0, V2.rows(), 2) = V2.leftCols(2);
	A.block(0, 3, V1.rows(), 1) = VectorXd::Zero(V1.rows());
	A.block(V1.rows(), 2, V2.rows(), 1) = VectorXd::Zero(V2.rows());
	MatrixXd Vtmp(V1.rows() + V2.rows(), 3);
	Vtmp.topRows(V1.rows()) = V1;
	Vtmp.bottomRows(V2.rows()) = V2;
	Vtmp.col(2) = Vtmp.col(2).array() - fixZ;
	VectorXd b = Vtmp.cwiseAbs2().rowwise().sum();

	cout << "1st regression" << endl;
	VectorXd ans1 = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
	Vector2d center = ans1.topRows(3) * 0.5;
	cout << "center(cm): " << center.transpose() << " " << fixZ << endl;
	double radius1 = sqrt(ans1(2) + center.squaredNorm());
	double radius2 = sqrt(ans1(3) + center.squaredNorm());
	cout << "radius1(cm): " << radius1 << endl;
	cout << "radius2(cm): " << radius2 << endl;

	//statistical evaluation
	VectorXd residual = b - A * ans1;
	double SSE = residual.cwiseAbs2().sum();
	double SST = (b.array() - (b.sum() / (double)b.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = ((Vtmp.leftCols(2).rowwise() - center.transpose()).rowwise().squaredNorm() + Vtmp.rightCols(1).cwiseAbs2()).cwiseSqrt()
		- A.col(2) * radius1 - A.col(3) * radius2;
	double s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;

	cout << "-------------refinement 1--------------" << endl; //reject |deviation|>2*(standard dev.) points and re-regression
	VectorXi safe = (residual.cwiseAbs().array() < 2 * s).cast<int>();
	VectorXi safeRows(safe.sum());
	for (int i = 0, r = 0, o = 0; i < safe.rows(); i++)
		if (safe(i)) safeRows(r++) = i;

	MatrixXd A1 = igl::slice(A, safeRows, 1);
	VectorXd b1 = igl::slice(b, safeRows, 1);
	ans1 = A1.bdcSvd(ComputeThinU | ComputeThinV).solve(b1);
	center = ans1.topRows(2) * 0.5;
	cout << safe.rows() << "->" << safe.sum() << endl;
	cout << "center(m): " << center.transpose() << " " << fixZ << endl;
	radius1 = sqrt(ans1(2) + center.squaredNorm());
	radius2 = sqrt(ans1(3) + center.squaredNorm());
	cout << "radius1(m): " << radius1 << endl;
	cout << "radius2(m): " << radius2 << endl;

	//statistical evaluation
	residual = b1 - A1 * ans1;
	SSE = residual.cwiseAbs2().sum();
	SST = (b1.array() - (b1.sum() / (double)b1.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = ((Vtmp.leftCols(2).rowwise() - center.transpose()).rowwise().squaredNorm() + Vtmp.rightCols(1).cwiseAbs2()).cwiseSqrt()
		- A1.col(2) * radius1 - A1.col(3) * radius2;
	s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;
	cout << "S'(distance to center, selected) = " << sqrt(igl::slice(residual, safeRows, 1).cwiseAbs2().sum() / ((double)safeRows.rows() - 1)) << endl;

	cout << "-------------refinement 2--------------" << endl; //reject |deviation|>(standard dev.) points and re-regression
	safe = (residual.cwiseAbs().array() < s).cast<int>();
	safeRows.resize(safe.sum());
	for (int i = 0, r = 0, o = 0; i < safe.rows(); i++)
		if (safe(i)) safeRows(r++) = i;

	A1 = igl::slice(A, safeRows, 1);
	b1 = igl::slice(b, safeRows, 1);
	ans1 = A1.bdcSvd(ComputeThinU | ComputeThinV).solve(b1);
	center = ans1.topRows(2) * 0.5;
	cout << safe.rows() << "->" << safe.sum() << endl;
	cout << "center(m): " << center.transpose() << " " << fixZ << endl;
	radius1 = sqrt(ans1(2) + center.squaredNorm());
	radius2 = sqrt(ans1(3) + center.squaredNorm());
	cout << "radius1(m): " << radius1 << endl;
	cout << "radius2(m): " << radius2 << endl;

	//statistical evaluation
	residual = b1 - A1 * ans1;
	SSE = residual.cwiseAbs2().sum();
	SST = (b1.array() - (b1.sum() / (double)b1.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = ((Vtmp.leftCols(2).rowwise() - center.transpose()).rowwise().squaredNorm() + Vtmp.rightCols(1).cwiseAbs2()).cwiseSqrt()
		- A1.col(2) * radius1 - A1.col(3) * radius2;
	s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;
	cout << "S'(distance to center, selected) = " << sqrt(igl::slice(residual, safeRows, 1).cwiseAbs2().sum() / ((double)safeRows.rows() - 1)) << endl;

}

void HTCViveTracker::CoSphereRegression(const MatrixXd& V1, const MatrixXd& V2)
{
	cout << "[COSPHERE regression - without fixed Z]" << endl;

	//generation of A and b
	MatrixXd A = MatrixXd::Ones(V1.rows() + V2.rows(), 5);
	A.block(0, 0, V1.rows(), 3) = V1;
	A.block(V1.rows(), 0, V2.rows(), 3) = V2;
	A.block(0, 4, V1.rows(), 1) = VectorXd::Zero(V1.rows());
	A.block(V1.rows(), 3, V2.rows(), 1) = VectorXd::Zero(V2.rows());
	VectorXd b(V1.rows() + V2.rows());
	b.topRows(V1.rows()) = V1.array().square().rowwise().sum();
	b.bottomRows(V2.rows()) = V2.array().square().rowwise().sum();

	cout << "1st regression" << endl;
	VectorXd ans1 = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
	Vector3d center = ans1.topRows(3) * 0.5;
	cout << "center(m): " << center.transpose() << endl;
	double radius1 = sqrt(ans1(3) + center.squaredNorm());
	double radius2 = sqrt(ans1(4) + center.squaredNorm());
	cout << "radius1(m): " << radius1 << endl;
	cout << "radius2(m): " << radius2 << endl;

	//statistical evaluation
	VectorXd residual = b - A * ans1;
	double SSE = residual.cwiseAbs2().sum();
	double SST = (b.array() - (b.sum() / (double)b.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = (A.leftCols(3).rowwise() - center.transpose()).rowwise().norm() - A.col(3) * radius1 - A.col(4) * radius2;
	double s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;

	cout << "-------------refinement 1--------------" << endl; //reject |deviation|>2*(standard dev.) points and re-regression
	VectorXi safe = (residual.cwiseAbs().array() < 2 * s).cast<int>();
	VectorXi safeRows(safe.sum());
	for (int i = 0, r = 0; i < safe.rows(); i++)
		if (safe(i)) safeRows(r++) = i;

	MatrixXd A1 = igl::slice(A, safeRows, 1);
	VectorXd b1 = igl::slice(b, safeRows, 1);
	ans1 = A1.bdcSvd(ComputeThinU | ComputeThinV).solve(b1);
	center = ans1.topRows(3) * 0.5;
	cout << safe.rows() << "->" << safe.sum() << endl;
	cout << "center(m): " << center.transpose() << endl;
	radius1 = sqrt(ans1(3) + center.squaredNorm());
	radius2 = sqrt(ans1(4) + center.squaredNorm());
	cout << "radius1(m): " << radius1 << endl;
	cout << "radius2(m): " << radius2 << endl;

	//statistical evaluation
	residual = b1 - A1 * ans1;
	SSE = residual.cwiseAbs2().sum();
	SST = (b1.array() - (b1.sum() / (double)b1.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = (A.leftCols(3).rowwise() - center.transpose()).rowwise().norm() - A.col(3) * radius1 - A.col(4) * radius2;
	s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;
	cout << "S'(distance to center, selected) = " << sqrt(igl::slice(residual, safeRows, 1).cwiseAbs2().sum() / ((double)safeRows.rows() - 1)) << endl;

	cout << "-------------refinement 2--------------" << endl; //reject |deviation|>(standard dev.) points and re-regression
	safe = (residual.cwiseAbs().array() < s).cast<int>();
	safeRows.resize(safe.sum());
	for (int i = 0, r = 0; i < safe.rows(); i++)
		if (safe(i)) safeRows(r++) = i;

	A1 = igl::slice(A, safeRows, 1);
	b1 = igl::slice(b, safeRows, 1);
	ans1 = A1.bdcSvd(ComputeThinU | ComputeThinV).solve(b1);
	center = ans1.topRows(3) * 0.5;
	cout << safe.rows() << "->" << safe.sum() << endl;
	cout << "center(m): " << center.transpose() << endl;
	radius1 = sqrt(ans1(3) + center.squaredNorm());
	radius2 = sqrt(ans1(4) + center.squaredNorm());
	cout << "radius1(m): " << radius1 << endl;
	cout << "radius2(m): " << radius2 << endl;

	//statistical evaluation
	residual = b1 - A1 * ans1;
	SSE = residual.cwiseAbs2().sum();
	SST = (b1.array() - (b1.sum() / (double)b1.rows())).abs2().sum();
	cout << "R2 = " << 1. - SSE / SST << endl;
	residual = (A.leftCols(3).rowwise() - center.transpose()).rowwise().norm() - A.col(3) * radius1 - A.col(4) * radius2;
	s = sqrt(residual.cwiseAbs2().sum() / ((double)residual.rows() - 1));
	cout << "S(distance to center) = " << s << endl;
	cout << "S'(distance to center, selected) = " << sqrt(igl::slice(residual, safeRows, 1).cwiseAbs2().sum() / ((double)safeRows.rows() - 1)) << endl;
}

Vector4d HTCViveTracker::averaging_quaternions(vector<Vector4d> quaternions) // x y z w
{
	if (quaternions.empty())
		return Eigen::Vector4d(0, 0, 0, 1);

	// first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
	Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

	for (int q = 0; q < quaternions.size(); ++q)
		A += quaternions[q] * quaternions[q].transpose();

	// normalise with the number of quaternions
	A /= quaternions.size();

	// Compute the SVD of this 4x4 matrix
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::MatrixXd U = svd.matrixU();

	// find the eigen vector corresponding to the largest eigen value
	int largestEigenValueIndex;
	float largestEigenValue;
	bool first = true;

	for (int i = 0; i < singularValues.rows(); ++i)
	{
		if (first)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
			first = false;
		}
		else if (singularValues(i) > largestEigenValue)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
		}
	}

	Eigen::Vector4d average;
	average(0) = -U(0, largestEigenValueIndex);
	average(1) = -U(1, largestEigenValueIndex);
	average(2) = -U(2, largestEigenValueIndex);
	average(3) = -U(3, largestEigenValueIndex);

	return average;
}