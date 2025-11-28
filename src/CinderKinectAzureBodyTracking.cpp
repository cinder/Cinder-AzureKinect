#include "CinderKinectAzureBodyTracking.h"

#include <k4abt.h>

#include "cinder/Log.h"
#include "cinder/Utilities.h"

using namespace std;
using namespace ci;

namespace cika {

BodyTracker::BodyTracker( Camera *camera )
	: mCamera( camera )
{
	k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
	trackerConfig.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
	k4a_calibration_t calibration = mCamera->getCalibration();
	if( K4A_RESULT_SUCCEEDED != k4abt_tracker_create( &calibration, trackerConfig, &mK4aTracker ) )
		throw ExceptionTracker();
	mBodiesTimestamp = 0;
	mTemporalSmoothing = K4ABT_DEFAULT_TRACKER_SMOOTHING_FACTOR;

	mSignalConnection = mCamera->getSignalNewCapture().connect( std::bind( &BodyTracker::newCaptureFn, this, std::placeholders::_1 ) );
}

BodyTracker::~BodyTracker()
{
	mSignalConnection.disconnect();
	k4abt_tracker_shutdown( mK4aTracker );
	k4abt_tracker_destroy( mK4aTracker );
}

void BodyTracker::newCaptureFn( k4a_capture_t capture )
{
	try {
		if( mK4aTracker ) {
			if( k4abt_tracker_enqueue_capture( mK4aTracker, capture, K4A_WAIT_INFINITE ) == K4A_WAIT_RESULT_FAILED )
				return;
		}
	}
	catch(...) {
		CI_LOG_E( "Uncaught exception on BodyTracker" );
	}
}

bool BodyTracker::update( bool waitForFrame )
{
	if( ! mK4aTracker )
		return false;

	int32_t timeoutMs = ( waitForFrame ) ? K4A_WAIT_INFINITE : 0;
	k4abt_frame_t bodyFrame = nullptr;
	if( K4A_WAIT_RESULT_SUCCEEDED != k4abt_tracker_pop_result( mK4aTracker, &bodyFrame, timeoutMs ) )
		return false;

	float scaleYZ = mCamera->scaleFromMm(), scaleX = mCamera->scaleFromMm();
	if( mCamera->getCoordinateSystem() == CoordinateSystem::RIGHT_HANDED_Y_UP )
		scaleYZ = -scaleYZ;

	uint32_t numBodies = k4abt_frame_get_num_bodies( bodyFrame );
	vector<Body> newBodies( numBodies );
	for( uint32_t i = 0; i < numBodies; i++ ) {
		k4abt_skeleton_t skeleton;
		k4abt_frame_get_body_skeleton( bodyFrame, i, &skeleton );
		uint32_t id = k4abt_frame_get_body_id( bodyFrame, i );
		newBodies[i].id = id;
		for( size_t j = 0; j < K4ABT_JOINT_COUNT; ++j ) {
			newBodies[i].joints[j].position = glm::vec3( skeleton.joints[j].position.xyz.x * scaleX, skeleton.joints[j].position.xyz.y * scaleYZ, skeleton.joints[j].position.xyz.z * scaleYZ );
			newBodies[i].joints[j].orientation = glm::quat( skeleton.joints[j].orientation.wxyz.w, skeleton.joints[j].orientation.wxyz.x, skeleton.joints[j].orientation.wxyz.y, skeleton.joints[j].orientation.wxyz.z );
			newBodies[i].joints[j].velocity = glm::vec3( 0 );
			newBodies[i].joints[j].confidence = skeleton.joints[j].confidence_level;
			if( j == 0 )
				newBodies[i].extents = AxisAlignedBox( newBodies[i].joints[j].position, newBodies[i].joints[j].position );
			else
				newBodies[i].extents.include( newBodies[i].joints[j].position );
		}
	}

	uint64_t newTimestamp = k4abt_frame_get_device_timestamp_usec( bodyFrame );
	float timeScale = 0.0f;
	if( mBodiesTimestamp != 0 && newTimestamp != mBodiesTimestamp )
		timeScale = 1000000.0f / (newTimestamp - mBodiesTimestamp);

	// calculate velocity
	for( uint32_t i = 0; i < newBodies.size(); i++ ) {
		uint32_t prev;
		for( prev = 0; prev < mBodies.size(); ++prev )
			if( mBodies[prev].id == newBodies[i].id )
				break;
		if( prev == mBodies.size() ) // couldn't find a match; must be a brand new body
			continue;
		for( size_t j = 0; j < K4ABT_JOINT_COUNT; ++j )
			newBodies[i].joints[j].velocity = (newBodies[i].joints[j].position - mBodies[prev].joints[j].position) * timeScale;
	}

	std::swap( mBodies, newBodies );
	mBodiesTimestamp = newTimestamp;

	k4abt_frame_release( bodyFrame );
	return true;
}

std::vector<Body> BodyTracker::getBodies() const
{
	return mBodies;
}

void BodyTracker::setTemporalSmoothing( float smoothing )
{
	mTemporalSmoothing = glm::clamp( smoothing, 0.0f, 1.0f );
	k4abt_tracker_set_temporal_smoothing( mK4aTracker, mTemporalSmoothing );
}

// skeletons
static const vector<pair<uint8_t, uint8_t>> sSkeletonBones = {
			// left arm
			{ K4ABT_JOINT_HANDTIP_LEFT, K4ABT_JOINT_HAND_LEFT }, { K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_WRIST_LEFT }, { K4ABT_JOINT_THUMB_LEFT, K4ABT_JOINT_WRIST_LEFT },
			{ K4ABT_JOINT_WRIST_LEFT, K4ABT_JOINT_ELBOW_LEFT }, { K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_SHOULDER_LEFT },
			{ K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_CLAVICLE_LEFT }, { K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SPINE_CHEST },
			// right arm
			{ K4ABT_JOINT_HANDTIP_RIGHT, K4ABT_JOINT_HAND_RIGHT }, { K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_WRIST_RIGHT }, { K4ABT_JOINT_THUMB_RIGHT, K4ABT_JOINT_WRIST_RIGHT },
			{ K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_ELBOW_RIGHT }, { K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT },
			{ K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_CLAVICLE_RIGHT }, { K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SPINE_CHEST },
			// head & spine
			{ K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HEAD }, { K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_PELVIS },
			// left leg
			{ K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT }, { K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT }, { K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT },
			{ K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT },
			// right leg
			{ K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT }, { K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT }, { K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT },
			{ K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT } };

static const vector<uint8_t> sSkeletonJoints = {	K4ABT_JOINT_HANDTIP_LEFT, K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_WRIST_LEFT, K4ABT_JOINT_THUMB_LEFT, K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_CLAVICLE_LEFT,
														K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_THUMB_RIGHT, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT,
														K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_HEAD, K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT,
														K4ABT_JOINT_FOOT_LEFT, K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT };

static const std::map<k4abt_joint_id_t, std::string> sSkeletonJointNames =
{
	std::make_pair(K4ABT_JOINT_PELVIS,        "PELVIS"),
	std::make_pair(K4ABT_JOINT_SPINE_NAVEL,   "SPINE_NAVEL"),
	std::make_pair(K4ABT_JOINT_SPINE_CHEST,   "SPINE_CHEST"),
	std::make_pair(K4ABT_JOINT_NECK,          "NECK"),
	std::make_pair(K4ABT_JOINT_CLAVICLE_LEFT, "CLAVICLE_LEFT"),
	std::make_pair(K4ABT_JOINT_SHOULDER_LEFT, "SHOULDER_LEFT"),
	std::make_pair(K4ABT_JOINT_ELBOW_LEFT,    "ELBOW_LEFT"),
	std::make_pair(K4ABT_JOINT_WRIST_LEFT,    "WRIST_LEFT"),
	std::make_pair(K4ABT_JOINT_HAND_LEFT,     "HAND_LEFT"),
	std::make_pair(K4ABT_JOINT_HANDTIP_LEFT,  "HANDTIP_LEFT"),
	std::make_pair(K4ABT_JOINT_THUMB_LEFT,    "THUMB_LEFT"),
	std::make_pair(K4ABT_JOINT_CLAVICLE_RIGHT,"CLAVICLE_RIGHT"),
	std::make_pair(K4ABT_JOINT_SHOULDER_RIGHT,"SHOULDER_RIGHT"),
	std::make_pair(K4ABT_JOINT_ELBOW_RIGHT,   "ELBOW_RIGHT"),
	std::make_pair(K4ABT_JOINT_WRIST_RIGHT,   "WRIST_RIGHT"),
	std::make_pair(K4ABT_JOINT_HAND_RIGHT,    "HAND_RIGHT"),
	std::make_pair(K4ABT_JOINT_HANDTIP_RIGHT, "HANDTIP_RIGHT"),
	std::make_pair(K4ABT_JOINT_THUMB_RIGHT,   "THUMB_RIGHT"),
	std::make_pair(K4ABT_JOINT_HIP_LEFT,      "HIP_LEFT"),
	std::make_pair(K4ABT_JOINT_KNEE_LEFT,     "KNEE_LEFT"),
	std::make_pair(K4ABT_JOINT_ANKLE_LEFT,    "ANKLE_LEFT"),
	std::make_pair(K4ABT_JOINT_FOOT_LEFT,     "FOOT_LEFT"),
	std::make_pair(K4ABT_JOINT_HIP_RIGHT,     "HIP_RIGHT"),
	std::make_pair(K4ABT_JOINT_KNEE_RIGHT,    "KNEE_RIGHT"),
	std::make_pair(K4ABT_JOINT_ANKLE_RIGHT,   "ANKLE_RIGHT"),
	std::make_pair(K4ABT_JOINT_FOOT_RIGHT,    "FOOT_RIGHT"),
	std::make_pair(K4ABT_JOINT_HEAD,          "HEAD"),
	std::make_pair(K4ABT_JOINT_NOSE,          "NOSE"),
	std::make_pair(K4ABT_JOINT_EYE_LEFT,      "EYE_LEFT"),
	std::make_pair(K4ABT_JOINT_EAR_LEFT,      "EAR_LEFT"),
	std::make_pair(K4ABT_JOINT_EYE_RIGHT,     "EYE_RIGHT"),
	std::make_pair(K4ABT_JOINT_EAR_RIGHT,     "EAR_RIGHT")
};

const std::vector<std::pair<uint8_t, uint8_t>>& getSkeletonBones()
{
	return sSkeletonBones;
}

const std::vector<uint8_t>& getSkeletonJoints()
{
	return sSkeletonJoints;
}

k4abt_joint_id_t jointNameToId( const std::string& name )
{
	std::string upper = name;
	for( int i = 0; i < upper.length(); i++ )
		upper[i] = toupper(upper[i]);
	
	for( auto& joint: sSkeletonJointNames )
		if( joint.second == upper )
			return joint.first;

	return K4ABT_JOINT_COUNT; // wasn't found
}

std::string jointIdToName( k4abt_joint_id_t joint )
{
	auto it = sSkeletonJointNames.find( joint );
	if( it != sSkeletonJointNames.end() )
		return it->second;
	else
		return "";
}

} // namespace cika