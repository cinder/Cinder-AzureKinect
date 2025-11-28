#pragma once

#include "CinderKinectAzure.h"

#include <k4abttypes.h>

namespace cika {

class BodyTrackingConfig {
  public:
	BodyTrackingConfig() {}
};

struct Joint {
	glm::vec3						position;
	glm::quat						orientation;
	glm::vec3						velocity; // units / second
	k4abt_joint_confidence_level_t	confidence;
};

struct Body {
	uint32_t				id;
	Joint					joints[K4ABT_JOINT_COUNT];
	cinder::AxisAlignedBox	extents;
};

class BodyTracker {
  public:
	BodyTracker( Camera *camera );
	~BodyTracker();

	//! Returns \c true if a new body tracking data is available. If \a waitForFrame is \c true then the call doesn't return until new body tracking data has been acquired
	bool					update( bool waitForFrame = false );
	std::vector<Body>		getBodies() const;
	
	void					setTemporalSmoothing( float smoothing );
	float					getTemporalSmoothing() const { return mTemporalSmoothing; }

  private:
	void	newCaptureFn( k4a_capture_t capture );

  	k4abt_tracker_t									mK4aTracker = nullptr;
	Camera											*mCamera;
	k4abt_tracker_configuration_t					mK4aTrackerConfiguration;
	std::vector<Body>								mBodies;
	uint64_t										mBodiesTimestamp;
	float											mTemporalSmoothing;
	cinder::signals::Connection						mSignalConnection;
};

//! Returns a list of joint pairs (bones) that can be connected to create a skeleton. Excludes features like eyes & ears.
const std::vector<std::pair<uint8_t, uint8_t>>& getSkeletonBones();
//! Returns the subset of joints used by getSkeletonBones(). Excludes features like eyes & ears.
const std::vector<uint8_t>& getSkeletonJoints();

//! Example: "SPINE_NAVEL" returns \c K4ABT_JOINT_SPINE_NAVEL=1. Returns \c K4ABT_JOINT_COUNT if not found. Case insensitive.
k4abt_joint_id_t jointNameToId( const std::string& name );
std::string jointIdToName( k4abt_joint_id_t joint );

} // namespace cika