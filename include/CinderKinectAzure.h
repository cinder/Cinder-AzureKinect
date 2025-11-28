#pragma once

#include <memory>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "cinder/Cinder.h"
#include "cinder/Exception.h"
#include "cinder/Surface.h"
#include "cinder/ImageIo.h"
#include "cinder/AxisAlignedBox.h"
#include "cinder/Camera.h"
#include "cinder/ConcurrentCircularBuffer.h"
#include "cinder/Timer.h"
#include "cinder/Signals.h"

#include <k4a/k4atypes.h>
#include <k4arecord/types.h>

namespace std {
class thread;
};

namespace cika {

enum class CoordinateSystem {
	DEFAULT, // Z forward, Y down, X Left. Default Kinect Azure.
	RIGHT_HANDED_Y_UP // Right-Handed with Y pointing up and Z backward. Standard OpenGL.
};

enum class Unit {
	MILLIMETER, // Default Kinect Azure.
	METER
};

struct ImuSample {
	glm::vec3				accelerometer; // accelerometer reading in Camera's CoordinateSystem. units are always m/s^2. Acceleration relative to a Kinect in freefall.
	glm::vec3				gyroscope; // gyroscope reading in Camera's CoordinateSystem. units are always radian/s
	k4a_imu_sample_t		raw;
};

class Device {
  public:
	//! Default device
	Device( uint8_t index = K4A_DEVICE_DEFAULT ) : mIndex( index ) {}
	Device( const std::string& serialNumber ) : mIndex( UNKOWN_DEVICE_INDEX ), mSerialNumber( serialNumber ) {}
	Device( uint8_t index, const std::string& serialNumber ) : mIndex( index ), mSerialNumber( serialNumber ) {}

	static uint32_t					getNumDevices();
	static std::vector<Device>		getDeviceList();

	uint8_t			getIndex() const;
	std::string		getSerialNumber() const;
	
  private:
	enum { UNKOWN_DEVICE_INDEX = 255 };

	mutable uint8_t			mIndex;
	mutable std::string		mSerialNumber;
};

class CaptureFormat {
  public:
	//! By default enabled color, depth, and IR capture
	CaptureFormat() : mColor( true ), mDepth( true ), mInfrared( true ), mImageColorGeomDepth( false ), mImageDepthGeomColor( false ), mPointsRgbGeomDepth( false ), mPointsRgbGeomColor( false ) {}
	CaptureFormat&		color( bool enable = true ) { mColor = enable; return *this; }
	CaptureFormat&		depth( bool enable = true ) { mDepth = enable; return *this; }
	CaptureFormat&		infrared( bool enable = true ) { mInfrared = enable; return *this; }
	//! Enables background thread to precompute results for getting color in depth camera geometry, accessible via getImageColorGeomDepth()
	CaptureFormat&		imageColorGeomDepth( bool enable = true ) { mImageColorGeomDepth = enable; return *this; }
	bool				getImageColorGeomDepth() const { return mImageColorGeomDepth; }
	//! Enables background thread to precompute results for getting depth in color camera geometry, accessible via getImageDepthGeomColor()
	CaptureFormat&		imageDepthGeomColor( bool enable = true ) { mImageDepthGeomColor = enable; return *this; }
	bool				getImageDepthGeomColor() const { return mImageDepthGeomColor; }
	//! Enables background thread to precompute 3D points with color in depth camera geometry, accessible via getPointsRgbGeomDepth()
	CaptureFormat&		pointsRgbGeomDepth( bool enable = true ) { mPointsRgbGeomDepth = enable; return *this; }
	bool				getPointsRgbGeomDepth() const { return mPointsRgbGeomDepth; }
	//! Enables background thread to precompute 3D points with color in color camera geometry, accessible via getPointsRgbGeomColor()
	CaptureFormat&		pointsRgbGeomColor( bool enable = true ) { mPointsRgbGeomColor = enable; return *this; }
	bool				getPointsRgbGeomColor() const { return mPointsRgbGeomColor; }

  private:
	bool			mColor, mDepth, mInfrared;
	bool			mImageColorGeomDepth, mImageDepthGeomColor;
	bool			mPointsRgbGeomDepth, mPointsRgbGeomColor;
};

class Camera {
  public:
	Camera( k4a_color_resolution_t resolution = K4A_COLOR_RESOLUTION_720P, k4a_depth_mode_t depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED, k4a_fps_t fps = K4A_FRAMES_PER_SECOND_30, const Device& device = Device( K4A_DEVICE_DEFAULT ) );
	//! Construct a Camera using a playback file
	Camera( std::filesystem::path &playbackPath, bool loop = true );
	//! Advanced use-case construction receiving a k4a_device_configuration_t directly
	Camera( k4a_device_configuration_t config, const Device& device = Device() );
	~Camera();

	void				setCoordinateSystem( CoordinateSystem system );
	CoordinateSystem	getCoordinateSystem() const { return mCoordinateSystem; }
	void				setUnits( Unit unit );
	Unit				getUnits() const { return mUnit; }

	bool				getColorEnabled() const { return mK4aCalibration.color_resolution != K4A_COLOR_RESOLUTION_OFF; }
	bool				getDepthEnabled() const { return mK4aCalibration.depth_mode != K4A_DEPTH_MODE_OFF; }

	//! Starts camera capture using a background thread which postprocesses captured images based on \a config
	void			startCapture( CaptureFormat config = CaptureFormat() );
	void			stopCapture();
	CaptureFormat	getCaptureConfig() const { return mCaptureConfig; }
	bool			isCapturing() const { return mCapturing; }

	std::string		getSerialNumber() const { return mSerialNumber; }

	//! Returns width in pixels of color stream
	int32_t		getColorWidth() const { return mK4aCalibration.color_camera_calibration.resolution_width; }
	//! Returns height in pixels of color stream
	int32_t		getColorHeight() const { return mK4aCalibration.color_camera_calibration.resolution_height; }
	//! Returns aspect ratio of color stream
	float		getColorAspectRatio() const { return getColorWidth() / (float)getColorHeight(); }
	//! Returns width in pixels of depth / IR stream
	int32_t		getDepthWidth() const { return mK4aCalibration.depth_camera_calibration.resolution_width; }
	//! Returns height in pixels of depth / IR stream
	int32_t		getDepthHeight() const { return mK4aCalibration.depth_camera_calibration.resolution_height; }
	//! Returns aspect ratio of depth / IR stream
	float			getDepthAspectRatio() const { return getDepthWidth() / (float)getDepthHeight(); }
	uint8_t		getFps() const;
	glm::ivec2	getColorSize() const { return glm::ivec2( mK4aCalibration.color_camera_calibration.resolution_width, mK4aCalibration.color_camera_calibration.resolution_height ); }
	glm::ivec2	getDepthSize() const { return glm::ivec2( mK4aCalibration.depth_camera_calibration.resolution_width, mK4aCalibration.depth_camera_calibration.resolution_height ); }

	//! Returns expected value range for the depth image based on the current depth mode
	ci::ivec2		getDepthRange() const;
	//! Returns expected value range for the infrared image based on the current depth mode
	ci::ivec2		getInfraredRange() const;
	ci::vec2		getColorFieldOfView() const;
	//! Returns field of view for the current depth mode in angles
	ci::vec2		getDepthFieldOfView() const;
	ci::CameraPersp	getDepthFrustum( float nearPlaneMeters, float farPlaneMeters, const glm::vec3& lookAt = glm::vec3{ 0, 0, -1 } ) const;
	ci::CameraPersp	getColorFrustum( float nearPlaneMeters, float farPlaneMeters ) const;

	//! Returns \c true if a new frame is available. If \a waitForFrame is \c true then the call doesn't return until a new frame has been acquired
	bool					update( bool waitForFrame = false );
	const ci::Surface8u&	getImageColor() const { return mCurrentCaptureData->mSurfaceColor; }
	const ci::Channel16u&	getImageInfrared() const { return mCurrentCaptureData->mChannelInfrared; }
	const ci::Channel16u&	getImageDepth() const { return mCurrentCaptureData->mChannelDepth; }
	//! Returns the color image warped into the geometry of the depth camera. Calculates the warp if in low latency mode or mot requested in CaptureFormat.
	const ci::Surface8u&	getImageColorGeomDepth() const;
	//! Returns the depth image warped into the geometry of the color camera.  Calculates the warp if in low latency mode or mot requested in CaptureFormat.
	const ci::Channel16u&	getImageDepthGeomColor() const;
	
	uint64_t				getTimestampColor() const { return mCurrentCaptureData->mTimestampColor; }
	double					getTimestampColorSeconds() const { return mCurrentCaptureData->mTimestampColor / 1000000.; }

	// Writes to \a dest the depth data as a point cloud with RGB color in depth camera geometry. Returns the number of points returned to \a dest. Data is [X:f32 Y:f32 Z:f32 R:f32 G:f32 B:f32] interleaved.
	uint32_t			getPointsRgbGeomDepth( float *dest, bool finitePointsOnly = false, cinder::AxisAlignedBox *extents = nullptr );
	// Writes to \a dest the depth data as a point cloud with RGB color in color camera geometry. Returns the number of points returned to \a dest. Data is [X:f32 Y:f32 Z:f32 R:f32 G:f32 B:f32] interleaved.
	uint32_t			getPointsRgbGeomColor( float *dest, bool finitePointsOnly = false, cinder::AxisAlignedBox *extents = nullptr );

	//! Writes to \a dest the depth data as a point cloud with RGB color in depth camera geometry. Expects RGB or RGBX channel order. Returns the number of points returned to \a dest. Data is [X:f32 Y:f32 Z:f32 R:f32 G:f32 B:f32] interleaved.
	uint32_t			transformPointsRgbGeomDepth( const ci::Surface8u& colorGeomDepth, const ci::Channel16u& depth, float *dest, bool finitePointsOnly = false ) const;

	//! Returns an image which can be indexed in 2D and multiplied by a point's depth to get its position in 3D in the depth camera's geometry
	const ci::Surface32f&	getDepthTableGeomDepth() const { calcDepthTableGeomDepth(); return *mDepthTableGeomDepth; }
	//! Returns an image which can be indexed in 2D and multiplied by a point's depth to get its position in 3D in the color camera's geometry
	const ci::Surface32f&	getDepthTableGeomColor() const { calcDepthTableGeomColor(); return *mDepthTableGeomColor; }
	//! Returns the 2D point in the color camera's geometry from a 3D point in the depth camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec2			depth3dToColor2d( glm::vec3 pt, bool *valid = nullptr ) const;
	//! Returns the 2D point in the depth camera's geometry from a 3D point in the depth camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec2			depth3dToDepth2d( glm::vec3 pt, bool *valid = nullptr ) const;
	//! Returns the 2D point in the depth camera's geometry from a 2D point in the color camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec2			color2dToDepth2d( glm::ivec2 colorPt, bool *valid = nullptr ) const;
	//! Returns the 2D point in the colora camera's geometry from a 2D point in the depth camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec2			depth2dToColor2d( glm::ivec2 depthPt, bool *valid = nullptr ) const;
	//! Returns the 3D point in the depth camera's geometry from a 2D point in the depth camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec3			depth2dToDepth3d( glm::ivec2 depthPt, bool *valid = nullptr ) const;
	//! Returns the 3D point in the color camera's geometry from a 2D point in the depth camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec3			depth2dToColor3d( glm::ivec2 depthPt, bool *valid = nullptr ) const;
	//! Returns the 3D point in the color camera's geometry from a 2D point in the color camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec3			color2dToColor3d( glm::ivec2 depthPt, bool *valid = nullptr ) const;
	//! Returns the 3D point in the depth camera's geometry from a 2D point in the color camera's geometry. If \a valid is not null, set to whether the projection is valid
	glm::vec3			color2dToDepth3d( glm::ivec2 depthPt, bool *valid = nullptr ) const;

	// Playback
	bool				isPlayingRecording() const { return mK4aPlayback != nullptr; }
	//! Returns timestamp representing the duration of the recording, in the recording's own timescale (microseconds)
	uint64_t			getPlaybackTimestamp() const { return mPlaybackDurationTimestamp; }
	//! Returns timestamp representing the duration of the recording in seconds
	double				getPlaybackDurationSeconds() const { return mPlaybackDurationTimestamp / 1000000.; }

	// IMU
	//! Starts the camera's IMU sample stream
	void				startImu();
	//! Stops the camera's IMU sample stream
	void				stopImu();
	//! Updates the current IMU sample. Returns \c false if no sample was updated. Can be retrieved with getImuSample().
	bool				updateImu( bool waitForSample = false );
	//! Returns the latest IMU sample captured by updateImu().
	ImuSample			getImuSample() const { return mImuSample; }

	//! Returns scale conversion from Kinect Azure-native millimeters to the current coordinate units
	float				scaleFromMm() const;
	k4a_calibration_t	getCalibration() const { return mK4aCalibration; }

	ci::signals::Signal<void(k4a_capture_t)>&	getSignalNewCapture() { return mSignalNewCapture; }

  private:
	void initDevice( k4a_device_configuration_t config, uint8_t deviceIndex );
	void initTransformAndBuffers();
	void calcDepthTableGeomDepth() const;
	void calcDepthTableGeomColor() const;
	// Represents the images of a single capture operation and any postprocessed images
	class CaptureData {
	  public:
		CaptureData( k4a_device_t device ) : mK4aDevice( device ) {}
		~CaptureData();

		bool		acquire( k4a_capture_t capture, bool synchronizedImages );
		void		release();

		k4a_device_t			mK4aDevice;
		k4a_capture_t			mK4aCapture = nullptr;
		k4a_image_t				mK4aImageColor = nullptr, mK4aImageInfrared = nullptr, mK4aImageDepth = nullptr;
		
		ci::Surface8u			mSurfaceColor;
		ci::Channel16u			mChannelInfrared, mChannelDepth;
		
		bool					mSurfaceColorGeomDepthIsValid = false, mChannelDepthGeomColorIsValid = false;
		ci::Surface8u			mSurfaceColorGeomDepth;
		ci::Channel16u			mChannelDepthGeomColor;

		bool					mPointsGeomDepthIsValid = false, mPointsGeomColorIsValid = false;
		mutable k4a_image_t		mK4aImageColorGeomDepth = nullptr, mK4aImageDepthGeomColor = nullptr;
		mutable k4a_image_t		k4aImagePointsGeomDepth = nullptr, k4aImagePointsGeomColor = nullptr;

		bool						mPointsRgbGeomDepthIsValid = false, mPointsRgbGeomColorIsValid = false;
		std::unique_ptr<float[]>	mPointsRgbGeomDepth, mPointsRgbGeomColor;

		uint64_t				mTimestampColor;
	};

	void			captureThreadFn();
	void			updateColorGeomDepth( CaptureData *imageSet ) const;
	void			updateDepthGeomColor( CaptureData *imageSet ) const;
	void			updatePointsGeomDepth( CaptureData *imageSet ) const;
	void			updatePointsGeomColor( CaptureData *imageSet ) const;
	void			updatePointsRgbGeomDepth( CaptureData *imageSet ) const;
	void			updatePointsRgbGeomColor( CaptureData *imageSet ) const;
	glm::vec2		point3dTo2dImpl( k4a_calibration_type_t srcGeom, k4a_calibration_type_t dstGeom, glm::vec3 pt, bool *valid ) const;
	glm::vec3		point2dTo3dImpl( k4a_calibration_type_t srcGeom, k4a_calibration_type_t dstGeom, glm::ivec2 pt, float depthMm, bool *valid ) const;

	k4a_device_t				mK4aDevice = nullptr;
	bool						mCapturing = false;
	std::atomic<bool>			mStoppingCapture = false;
	CaptureFormat				mCaptureConfig;
	std::string					mSerialNumber;
	k4a_device_configuration_t	mK4aConfig;
	k4a_calibration_t			mK4aCalibration;
	k4a_transformation_t		mK4aTransformation = nullptr;

	mutable std::unique_ptr<ci::Surface32f>		mDepthTableGeomColor, mDepthTableGeomDepth;

	cinder::signals::Signal<void(k4a_capture_t)>	mSignalNewCapture;

	CoordinateSystem			mCoordinateSystem = CoordinateSystem::DEFAULT;
	Unit						mUnit = Unit::MILLIMETER;

	// Playback
	k4a_playback_t				mK4aPlayback = nullptr;
	cinder::Timer				mPlaybackTimer; // for enforcing framerate
	double						mPlaybackTimerLastFrameTime;
	std::filesystem::path		mPlaybackPath;
	bool						mPlaybackLoop;
	uint64_t					mPlaybackDurationTimestamp;

	// IMU
	ImuSample					mImuSample;

	// Threading
	std::unique_ptr<std::thread>					mCaptureThread;
	std::vector<std::unique_ptr<CaptureData>>		mCaptureDataStorage;
	ci::ConcurrentCircularBuffer<CaptureData*>		mUnusedCaptureDataCcb;
	ci::ConcurrentCircularBuffer<CaptureData*>		mReadyCaptureDataCcb;
	CaptureData*									mCurrentCaptureData;
};

//! Sets minimum logging message level, mapped into Cinder's logging system.
void setLogMessageLevel( k4a_log_level_t level );

glm::ivec2	getDepthRange( k4a_depth_mode_t mode );
glm::ivec2	getInfraredRange( k4a_depth_mode_t mode );
glm::vec2	getDepthFieldOfView( k4a_depth_mode_t depthMode );
glm::vec2	getColorFieldOfView( k4a_color_resolution_t mode );
glm::ivec2	getColorResolution( k4a_color_resolution_t mode );
ci::CameraPersp	getDepthFrustum( k4a_depth_mode_t depthMode, float nearPlaneMeters, float farPlaneMeters, const glm::vec3& lookAt = glm::vec3{ 0, 0, -1 } );


class Exception : public ci::Exception {
  protected:
	Exception()
		: ci::Exception() {}
};

class ExceptionDeviceOpen : public Exception {
  public:
	ExceptionDeviceOpen() : Exception() {}
};

class ExceptionCreation : public Exception {
  public:
	ExceptionCreation() : Exception() {}
};

class ExceptionStart : public Exception {
  public:
	ExceptionStart() : Exception() {}
};

class ExceptionGetImage : public Exception {
  public:
	ExceptionGetImage() : Exception() {}
};

class ExceptionTracker : public Exception {
  public:
	ExceptionTracker() : Exception() {}
};

class ExceptionProjection : public Exception {
  public:
	ExceptionProjection() : Exception() {}
};

class ExceptionImu: public Exception {
  public:
	ExceptionImu() : Exception() {}
};

} //  namespace cika
