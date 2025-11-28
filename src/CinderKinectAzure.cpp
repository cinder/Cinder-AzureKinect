#include "CinderKinectAzure.h"

#include "cinder/Log.h"
#include "cinder/ImageIo.h"
#include "cinder/Timer.h"
#include "cinder/Utilities.h"

#include <memory>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

using namespace std;
using namespace ci;

///////////////////////////////////////////////////////////////////////////////////
namespace cika {

namespace {

static std::vector<uint8_t> sFrameRates = { 5 /*K4A_FRAMES_PER_SECOND_5*/, 15 /*K4A_FRAMES_PER_SECOND_15*/, 30 /*K4A_FRAMES_PER_SECOND_30*/ };

std::string getDeviceSerialNumber( k4a_device_t device )
{
	size_t serialNumberLength = 0;
	if( K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum( device, nullptr, &serialNumberLength ) )
		return std::string();

	std::unique_ptr<char[]> serialNumber( new char[serialNumberLength] );
	if( serialNumber == nullptr )
		return std::string();

	if( K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum( device, serialNumber.get(), &serialNumberLength ) )
		return std::string();

	return std::string( serialNumber.get() );
}
} // anonyous NS

uint32_t Device::getNumDevices()
{
	return k4a_device_get_installed_count();
}

std::vector<Device> Device::getDeviceList()
{
	std::vector<Device> result;

	k4a_device_t device = nullptr;
	uint32_t device_count = k4a_device_get_installed_count();
	
	for( uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++ ) {
		if( K4A_RESULT_SUCCEEDED != k4a_device_open( deviceIndex, &device ) ) // fails if a device is already open
			continue;

		std::string serialNumber = getDeviceSerialNumber( device );
		if( serialNumber.empty() ) {
			k4a_device_close( device );
			throw ExceptionDeviceOpen();
		}

		k4a_device_close( device );
		result.emplace_back( deviceIndex, serialNumber );
	}

	return result;
}

uint8_t Device::getIndex() const
{
	if( mIndex == UNKOWN_DEVICE_INDEX ) { // if we were instantiated with a serial, we won't know the device index
		auto devices = getDeviceList();
		for( uint8_t deviceIndex = 0; deviceIndex < (uint8_t)devices.size(); ++deviceIndex ) {
			if( devices[deviceIndex].mSerialNumber == mSerialNumber ) {
				mIndex = devices[deviceIndex].mIndex;
				break;
			}
		}
	}

	return mIndex;
}

std::string	Device::getSerialNumber() const
{
	if( mSerialNumber.empty() ) { // if we were instantiated with a device index, we won't know the serial number yet
		k4a_device_t device = nullptr;
		if( K4A_RESULT_SUCCEEDED != k4a_device_open( mIndex, &device ) )
			throw ExceptionDeviceOpen();
		mSerialNumber = getDeviceSerialNumber( device );
		k4a_device_close( device );
		if( mSerialNumber.empty() ) {
			throw ExceptionDeviceOpen();
		}
	}

	return mSerialNumber;
}

Camera::Camera( k4a_color_resolution_t resolution, k4a_depth_mode_t depthMode, k4a_fps_t fps, const Device& device )
	: mSerialNumber( device.getSerialNumber() ), mUnusedCaptureDataCcb( 2 ), mReadyCaptureDataCcb( 2 )
{
	if( (resolution == K4A_COLOR_RESOLUTION_3072P || depthMode == K4A_DEPTH_MODE_WFOV_UNBINNED) && fps == K4A_FRAMES_PER_SECOND_30 ) {
		CI_LOG_E( "Selected resolution does not support 30 FPS. Forcing 15 FPS." );
		fps = K4A_FRAMES_PER_SECOND_15;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = resolution;
	config.depth_mode = depthMode;
	config.camera_fps = fps;
	config.synchronized_images_only = ( resolution != K4A_COLOR_RESOLUTION_OFF && depthMode != K4A_DEPTH_MODE_OFF );

	initDevice( config, device.getIndex() );
	initTransformAndBuffers();
}

Camera::Camera( k4a_device_configuration_t config, const Device& device )
	: mSerialNumber( device.getSerialNumber() ), mUnusedCaptureDataCcb( 2 ), mReadyCaptureDataCcb( 2 )
{
	initDevice( config, device.getIndex() );
	initTransformAndBuffers();
}

// Playback constructor
Camera::Camera( std::filesystem::path &playbackPath, bool loop )
	: mPlaybackPath( playbackPath ), mPlaybackLoop( loop ), mUnusedCaptureDataCcb( 2 ), mReadyCaptureDataCcb( 2 )
{
	if( K4A_RESULT_SUCCEEDED != k4a_playback_open( playbackPath.string().c_str(), &mK4aPlayback ) )
		throw ExceptionCreation();
	
	if( K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration( mK4aPlayback, &mK4aCalibration ) )
		throw ExceptionCreation();

	if( K4A_RESULT_SUCCEEDED != k4a_playback_set_color_conversion( mK4aPlayback, K4A_IMAGE_FORMAT_COLOR_BGRA32 ) )
		throw ExceptionCreation();

	mPlaybackDurationTimestamp = k4a_playback_get_recording_length_usec( mK4aPlayback );

	char serial[1024];
	size_t sz = sizeof(serial);
	if( K4A_RESULT_SUCCEEDED == k4a_playback_get_tag( mK4aPlayback, "K4A_DEVICE_SERIAL_NUMBER", serial, &sz ) )
		mSerialNumber = std::string( serial );

	k4a_record_configuration_t recordConfiguration;
	if( K4A_RESULT_SUCCEEDED != k4a_playback_get_record_configuration( mK4aPlayback, &recordConfiguration ) )
		throw ExceptionCreation();
	mK4aConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	mK4aConfig.color_format = recordConfiguration.color_format;
	mK4aConfig.depth_mode = recordConfiguration.depth_mode;
	mK4aConfig.color_resolution = recordConfiguration.color_resolution;
	mK4aConfig.camera_fps = recordConfiguration.camera_fps;
	mK4aConfig.synchronized_images_only = ( mK4aConfig.color_resolution != K4A_COLOR_RESOLUTION_OFF && mK4aConfig.depth_mode != K4A_DEPTH_MODE_OFF );

	initTransformAndBuffers();
}

void Camera::initDevice( k4a_device_configuration_t config, uint8_t deviceIndex )
{
	mK4aConfig = config;

	if( K4A_RESULT_SUCCEEDED != k4a_device_open( deviceIndex, &mK4aDevice ) )
		throw ExceptionCreation();

	if( K4A_RESULT_SUCCEEDED != k4a_device_get_calibration( mK4aDevice, mK4aConfig.depth_mode, mK4aConfig.color_resolution, &mK4aCalibration ) )
		throw ExceptionStart();
}

void Camera::initTransformAndBuffers()
{
	mK4aTransformation = k4a_transformation_create( &mK4aCalibration );

	for( size_t i = 0; i < 2; ++i ) {
		mCaptureDataStorage.emplace_back( new CaptureData( mK4aDevice ) );
		mUnusedCaptureDataCcb.pushFront( mCaptureDataStorage.back().get() );
	}
	
	mUnusedCaptureDataCcb.popBack( &mCurrentCaptureData );
}

Camera::~Camera()
{
	mStoppingCapture = true;
	stopCapture();
	mUnusedCaptureDataCcb.cancel();
	mReadyCaptureDataCcb.cancel();
	if( mCaptureThread )
		mCaptureThread->join();

	mCaptureDataStorage.clear();

	if( mK4aDevice )
		k4a_device_close( mK4aDevice );
	if( mK4aTransformation )
		k4a_transformation_destroy( mK4aTransformation );
	if( mK4aPlayback )
		k4a_playback_close( mK4aPlayback );
}

void Camera::setCoordinateSystem( CoordinateSystem system )
{
	mCoordinateSystem = system;
	mDepthTableGeomDepth.reset();
	mDepthTableGeomColor.reset();
}

void Camera::setUnits( Unit unit )
{
	mUnit = unit;
	mDepthTableGeomDepth.reset();
	mDepthTableGeomColor.reset();
}


void Camera::startCapture( CaptureFormat captureConfig )
{
	if( mCapturing ) {
		CI_LOG_E("Already capturing. Ignoring startCapture()");
		return;
	}

	mCaptureConfig = captureConfig;

	// playback is handled
	if( ! mK4aPlayback ) {
		if( K4A_RESULT_SUCCEEDED != k4a_device_start_cameras( mK4aDevice, &mK4aConfig ) )
			throw ExceptionStart();
	}	
	mCapturing = true;

	mCaptureThread = std::make_unique<thread>( &Camera::captureThreadFn, this );
}

void Camera::stopCapture()
{
	if( ! mCapturing )
		return;

	k4a_device_stop_cameras( mK4aDevice );
	mCapturing = false;
}

void Camera::captureThreadFn()
{
	mPlaybackTimer.start();
	mPlaybackTimerLastFrameTime = -1;
	const double playbackFrameDuration = 1.0 / getFps();

	while( true ) {
		CaptureData *newImageSet;
		mUnusedCaptureDataCcb.popBack( &newImageSet );
		if( mStoppingCapture )
			return;
		newImageSet->release();

		while( true ) {
			try {
				k4a_capture_t capture = nullptr;
				if( ! mK4aPlayback )
					k4a_device_get_capture( mK4aDevice, &capture, K4A_WAIT_INFINITE );
				else { // playback frame
					while( mPlaybackTimer.getSeconds() - mPlaybackTimerLastFrameTime < playbackFrameDuration )
						std::this_thread::sleep_for( chrono::microseconds( 1000 ) ); // this could be made to optionally spin-wait or using an OS-specific high resolution timer
					mPlaybackTimerLastFrameTime = mPlaybackTimer.getSeconds();
					k4a_stream_result_t result = k4a_playback_get_next_capture( mK4aPlayback, &capture );
					if( result == K4A_STREAM_RESULT_EOF && mPlaybackLoop )
						k4a_playback_seek_timestamp( mK4aPlayback, 0, K4A_PLAYBACK_SEEK_BEGIN );
				}
				if( capture ) {
					if( newImageSet->acquire( capture, mK4aConfig.synchronized_images_only ) ) {
						// postprocessing
						if( mCaptureConfig.getImageColorGeomDepth() )
							updateColorGeomDepth( newImageSet );
						if( mCaptureConfig.getImageDepthGeomColor() )
							updateDepthGeomColor( newImageSet );
						if( mCaptureConfig.getPointsRgbGeomDepth() )
							updatePointsRgbGeomDepth( newImageSet );
						if( mCaptureConfig.getPointsRgbGeomColor() )
							updatePointsRgbGeomColor( newImageSet );

						mSignalNewCapture.emit( capture );

						mReadyCaptureDataCcb.pushFront( newImageSet );
						break;
					}
					else // this will fire if acquire() failed, which should only happen if we're simulating synchronized_images_only for playback
						k4a_capture_release( capture );
				}
			
				if( mStoppingCapture )
					return;
			}
			catch(...) {
				CI_LOG_E( "Uncaught exception on Kinect thread fn" );
			}
		}
	}
}

uint8_t Camera::getFps() const
{
	return sFrameRates[(int)mK4aConfig.camera_fps];
}

glm::vec2	getColorFieldOfView( k4a_color_resolution_t mode );
glm::ivec2	getColorResolution( k4a_color_resolution_t mode );

ivec2 Camera::getDepthRange() const
{
	return cika::getDepthRange( mK4aCalibration.depth_mode );
}

float getDepthAspectRatio( k4a_depth_mode_t mode )
{
	 switch( mode ) {
		case K4A_DEPTH_MODE_NFOV_2X2BINNED: return 320 / 288.0f;
		case K4A_DEPTH_MODE_NFOV_UNBINNED: return 640 / 576.0f;
		case K4A_DEPTH_MODE_WFOV_2X2BINNED: return 512 / 512.0f;
		case K4A_DEPTH_MODE_WFOV_UNBINNED: return 1024 / 1024.0f;
		case K4A_DEPTH_MODE_PASSIVE_IR:
		default: return 1.0f;
	}
}

ivec2 getDepthRange( k4a_depth_mode_t mode )
{
	 switch( mode ) {
		case K4A_DEPTH_MODE_NFOV_2X2BINNED: return { 500, 5800 };
		case K4A_DEPTH_MODE_NFOV_UNBINNED: return { 500, 4000 };
		case K4A_DEPTH_MODE_WFOV_2X2BINNED: return { 250, 3000 };
		case K4A_DEPTH_MODE_WFOV_UNBINNED: return { 250, 2500 };
		case K4A_DEPTH_MODE_PASSIVE_IR:
		default: return { 0, 0 };
	}
}

ivec2 Camera::getInfraredRange() const
{
	return cika::getInfraredRange( mK4aCalibration.depth_mode );
}

ivec2 getInfraredRange( k4a_depth_mode_t mode )
{
	switch( mode ) {
		case K4A_DEPTH_MODE_PASSIVE_IR: return { 0, 100 };
		case K4A_DEPTH_MODE_OFF: return { 0, 0 };
		default: return { 0, 1000 };
	}
}

vec2 Camera::getColorFieldOfView() const
{
	return cika::getColorFieldOfView( mK4aCalibration.color_resolution );
}

vec2 getColorFieldOfView( k4a_color_resolution_t mode )
{
	 switch( mode ) {
		case K4A_COLOR_RESOLUTION_720P: case K4A_COLOR_RESOLUTION_1080P: case K4A_COLOR_RESOLUTION_1440P: case K4A_COLOR_RESOLUTION_2160P: return { 90.0f, 59.0f };
		case K4A_COLOR_RESOLUTION_1536P: case K4A_COLOR_RESOLUTION_3072P: return { 90.0f, 74.3f };
	}
	return {0, 0};
}

ivec2 getColorResolution( k4a_color_resolution_t mode )
{
	 switch( mode ) {
		case K4A_COLOR_RESOLUTION_720P: return{ 1280, 720 };
		case K4A_COLOR_RESOLUTION_1080P: return{ 1920, 1080 };
		case K4A_COLOR_RESOLUTION_1440P: return{ 2560, 1440 };
		case K4A_COLOR_RESOLUTION_1536P: return{ 2048, 1536 };
		case K4A_COLOR_RESOLUTION_2160P: return{ 3840, 2160 };
		case K4A_COLOR_RESOLUTION_3072P: return{ 4096, 3072 };
	}
	return {0, 0};
}

vec2 getDepthFieldOfView( k4a_depth_mode_t depthMode )
{
	 switch( depthMode ) {
		case K4A_DEPTH_MODE_NFOV_2X2BINNED: case K4A_DEPTH_MODE_NFOV_UNBINNED: return { 75.0f, 65.0f };
		case K4A_DEPTH_MODE_WFOV_2X2BINNED: case K4A_DEPTH_MODE_WFOV_UNBINNED: case K4A_DEPTH_MODE_PASSIVE_IR: return { 120.0f, 120.0f };
	}
	return {0, 0};
}

vec2 Camera::getDepthFieldOfView() const
{
	return cika::getDepthFieldOfView( mK4aCalibration.depth_mode );
}

ci::CameraPersp Camera::getColorFrustum( float nearPlaneMeters, float farPlaneMeters ) const
{
	CameraPersp result;
	float unitScale = 1.0f;
	result.setEyePoint( vec3(0, 0, 0) );
	result.lookAt( vec3(0, 0, -1) );
	result.setPerspective( getColorFieldOfView().y, getColorAspectRatio(), unitScale * nearPlaneMeters, unitScale * farPlaneMeters );
	return result;
}

ci::CameraPersp Camera::getDepthFrustum( float nearPlaneMeters, float farPlaneMeters, const vec3 &lookAt ) const
{
	CameraPersp result;
	float unitScale = 1.0f;
	result.setEyePoint( vec3(0, 0, 0) );
	result.lookAt( lookAt );
	result.setPerspective( getDepthFieldOfView().y, getDepthAspectRatio(), unitScale * nearPlaneMeters, unitScale * farPlaneMeters );
	return result;
}

ci::CameraPersp getDepthFrustum( k4a_depth_mode_t depthMode, float nearPlaneMeters, float farPlaneMeters, const glm::vec3& lookAt )
{
	CameraPersp result;
	float unitScale = 1.0f;
	result.setEyePoint( vec3(0, 0, 0) );
	result.lookAt( lookAt );
	result.setPerspective( getDepthFieldOfView( depthMode ).y, getDepthAspectRatio( depthMode ), unitScale * nearPlaneMeters, unitScale * farPlaneMeters );
	return result;
}

bool Camera::update( bool waitForFrame )
{
	if( waitForFrame ) {
		mUnusedCaptureDataCcb.pushFront( mCurrentCaptureData );
		mReadyCaptureDataCcb.popBack( &mCurrentCaptureData );
	}
	else {
		CaptureData *newImageSet;
		if( mReadyCaptureDataCcb.tryPopBack( &newImageSet ) ) {
			mUnusedCaptureDataCcb.pushFront( mCurrentCaptureData );
			mCurrentCaptureData = newImageSet;
		}
		else
			return false;
	}

	return ! mStoppingCapture;
}

//! If \a synchronizedImages, will return false if not both depth and color are present
bool Camera::CaptureData::acquire( k4a_capture_t capture, bool synchronizedImages )
{
	mK4aCapture = capture;

	mK4aImageColor = k4a_capture_get_color_image( mK4aCapture );
	mK4aImageInfrared = k4a_capture_get_ir_image( mK4aCapture );
	mK4aImageDepth = k4a_capture_get_depth_image( mK4aCapture );

	if( synchronizedImages && ( ! mK4aImageColor || ! mK4aImageDepth ) )
		return false;
	
	if( mK4aImageColor ) {
		mSurfaceColor = Surface8u( k4a_image_get_buffer( mK4aImageColor ), k4a_image_get_width_pixels( mK4aImageColor ),
			k4a_image_get_height_pixels( mK4aImageColor ), k4a_image_get_stride_bytes( mK4aImageColor ), SurfaceChannelOrder::BGRX );
		mTimestampColor = k4a_image_get_device_timestamp_usec( mK4aImageColor );
	}

	if( mK4aImageInfrared )
		mChannelInfrared = ci::Channel16u( k4a_image_get_width_pixels( mK4aImageInfrared ), k4a_image_get_height_pixels( mK4aImageInfrared ),
				k4a_image_get_stride_bytes( mK4aImageInfrared ), 1, (uint16_t*)k4a_image_get_buffer( mK4aImageInfrared ) );

	if( mK4aImageDepth )
		mChannelDepth = ci::Channel16u( k4a_image_get_width_pixels( mK4aImageDepth ), k4a_image_get_height_pixels( mK4aImageDepth ),
				k4a_image_get_stride_bytes( mK4aImageDepth ), 1, (uint16_t*)k4a_image_get_buffer( mK4aImageDepth ) );

	return true;
}

void Camera::CaptureData::release()
{
	if( mK4aCapture ) {
		k4a_capture_release( mK4aCapture );
		mK4aCapture = nullptr;
	}

	if( mK4aImageColor )
		k4a_image_release( mK4aImageColor );
	if( mK4aImageDepth )
		k4a_image_release( mK4aImageDepth );
	if( mK4aImageInfrared )
		k4a_image_release( mK4aImageInfrared );

	mK4aImageColor = mK4aImageDepth = mK4aImageInfrared = nullptr;
	mSurfaceColorGeomDepthIsValid = false;
	mChannelDepthGeomColorIsValid = false;
	mChannelDepthGeomColorIsValid = false;
	mPointsGeomDepthIsValid = false;
	mPointsGeomColorIsValid = false;
	mPointsRgbGeomDepthIsValid = false;
	mPointsRgbGeomColorIsValid = false;
	mTimestampColor = 0;
}

Camera::CaptureData::~CaptureData()
{
	release();

	if( mK4aImageColorGeomDepth )
		k4a_image_release( mK4aImageColorGeomDepth );
	if( mK4aImageDepthGeomColor )
		k4a_image_release( mK4aImageDepthGeomColor );
	if( k4aImagePointsGeomDepth )
		k4a_image_release( k4aImagePointsGeomDepth );
	if( k4aImagePointsGeomColor )
		k4a_image_release( k4aImagePointsGeomColor );
}

const ci::Surface8u& Camera::getImageColorGeomDepth() const
{
	updateColorGeomDepth( const_cast<CaptureData*>( mCurrentCaptureData ) );

	return mCurrentCaptureData->mSurfaceColorGeomDepth;
}

const ci::Channel16u& Camera::getImageDepthGeomColor() const
{
	updateDepthGeomColor( const_cast<CaptureData*>( mCurrentCaptureData ) );

	return mCurrentCaptureData->mChannelDepthGeomColor;
}

void Camera::updateColorGeomDepth( CaptureData *imageSet ) const
{
	if( imageSet->mSurfaceColorGeomDepthIsValid )
		return;

	// Requires a valid color and depth image
	if( ! imageSet->mK4aImageColor || ! imageSet->mK4aImageDepth )
		return;

	// allocate the k4a_image_t if this CaptureData doesn't already have one
	if( ! imageSet->mK4aImageColorGeomDepth ) {
		if( K4A_RESULT_SUCCEEDED != k4a_image_create( K4A_IMAGE_FORMAT_COLOR_BGRA32, k4a_image_get_width_pixels( imageSet->mK4aImageDepth ),
														k4a_image_get_height_pixels( imageSet->mK4aImageDepth ), k4a_image_get_width_pixels( imageSet->mK4aImageDepth ) * 4 * (int)sizeof(uint8_t),
														&imageSet->mK4aImageColorGeomDepth) ) {
			throw ExceptionGetImage();
		}
	}

	// warp color image into geometry of depth camera
	if( K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera( mK4aTransformation, imageSet->mK4aImageDepth, imageSet->mK4aImageColor, imageSet->mK4aImageColorGeomDepth))
		throw ExceptionGetImage();

	// wrap with a surface
	imageSet->mSurfaceColorGeomDepth = Surface8u( k4a_image_get_buffer( imageSet->mK4aImageColorGeomDepth ), k4a_image_get_width_pixels( imageSet->mK4aImageColorGeomDepth ),
		k4a_image_get_height_pixels( imageSet->mK4aImageColorGeomDepth ), k4a_image_get_stride_bytes( imageSet->mK4aImageColorGeomDepth ), SurfaceChannelOrder::BGRX );	

	imageSet->mSurfaceColorGeomDepthIsValid = true;
}

void Camera::updateDepthGeomColor( CaptureData *imageSet ) const
{
	if( imageSet->mChannelDepthGeomColorIsValid )
		return;

	// Requires a valid color and depth image
	if( ! imageSet->mK4aImageColor || ! imageSet->mK4aImageDepth )
		return;

	// allocate the k4a_image_t if this CaptureData doesn't already have one
	if( ! imageSet->mK4aImageDepthGeomColor ) {
		if( K4A_RESULT_SUCCEEDED != k4a_image_create( K4A_IMAGE_FORMAT_DEPTH16, k4a_image_get_width_pixels( imageSet->mK4aImageColor ),
														k4a_image_get_height_pixels( imageSet->mK4aImageColor ), k4a_image_get_width_pixels( imageSet->mK4aImageColor ) * (int)sizeof(uint16_t),
														&imageSet->mK4aImageDepthGeomColor) ) {
			throw ExceptionGetImage();
		}
	}

	// warp color image into geometry of depth camera
	if( K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera( mK4aTransformation, imageSet->mK4aImageDepth, imageSet->mK4aImageDepthGeomColor ) )
		throw ExceptionGetImage();

	// wrap with a channel
	imageSet->mChannelDepthGeomColor = ci::Channel16u( k4a_image_get_width_pixels( imageSet->mK4aImageDepthGeomColor ), k4a_image_get_height_pixels( imageSet->mK4aImageDepthGeomColor ),
				k4a_image_get_stride_bytes( imageSet->mK4aImageDepthGeomColor ), 1, (uint16_t*)k4a_image_get_buffer( imageSet->mK4aImageDepthGeomColor ) );

	imageSet->mChannelDepthGeomColorIsValid = true;
}

void Camera::updatePointsRgbGeomDepth( CaptureData *imageSet ) const
{
	if( imageSet->mPointsRgbGeomDepthIsValid )
		return;

	calcDepthTableGeomDepth();
	updateColorGeomDepth( imageSet );

	int32_t width = imageSet->mChannelDepth.getWidth();
	int32_t height = imageSet->mChannelDepth.getHeight();
	uint32_t numPoints = width * height, resultPoints = 0;

	if( ! imageSet->mPointsRgbGeomDepth )
		imageSet->mPointsRgbGeomDepth = std::unique_ptr<float[]>( new float[6 * width * height] );

	float *dest = imageSet->mPointsRgbGeomDepth.get();

	size_t p = 0; // offset within points
	for( size_t y = 0; y < height; ++y ) {
		const uint8_t *colors = imageSet->mSurfaceColorGeomDepth.getData( ivec2( 0, y ) );
		const uint16_t *depthPtr = imageSet->mChannelDepth.getData( ivec2( 0, y ) );
		const Colorf *depthTableRow = (Colorf*)mDepthTableGeomDepth->getData( ivec2( 0, y ) );
		for( size_t x = 0; x < width; ++x ) {
			float depth = (float)depthPtr[x];
			Colorf tableEntry = depthTableRow[x];
			vec3 v( tableEntry.r * depth, -tableEntry.g * depth, tableEntry.b * depth );
			dest[0] = v.x; dest[1] = v.y; dest[2] = v.z;
			dest[3] = colors[x*4+2] * 0.00392156f; dest[4] = colors[x*4+1] * 0.00392156f; dest[5] = colors[x*4+0] * 0.00392156f;
			dest += 6;
			++p;
		}
	}

	imageSet->mPointsRgbGeomDepthIsValid = true;
}

void Camera::updatePointsRgbGeomColor( CaptureData *imageSet ) const
{
	if( imageSet->mPointsRgbGeomColorIsValid )
		return;

	calcDepthTableGeomColor();
	updateDepthGeomColor( imageSet );

	int32_t width = imageSet->mSurfaceColor.getWidth();
	int32_t height = imageSet->mSurfaceColor.getHeight();
	uint32_t numPoints = width * height, resultPoints = 0;

	if( ! imageSet->mPointsRgbGeomColor )
		imageSet->mPointsRgbGeomColor = std::unique_ptr<float[]>( new float[6 * width * height] );

	float *dest = imageSet->mPointsRgbGeomColor.get();

	size_t p = 0; // offset within points
	for( size_t y = 0; y < height; ++y ) {
		const uint8_t *colors = imageSet->mSurfaceColor.getData( ivec2( 0, y ) );
		const uint16_t *depthPtr = imageSet->mChannelDepthGeomColor.getData( ivec2( 0, y ) );
		const Colorf *depthTableRow = (Colorf*)mDepthTableGeomColor->getData( ivec2( 0, y ) );
		for( size_t x = 0; x < width; ++x ) {
			float depth = (float)depthPtr[x];
			Colorf tableEntry = depthTableRow[x];
			vec3 v( tableEntry.r * depth, -tableEntry.g * depth, tableEntry.b * depth );
			dest[0] = v.x; dest[1] = v.y; dest[2] = v.z;
			dest[3] = colors[x*4+2] * 0.00392156f; dest[4] = colors[x*4+1] * 0.00392156f; dest[5] = colors[x*4+0] * 0.00392156f;
			dest += 6;
			++p;
		}
	}

	imageSet->mPointsRgbGeomColorIsValid = true;
}

uint32_t Camera::getPointsRgbGeomDepth( float *dest, bool finitePointsOnly, AxisAlignedBox* __restrict extents )
{
	updatePointsRgbGeomDepth( mCurrentCaptureData );

	int32_t width = mCurrentCaptureData->mChannelDepth.getWidth();
	int32_t height = mCurrentCaptureData->mChannelDepth.getHeight();
	uint32_t numPoints = width * height;

	// can we just memcpy?
	if( ! finitePointsOnly ) {
		memcpy( dest, mCurrentCaptureData->mPointsRgbGeomDepth.get(), sizeof(float) * 6 * numPoints );
		if( extents ) {
			*extents = AxisAlignedBox( vec3( dest[0], dest[1], dest[2] ), vec3( dest[0], dest[1], dest[2] ) );
			const float *ptr = dest + 6;
			for( uint32_t p = 1; p < numPoints; ++p, ptr += 6 )
				extents->include( vec3( ptr[0], ptr[1], ptr[2] ) );
		}

		return numPoints;
	}
	else {
		// testing for finite points
		bool firstPointsExtents = true;
		uint32_t returnedPoints = 0;
		const float *ptr = mCurrentCaptureData->mPointsRgbGeomDepth.get();
		for( size_t p = 0; p < numPoints; ++p, ptr += 6 ) {
			vec3 v( ptr[0], ptr[1], ptr[2] );
			if( isnan( ptr[0] ) )
				continue;
			memcpy( dest, ptr, sizeof(float) * 6 );
			returnedPoints++;
			dest += 6;
			if( extents ) {
				if( firstPointsExtents ) {
					*extents = AxisAlignedBox( vec3( ptr[0], ptr[1], ptr[2] ), vec3( ptr[0], ptr[1], ptr[2] ) );
					firstPointsExtents = false;
				}
				else
					extents->include( vec3( ptr[0], ptr[1], ptr[2] ) );
			}
		}

		return returnedPoints;
	}
}

uint32_t Camera::transformPointsRgbGeomDepth( const ci::Surface8u& colorGeomDepth, const ci::Channel16u& depth, float *dest, bool finitePointsOnly ) const
{
	// Ensure depth table is computed
	calcDepthTableGeomDepth();

	int32_t width = depth.getWidth();
	int32_t height = depth.getHeight();
	uint32_t numPoints = finitePointsOnly ? 0 : width * height;
	uint8_t pixelInc = colorGeomDepth.getPixelInc();

	for( size_t y = 0; y < height; ++y ) {
		const uint8_t *colors = colorGeomDepth.getData( ivec2( 0, y ) );
		const uint16_t *depthPtr = depth.getData( ivec2( 0, y ) );
		const Colorf *depthTableRow = (Colorf*)mDepthTableGeomDepth->getData( ivec2( 0, y ) );
		if( finitePointsOnly ) {
			for( size_t x = 0; x < width; ++x ) {
				if( depthPtr[x] == 0 )
					continue;
				float depth = (float)depthPtr[x];
				Colorf tableEntry = depthTableRow[x];
				vec3 v( tableEntry.r * depth, -tableEntry.g * depth, tableEntry.b * depth );
				if( isnan( v.x ) )
					continue;
				dest[0] = v.x; dest[1] = v.y; dest[2] = v.z;
				dest[3] = colors[x*pixelInc+0] * 0.00392156f; dest[4] = colors[x*pixelInc+1] * 0.00392156f; dest[5] = colors[x*pixelInc+2] * 0.00392156f;
				dest += 6;
				++numPoints;
			}
		}
		else {
			for( size_t x = 0; x < width; ++x ) {
				float depth = (float)depthPtr[x];
				Colorf tableEntry = depthTableRow[x];
				vec3 v( tableEntry.r * depth, -tableEntry.g * depth, tableEntry.b * depth );
				dest[0] = v.x; dest[1] = v.y; dest[2] = v.z;
				dest[3] = colors[x*pixelInc+0] * 0.00392156f; dest[4] = colors[x*pixelInc+1] * 0.00392156f; dest[5] = colors[x*pixelInc+2] * 0.00392156f;
				dest += 6;
			}
		}
	}

	return numPoints;
}

uint32_t Camera::getPointsRgbGeomColor( float *dest, bool finitePointsOnly, AxisAlignedBox *extents )
{
	updatePointsRgbGeomColor( mCurrentCaptureData );

	int32_t width = mCurrentCaptureData->mSurfaceColor.getWidth();
	int32_t height = mCurrentCaptureData->mSurfaceColor.getHeight();
	uint32_t numPoints = width * height;

	// can we just memcpy?
	if( ! finitePointsOnly ) {
		memcpy( dest, mCurrentCaptureData->mPointsRgbGeomColor.get(), sizeof(float) * 6 * numPoints );
		if( extents ) {
			*extents = AxisAlignedBox( vec3( dest[0], dest[1], dest[2] ), vec3( dest[0], dest[1], dest[2] ) );
			const float *ptr = dest + 6;
			for( uint32_t p = 1; p < numPoints; ++p, ptr += 6 )
				extents->include( vec3( ptr[0], ptr[1], ptr[2] ) );
		}

		return numPoints;
	}
	else {
		// testing for finite points
		bool firstPointsExtents = true;
		uint32_t returnedPoints = 0;
		const float *ptr = mCurrentCaptureData->mPointsRgbGeomColor.get();
		for( size_t p = 0; p < numPoints; ++p, ptr += 6 ) {
			vec3 v( ptr[0], ptr[1], ptr[2] );
			if( isnan( ptr[0] ) )
				continue;
			memcpy( dest, ptr, sizeof(float) * 6 );
			dest += 6;
			returnedPoints++;
			if( extents ) {
				if( firstPointsExtents ) {
					*extents = AxisAlignedBox( vec3( ptr[0], ptr[1], ptr[2] ), vec3( ptr[0], ptr[1], ptr[2] ) );
					firstPointsExtents = false;
				}
				else
					extents->include( vec3( ptr[0], ptr[1], ptr[2] ) );
			}
		}

		return returnedPoints;
	}
}

void Camera::updatePointsGeomDepth( CaptureData *imageSet ) const
{
	if( mCurrentCaptureData->mPointsGeomDepthIsValid )
		return;

	// Requires a valid depth image
	if( ! imageSet->mK4aImageDepth )
		throw ExceptionGetImage();

	// allocate the k4a_image_t if this CaptureData doesn't already have one
	if( ! imageSet->k4aImagePointsGeomDepth ) {
		if( K4A_RESULT_SUCCEEDED != k4a_image_create( K4A_IMAGE_FORMAT_CUSTOM16, k4a_image_get_width_pixels( imageSet->mK4aImageDepth ),
														k4a_image_get_height_pixels( imageSet->mK4aImageDepth ), k4a_image_get_width_pixels( imageSet->mK4aImageDepth ) * 3 * (int)sizeof(int16_t),
														&imageSet->k4aImagePointsGeomDepth) ) {
			throw ExceptionGetImage();
		}
	}

	// warp color image into geometry of depth camera
	if( K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud( mK4aTransformation, imageSet->mK4aImageDepth, K4A_CALIBRATION_TYPE_DEPTH, imageSet->k4aImagePointsGeomDepth ) )
		throw ExceptionGetImage();

	imageSet->mPointsGeomDepthIsValid = true;
}

void Camera::updatePointsGeomColor( CaptureData *imageSet ) const
{
	if( mCurrentCaptureData->mPointsGeomColorIsValid )
		return;

	// Requires a valid color and depth image
	if( ! imageSet->mK4aImageColor || ! imageSet->mK4aImageDepth || ! imageSet->mK4aImageDepthGeomColor )
		throw ExceptionGetImage();

	// allocate the k4a_image_t if this CaptureData doesn't already have one
	if( ! imageSet->k4aImagePointsGeomColor ) {
		if( K4A_RESULT_SUCCEEDED != k4a_image_create( K4A_IMAGE_FORMAT_CUSTOM16, k4a_image_get_width_pixels( imageSet->mK4aImageDepthGeomColor ),
														k4a_image_get_height_pixels( imageSet->mK4aImageDepthGeomColor ), k4a_image_get_width_pixels( imageSet->mK4aImageDepthGeomColor ) * 3 * (int)sizeof(int16_t),
														&imageSet->k4aImagePointsGeomColor) ) {
			throw ExceptionGetImage();
		}
	}

	// warp color image into geometry of depth camera
	if( K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud( mK4aTransformation, imageSet->mK4aImageDepthGeomColor, K4A_CALIBRATION_TYPE_COLOR, imageSet->k4aImagePointsGeomColor ) )
		throw ExceptionGetImage();

	imageSet->mPointsGeomColorIsValid = true;
}

void Camera::calcDepthTableGeomDepth() const
{
	// if already cached, return
	if( mDepthTableGeomDepth )
		return;

	mDepthTableGeomDepth = make_unique<ci::Surface32f>( getDepthWidth(), getDepthHeight(), false, SurfaceChannelOrder::RGB );

	float scaleYZ = scaleFromMm(), scaleX = scaleFromMm();
	if( mCoordinateSystem == CoordinateSystem::RIGHT_HANDED_Y_UP )
		scaleYZ = -scaleYZ;

	vec3 *tableData = (vec3*)mDepthTableGeomDepth->getData();
	for( int y = 0, idx = 0; y < getDepthHeight(); y++ ) {
		for( int x = 0; x < getDepthWidth(); x++, idx++ ) {
			int valid;
			k4a_float2_t p;
			p.xy.x = (float)x;
			p.xy.y = (float)y;
			k4a_float3_t ray;
			k4a_calibration_2d_to_3d( &mK4aCalibration, &p, 1.0f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid );

			if( valid )
				tableData[idx] = { ray.xyz.x * scaleX, -ray.xyz.y * scaleYZ, ray.xyz.z * scaleYZ };
			else
				tableData[idx] = { nan(""), nan(""), 0 };
		}
	}
}

void Camera::calcDepthTableGeomColor() const
{
	// if already cached, return
	if( mDepthTableGeomColor )
		return;

	mDepthTableGeomColor = make_unique<ci::Surface32f>( getColorWidth(), getColorHeight(), false, SurfaceChannelOrder::RGB );

	float scaleYZ = scaleFromMm(), scaleX = scaleFromMm();
	if( mCoordinateSystem == CoordinateSystem::RIGHT_HANDED_Y_UP )
		scaleYZ = -scaleYZ;

	vec3 *tableData = (vec3*)mDepthTableGeomColor->getData();
	for( int y = 0, idx = 0; y < getColorHeight(); y++ ) {
		for( int x = 0; x < getColorWidth(); x++, idx++ ) {
			int valid;
			k4a_float2_t p;
			p.xy.x = (float)x;
			p.xy.y = (float)y;
			k4a_float3_t ray;
			k4a_calibration_2d_to_3d( &mK4aCalibration, &p, 1.0f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid );

			if( valid )
				tableData[idx] = { ray.xyz.x * scaleX, -ray.xyz.y * scaleYZ, ray.xyz.z * scaleYZ };
			else
				tableData[idx] = { nan(""), nan(""), 0 };
		}
	}
}

glm::vec2 Camera::point3dTo2dImpl( k4a_calibration_type_t srcGeom, k4a_calibration_type_t dstGeom, glm::vec3 pt, bool *valid ) const
{
	float scaleYZ = 1.0f / scaleFromMm(), scaleX = 1.0f / scaleFromMm();
	if( mCoordinateSystem == CoordinateSystem::RIGHT_HANDED_Y_UP )
		scaleYZ = -scaleYZ;
	k4a_float3_t pt3 = { pt.x * scaleX, pt.y * scaleYZ, pt.z * scaleYZ };
	k4a_float2_t result;
	int validInt = 0;

	if( K4A_RESULT_SUCCEEDED != k4a_calibration_3d_to_2d( &mK4aCalibration, &pt3, srcGeom, dstGeom, &result, &validInt ) )
		throw ExceptionProjection();
	if( valid )
		*valid = validInt == 1;

	return vec2{ result.xy.x, result.xy.y };	
}

vec2 Camera::depth3dToColor2d( glm::vec3 pt, bool *valid ) const
{
	return point3dTo2dImpl( K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, pt, valid );
}

vec2 Camera::depth3dToDepth2d( glm::vec3 pt, bool *valid ) const
{
	return point3dTo2dImpl( K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, pt, valid );
}

glm::vec2 Camera::color2dToDepth2d( glm::ivec2 colorPt, bool *valid ) const
{
	k4a_float2_t ptColor = { (float)colorPt.x, (float)colorPt.y };
	k4a_float2_t result;
	int validInt = 0;
	if( K4A_RESULT_SUCCEEDED != k4a_calibration_color_2d_to_depth_2d( &mK4aCalibration, &ptColor, mCurrentCaptureData->mK4aImageDepth, &result, &validInt ) )
		throw ExceptionProjection();
	if( valid )
		*valid = validInt == 1;

	return vec2{ result.xy.x, result.xy.y };
}

glm::vec2 Camera::depth2dToColor2d( glm::ivec2 depthPt, bool *valid ) const
{
	k4a_float2_t ptDepthGeom = { (float)depthPt.x, (float)depthPt.y };
	k4a_float2_t result;
	int validInt = 0;

	if( ! mCurrentCaptureData->mK4aImageDepth ) {
		if( valid )
			*valid = false;
		return { 0, 0 };
	}

	// nearest neighbor sample of depth map
	float depthMm = mCurrentCaptureData->mChannelDepth.getValue( depthPt );

	if( K4A_RESULT_SUCCEEDED != k4a_calibration_2d_to_2d( &mK4aCalibration, &ptDepthGeom, depthMm, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &result, &validInt ) )
		throw ExceptionProjection();
	if( valid )
		*valid = validInt == 1;

	return vec2{ result.xy.x, result.xy.y };
}

vec3 Camera::point2dTo3dImpl( k4a_calibration_type_t srcGeom, k4a_calibration_type_t dstGeom, glm::ivec2 pt, float depthMm, bool *valid ) const
{
	if( ! mCurrentCaptureData->mK4aImageDepth ) {
		if( valid )
			*valid = false;
		return { 0, 0, 0 };
	}

	k4a_float2_t ptColorGeom = { (float)pt.x, (float)pt.y };
	k4a_float3_t result;
	int validInt = 0;
	if( K4A_RESULT_SUCCEEDED != k4a_calibration_2d_to_3d( &mK4aCalibration, &ptColorGeom, depthMm, srcGeom, dstGeom, &result, &validInt ) )
		throw ExceptionProjection();
	if( valid )
		*valid = validInt == 1;

	float scaleYZ = scaleFromMm(), scaleX = scaleFromMm();
	if( mCoordinateSystem == CoordinateSystem::RIGHT_HANDED_Y_UP )
		scaleYZ = -scaleYZ;
	return vec3{ result.xyz.x * scaleX, result.xyz.y * scaleYZ, result.xyz.z * scaleYZ };
}

vec3 Camera::depth2dToDepth3d( glm::ivec2 pt, bool *valid ) const
{
	if( ! mCurrentCaptureData->mK4aImageDepth ) return vec3( 0 );
	// nearest neighbor sample of depth map
	float depthMm = mCurrentCaptureData->mChannelDepth.getValue( pt );
	return point2dTo3dImpl( K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, pt, depthMm, valid );
}

vec3 Camera::depth2dToColor3d( glm::ivec2 pt, bool *valid ) const
{
	if( ! mCurrentCaptureData->mK4aImageDepth ) return vec3( 0 );
	// nearest neighbor sample of depth map
	float depthMm = mCurrentCaptureData->mChannelDepth.getValue( pt );
	return point2dTo3dImpl( K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, pt, depthMm, valid );
}

vec3 Camera::color2dToColor3d( glm::ivec2 pt, bool *valid ) const
{
	updateDepthGeomColor( mCurrentCaptureData );
	if( ! mCurrentCaptureData->mK4aImageDepthGeomColor ) return vec3( 0 );
	// nearest neighbor sample of depth map
	float depthMm = mCurrentCaptureData->mChannelDepthGeomColor.getValue( pt );
	return point2dTo3dImpl( K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, pt, depthMm, valid );
}

vec3 Camera::color2dToDepth3d( glm::ivec2 pt, bool *valid ) const
{
	updateDepthGeomColor( mCurrentCaptureData );
	if( ! mCurrentCaptureData->mK4aImageDepthGeomColor ) return vec3( 0 );
	// nearest neighbor sample of depth map
	float depthMm = mCurrentCaptureData->mChannelDepthGeomColor.getValue( pt );
	return point2dTo3dImpl( K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, pt, depthMm, valid );
}

//! Starts the camera's IMU sample stream
void Camera::startImu()
{
	if( K4A_RESULT_SUCCEEDED != k4a_device_start_imu( mK4aDevice ) )
		throw ExceptionImu();
}

//! Stops the camera's IMU sample stream
void Camera::stopImu()
{
	k4a_device_stop_imu( mK4aDevice );
}

//! Updates the current IMU sample. Returns \c false if no sample was updated. Can be retrieved with getImuSample().
bool Camera::updateImu( bool waitForSample )
{
	int32_t timeoutMs = waitForSample ? K4A_WAIT_INFINITE : 0;
	if( K4A_RESULT_SUCCEEDED == k4a_device_get_imu_sample( mK4aDevice, &mImuSample.raw, timeoutMs ) ) {
		if( mCoordinateSystem == CoordinateSystem::RIGHT_HANDED_Y_UP ) {
			mImuSample.accelerometer = vec3( mImuSample.raw.acc_sample.xyz.y, -mImuSample.raw.acc_sample.xyz.z, -mImuSample.raw.acc_sample.xyz.x );
			mImuSample.gyroscope = vec3( mImuSample.raw.gyro_sample.xyz.y, -mImuSample.raw.gyro_sample.xyz.z, -mImuSample.raw.gyro_sample.xyz.x );
		}
		else {
			mImuSample.accelerometer = vec3( -mImuSample.raw.acc_sample.xyz.y, mImuSample.raw.acc_sample.xyz.z, -mImuSample.raw.acc_sample.xyz.x );
			mImuSample.gyroscope = vec3( -mImuSample.raw.gyro_sample.xyz.y, mImuSample.raw.gyro_sample.xyz.z, -mImuSample.raw.gyro_sample.xyz.x );
		}
		return true;
	}
	else
		return false;
}

float Camera::scaleFromMm() const
{
	if( mUnit == Unit::METER )
		return 1.0f / 1000.0f;
	else
		return 1.0f;
}

namespace {
ci::log::Level k4aLogLevelToCinder( k4a_log_level_t level )
{
	switch( level ) {
		case K4A_LOG_LEVEL_CRITICAL:	return ci::log::LEVEL_INFO; // TODO: K4A passes this through when devices are initialized, there are some false positives we may need to filter out somehow
		case K4A_LOG_LEVEL_ERROR:		return ci::log::LEVEL_ERROR;
		case K4A_LOG_LEVEL_WARNING:		return ci::log::LEVEL_WARNING;
		case K4A_LOG_LEVEL_INFO:		return ci::log::LEVEL_INFO;
		case K4A_LOG_LEVEL_TRACE:		return ci::log::LEVEL_DEBUG;
		case K4A_LOG_LEVEL_OFF:			return ci::log::LEVEL_VERBOSE; // nothing in ci::log maps to this
		default:						CI_ASSERT_NOT_REACHABLE();
	}

	return ci::log::LEVEL_INFO;
}
}

void onKinectAzureLogMessage(void *context, k4a_log_level_t level, const char *file, const int line, const char *message )
{
	auto cinderLevel = k4aLogLevelToCinder( level );
	ci::log::Entry( cinderLevel, ci::log::Location( CINDER_CURRENT_FUNCTION, file, line ) ) << message;
}

void setLogMessageLevel( k4a_log_level_t level )
{
	k4a_result_t result = k4a_set_debug_message_handler( onKinectAzureLogMessage, nullptr, level );
	CI_VERIFY( result == K4A_RESULT_SUCCEEDED );
}

} // namespace cika