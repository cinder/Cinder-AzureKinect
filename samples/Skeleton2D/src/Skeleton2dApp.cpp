#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/CinderImGui.h"
#include "cinder/CameraUi.h"
#include "cinder/Utilities.h"
#include "cinder/AxisAlignedBox.h"

#include "CinderKinectAzure.h"
#include "CinderKinectAzureBodyTracking.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const std::vector<Color8u> sColors = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, // R G B
										{ 0, 255, 255 }, { 255, 0, 255 }, { 255, 255, 0 } }; // C M Y

class Skeleton2dApp : public App {
  public:
	void setupCamera();
	void setup() override;
	void doGui();
	void update() override;
	void draw() override;

	std::unique_ptr<cika::Camera>		mKinectCamera;
	std::filesystem::path				mPlaybackPath;
	std::unique_ptr<cika::BodyTracker>	mBodyTracker;

	gl::TextureRef						mColorTex, mDepthTex;

	int						mCameraDeviceIndex = 0;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 2; // PERFORMANCE
	int						mCameraFps;
	bool					mDrawCameraVideo = false;
	bool					mDrawSkeletons = true;
	int						mGeomColor = 1; // which camera geometry to map to - depth, color

	std::vector<cika::Body>			mBodies; // <id, bounding box, texture>
};

void Skeleton2dApp::setupCamera()
{
	mKinectCamera.reset();
	mColorTex.reset();

	// initialize
	try {
		if( mPlaybackPath.empty() )
			mKinectCamera = make_unique<cika::Camera>( (k4a_color_resolution_t)(K4A_COLOR_RESOLUTION_OFF + mCameraResolutionInt), k4a_depth_mode_t(K4A_DEPTH_MODE_OFF + mDepthModeInt) );
		else {
			mKinectCamera = make_unique<cika::Camera>( mPlaybackPath, true );
			mCameraResolutionInt = (int)mKinectCamera->getCalibration().color_resolution;
			mDepthModeInt = (int)mKinectCamera->getCalibration().depth_mode;
		}

		mKinectCamera->setCoordinateSystem( cika::CoordinateSystem::RIGHT_HANDED_Y_UP );
		mKinectCamera->setUnits( cika::Unit::METER );
		mKinectCamera->startCapture( cika::CaptureFormat() );
		mBodyTracker = make_unique<cika::BodyTracker>( mKinectCamera.get() );

		mCameraFps = mKinectCamera->getFps();
		
		mColorTex = gl::Texture::create( Surface8u( mKinectCamera->getColorWidth(), mKinectCamera->getColorHeight(), false ), gl::Texture::Format().loadTopDown() );
		mDepthTex = gl::Texture::create( Channel16u( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() ), gl::Texture::Format().loadTopDown() );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void Skeleton2dApp::setup()
{
	ImGui::Initialize();

//	cika::setLogMessageLevel( K4A_LOG_LEVEL_INFO );

	setupCamera();
	setWindowSize( mKinectCamera->getColorSize() );
}

void Skeleton2dApp::doGui()
{
	ImGui::Begin( "Parameters" );
	ImGui::BeginGroup();

	if( mKinectCamera ) {
		static vector<string> resolutionNames = { "None", "720p (1280 x 720)", "1080p (1920 x 1080)", "1440p (2560 x 1440)", "1536p (2048 x 1536)", "2160p (3840 x 2160)", "3072p (4096 x 3072)" };
		if( ImGui::Combo( "Color Resolution", &mCameraResolutionInt, resolutionNames ) )
			setupCamera();

		static vector<string> depthModeNames = { "None", "Narrow FOV Binned (320 x 288)", "Narrow FOV Unbinned (640 x 576)", "Wide FOV Binned (512 x 512)", "Wide FOV Unbinned (1024 x 1024)" };
		if( ImGui::Combo( "Depth Mode", &mDepthModeInt, depthModeNames ) )
			setupCamera();

		// Camera FPS
		ImGui::LabelText( "FPS", std::to_string(mCameraFps).c_str() );

		static vector<string> geomNames = { "Depth", "Color" };
		ImGui::Combo( "Camera Geometry", &mGeomColor, geomNames );

		ImGui::EndGroup();

		ImGui::Checkbox( "Draw Skeletons", &mDrawSkeletons );

/*		if( mKinectCamera->isPlaybackPaused() ) {
			if( ImGui::Button( "Play" ) )
				mKinectCamera->setPlaybackPaused( false );
		}
		else {
			if( ImGui::Button( "Pause" ) )
				mKinectCamera->setPlaybackPaused( true );
		}*/

		if( mKinectCamera->isPlayingRecording() ) {
			ImGui::Separator();
			if( mKinectCamera->isPlayingRecording() ) {
				ImGui::ProgressBar( (float)mKinectCamera->getTimestampColorSeconds() / (float)mKinectCamera->getPlaybackDurationSeconds() );
			}
		}
	}

	if( ImGui::Button( "Open Recording" ) ) {
		auto path = getOpenFilePath( "", { "mkv" } );
		if( ! path.empty() ) {
			mPlaybackPath = path;
			setupCamera();
		}
	}

	ImGui::End();
}

void Skeleton2dApp::update()
{
	doGui();

	try {	
		if( mKinectCamera && mKinectCamera->update() ) {
			mColorTex->update( mKinectCamera->getImageColor() );
			mDepthTex->update( mKinectCamera->getImageDepth() );
			if( mBodyTracker->update() )
				mBodies = mBodyTracker->getBodies();
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void Skeleton2dApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::enableAlphaBlending();
	gl::disableDepthRead();

	if( mGeomColor ) {
		gl::color( Colorf( 1.0f, 1.0f, 1.0f ) );
		gl::draw( mColorTex );
	}
	else {
		gl::color( Colorf( 20.0f, 20.0f, 20.0f ) );
		gl::draw( mDepthTex );
	}

	{ // draw skeleton
		if( mDrawSkeletons ) {
			for( auto &body : mBodies ) {
				gl::color( sColors[body.id % sColors.size()] );

				for( auto &bone : cika::getSkeletonBones() ) {
					vec2 a, b;
					if( mGeomColor ) {
						a = mKinectCamera->depth3dToColor2d( body.joints[bone.first].position );
						b = mKinectCamera->depth3dToColor2d( body.joints[bone.second].position );
					}
					else {
						a = mKinectCamera->depth3dToDepth2d( body.joints[bone.first].position );
						b = mKinectCamera->depth3dToDepth2d( body.joints[bone.second].position );
					}
					gl::drawLine( a, b );
				}

				for( size_t j = 0; j < K4ABT_JOINT_COUNT; ++j )
					if( mGeomColor )
						gl::drawSolidCircle( mKinectCamera->depth3dToColor2d( body.joints[j].position ), 2.0f );
					else
						gl::drawSolidCircle( mKinectCamera->depth3dToDepth2d( body.joints[j].position ), 2.0f );
			}
		}
	}
}

CINDER_APP( Skeleton2dApp, RendererGl )
