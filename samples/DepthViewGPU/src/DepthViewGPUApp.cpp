#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/CinderImGui.h"
#include "cinder/CameraUi.h"
#include "cinder/Utilities.h"

#include "CinderKinectAzure.h"
#include "CinderKinectAzureGl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class DepthViewGpuApp : public App {
  public:
	void setupCamera();
	void setup() override;
	void doGui();
	void update() override;
	void draw() override;
	void resize() override;

	std::unique_ptr<cika::Camera>			mKinectCamera;
	std::unique_ptr<cika::PointCloudGl>		mPointCloudGl;
	std::filesystem::path					mPlaybackPath;

	CameraPersp		mCamera;
	CameraUi		mCamUi;

	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 4;
	int						mCameraFps;

	bool					mDrawFrustum = true;
	bool					mDrawCameraVideo = false;
};

void DepthViewGpuApp::setupCamera()
{
	mKinectCamera.reset();
	mPointCloudGl.reset();

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
		mKinectCamera->startCapture( cika::CaptureFormat().imageColorGeomDepth() );

		mCameraFps = mKinectCamera->getFps();

		// Create the GL point cloud renderer
		glm::ivec2 depthRes( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() );
		mPointCloudGl = make_unique<cika::PointCloudGl>( depthRes );

		// Initialize the depth table texture
		Surface32f depthTable = mKinectCamera->getDepthTableGeomDepth();
		mPointCloudGl->updateDepthTable( depthTable );
	}
	catch( const std::exception& e ) {
		CI_LOG_E( "Error initializing camera: " << e.what() );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void DepthViewGpuApp::setup()
{
	ImGui::Initialize();

	setWindowSize( 1280, 720 );

	setupCamera();

	mCamera = CameraPersp();
	mCamera.setEyePoint( vec3(0, 0, 1) );
	mCamera.lookAt( vec3(0, 0, -1) );
	mCamera.setPerspective( 45, getWindowAspectRatio(), 0.2f, 2000.0f );
	mCamUi = CameraUi( &mCamera, getWindow() );
}

void DepthViewGpuApp::doGui()
{
	ImGui::Begin( "Parameters" );

	if( mKinectCamera ) {
		ImGui::BeginGroup();
		static vector<string> resolutionNames = { "None", "720p (1280 x 720)", "1080p (1920 x 1080)", "1440p (2560 x 1440)", "1536p (2048 x 1536)", "2160p (3840 x 2160)", "3072p (4096 x 3072)" };
		if( ImGui::Combo( "Color Resolution", &mCameraResolutionInt, resolutionNames ) )
			setupCamera();

		static vector<string> depthModeNames = { "None", "Narrow FOV Binned (320 x 288)", "Narrow FOV Unbinned (640 x 576)", "Wide FOV Binned (512 x 512)", "Wide FOV Unbinned (1024 x 1024)" };
		if( ImGui::Combo( "Depth Mode", &mDepthModeInt, depthModeNames ) )
			setupCamera();

		// Camera FPS
		ImGui::LabelText( "FPS", std::to_string(mCameraFps).c_str() );

		ImGui::EndGroup();

		ImGui::Checkbox( "Draw Frustum", &mDrawFrustum );
		ImGui::Checkbox( "Draw Video", &mDrawCameraVideo );

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

void DepthViewGpuApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
}

void DepthViewGpuApp::update()
{
	doGui();

	try {
		if( mKinectCamera && mPointCloudGl && mKinectCamera->update() ) {
			// Update the GL textures from camera data
			mPointCloudGl->updateColor( mKinectCamera->getImageColorGeomDepth() );
			mPointCloudGl->updateDepth( mKinectCamera->getImageDepth() );
		}
	}
	catch( const std::exception& e ) {
		CI_LOG_E( "Error grabbing new frame: " << e.what() );
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void DepthViewGpuApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::pointSize( 2.0f );

	if( mDrawCameraVideo && mKinectCamera && mPointCloudGl ) {
		gl::color( ColorA8u( 255, 255, 255, 255 ) );
		gl::draw( mPointCloudGl->getColorTexture(), Rectf( 0, 0, mKinectCamera->getDepthWidth() / 2.0f, mKinectCamera->getDepthHeight() / 2.0f ) );
	}

	gl::ScopedMatrices mtx_;
	gl::setMatrices( mCamera );

	if( mPointCloudGl ) {
		mPointCloudGl->draw();
	}

	if( mDrawFrustum && mKinectCamera ) {
		gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
		gl::color( ColorA8u( 255, 128, 64, 255 ) );
		gl::drawFrustum( mKinectCamera->getDepthFrustum( 0.2f, 4.0f ) );
	}

	gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
	float coordinateFrameScale = 0.2f;
	gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale );
}

CINDER_APP( DepthViewGpuApp, RendererGl )
