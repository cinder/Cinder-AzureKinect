#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImgui.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"

#include "CinderKinectAzure.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const int DRAW_BORDER = 5;

class CameraInstance {
  public:
	CameraInstance( const cika::Device &device ) : mDevice( device ) {}

	void setup();
	void update();

	std::unique_ptr<cika::Camera>		mKinectCamera;
	gl::TextureRef						mColorTex, mDepthTex;

	cika::Device			mDevice;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 1;
	int						mCameraFps;
};

class MultiCamImageCaptureApp : public App {
  public:
	void setup() override;
	void doCameraInstanceGui( CameraInstance &camera, size_t index );
	void doGui();
	void update() override;
	void draw() override;

	std::vector<cika::Device>				mDevices;
	std::vector<CameraInstance>				mCameraInstances;

	ci::gl::GlslProgRef					mHeatMapGlsl;
};

void CameraInstance::setup()
{
	mKinectCamera.reset();
	mColorTex.reset();
	mDepthTex.reset();

	// initialize
	try {
		mKinectCamera = make_unique<cika::Camera>( (k4a_color_resolution_t)(K4A_COLOR_RESOLUTION_OFF + mCameraResolutionInt), k4a_depth_mode_t(K4A_DEPTH_MODE_OFF + mDepthModeInt),
						K4A_FRAMES_PER_SECOND_30, mDevice );
		mKinectCamera->startCapture( cika::CaptureFormat() );

		mCameraFps = mKinectCamera->getFps();
		
		console() << "Camera initialized. Serial #" << mKinectCamera->getSerialNumber() << std::endl;

		if( mKinectCamera->getColorEnabled() )
			mColorTex = gl::Texture::create( Surface8u( mKinectCamera->getColorWidth(), mKinectCamera->getColorHeight(), false ), gl::Texture::Format().loadTopDown() );
		if( mKinectCamera->getDepthEnabled() )
			mDepthTex = gl::Texture::create( Channel16u( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() ), gl::Texture::Format().loadTopDown() );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}


void MultiCamImageCaptureApp::setup()
{
	//cika::setLogMessageLevel( k4a_log_level_t::K4A_LOG_LEVEL_TRACE );
	ImGui::Initialize();

	mDevices = cika::Device::getDeviceList();
	for( auto& device : mDevices ) {
		mCameraInstances.emplace_back( device );
		mCameraInstances.back().setup();
	}

	// heat map shader
	mHeatMapGlsl = gl::GlslProg::create( loadAsset( "drawTexture.vert" ), loadAsset( "heatMap.frag" ) );
}

void CameraInstance::update()
{
	try {
		if( mKinectCamera && mKinectCamera->update() ) {
			if( mColorTex )
				mColorTex->update( mKinectCamera->getImageColor() );
			if( mDepthTex )
				mDepthTex->update( mKinectCamera->getImageDepth() );
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void MultiCamImageCaptureApp::doCameraInstanceGui( CameraInstance &camera, size_t index )
{
	ImGui::Separator();
	ImGui::PushID( (int)index );
	ImGui::Text( ("Camera " + to_string( index )).c_str() );
	ImGui::LabelText( "Serial #", camera.mKinectCamera->getSerialNumber().c_str() );

	static vector<string> resolutionNames = { "None", "720p (1280 x 720)", "1080p (1920 x 1080)", "1440p (2560 x 1440)", "1536p (2048 x 1536)", "2160p (3840 x 2160)", "3072p (4096 x 3072)" };
	if( ImGui::Combo( "Color Resolution", &camera.mCameraResolutionInt, resolutionNames ) )
		camera.setup();

	static vector<string> depthModeNames = { "None", "Narrow FOV Binned (320 x 288)", "Narrow FOV Unbinned (640 x 576)", "Wide FOV Binned (512 x 512)", "Wide FOV Unbinned (1024 x 1024)" };
	if( ImGui::Combo( "Depth Mode", &camera.mDepthModeInt, depthModeNames ) )
		camera.setup();

	// Camera FPS
	ImGui::LabelText( "FPS", std::to_string(camera.mCameraFps).c_str() );

	ImGui::PopID();
}

void MultiCamImageCaptureApp::doGui()
{
	ImGui::Begin( "Parameters" );

	int index = 0;
	for( auto &camera : mCameraInstances ) {
		doCameraInstanceGui( camera, index );
		++index;
	}
	
	ImGui::End();
}

void MultiCamImageCaptureApp::update()
{
	doGui();

	for( auto &camera : mCameraInstances )
		camera.update();
}

void MultiCamImageCaptureApp::draw()
{
	int32_t nativeDrawWidth = 0;
	int32_t nativeDrawHeight = 0;
	for( auto& camera : mCameraInstances ) {
		nativeDrawWidth = std::max( nativeDrawWidth, camera.mKinectCamera->getColorWidth() + camera.mKinectCamera->getDepthWidth());
		nativeDrawHeight += std::max( camera.mKinectCamera->getColorHeight(), camera.mKinectCamera->getDepthHeight() ) + DRAW_BORDER;
	}	
	float drawScale = std::min( getWindowWidth() / (float)nativeDrawWidth, getWindowHeight() / (float)nativeDrawHeight );

	gl::clear();

	gl::ScopedModelMatrix mtx_;
	gl::scale( drawScale, drawScale );
	
	for( auto& camera : mCameraInstances ) {
		if( camera.mColorTex )
			gl::draw( camera.mColorTex );

		if( camera.mDepthTex ) {
			gl::ScopedGlslProg glslHeat( mHeatMapGlsl );
			gl::ScopedTextureBind depthTex( camera.mDepthTex );
			mHeatMapGlsl->uniform( "uDepthRange", vec2( camera.mKinectCamera->getDepthRange() ) / vec2( 65535.0f ) );
			float scale = camera.mKinectCamera->getColorEnabled() ? camera.mKinectCamera->getColorHeight() / (float)camera.mKinectCamera->getDepthHeight() : 1.0f;
			Rectf drawRect = Rectf( 0, 0, camera.mDepthTex->getWidth() * scale, camera.mDepthTex->getHeight() * scale ) + vec2( DRAW_BORDER + camera.mKinectCamera->getColorWidth(), 0 );
			gl::drawSolidRect( drawRect, vec2( 0, 0 ), vec2( 1, 1 ) );
		}

		gl::translate( 0, std::max( camera.mKinectCamera->getColorHeight(), camera.mKinectCamera->getDepthHeight() ) + (float)DRAW_BORDER );
	}
}

CINDER_APP( MultiCamImageCaptureApp, RendererGl )
