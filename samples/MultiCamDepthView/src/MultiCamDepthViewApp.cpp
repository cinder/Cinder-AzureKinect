#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/CinderImGui.h"
#include "cinder/CameraUi.h"
#include "cinder/Utilities.h"

#include "CinderKinectAzure.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class CameraInstance {
  public:
	CameraInstance( const cika::Device &device ) : mDevice( device ) {}

	void setup();
	void update();

	std::unique_ptr<cika::Camera>		mKinectCamera;

	gl::VboMeshRef						mPointsVboMesh;
	cika::ImuSample						mImuSample;

	vec3								mTranslation{ 0 };
	vec3								mEulerAngles{ 0 };

	cika::Device			mDevice;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 1;
	int						mCameraFps;
	int						mGeomColor = 0; // camera geometry - depth:0, color:1
	bool					mDrawPoints = true;
	bool					mDrawCoordinateFrame = true;
	bool					mDrawColorFrustum = false;
	bool					mDrawDepthFrustum = false;
	vec2					mFrustumRange = vec2( 0.2f, 5.0f ); // meters
};


class MultiCamDepthViewApp : public App {
  public:
	void setup() override;
	void doGui();
	void doCameraInstanceGui( CameraInstance &camera, size_t index );
	void update() override;
	void draw() override;
	void resize() override;

	std::vector<cika::Device>			mDevices;
	std::vector<CameraInstance>			mCameraInstances;

	CameraPersp		mCamera;
	CameraUi		mCamUi;
};

void CameraInstance::setup()
{
	mKinectCamera.reset();

	// initialize
	try {
		mKinectCamera = make_unique<cika::Camera>( (k4a_color_resolution_t)(K4A_COLOR_RESOLUTION_OFF + mCameraResolutionInt), k4a_depth_mode_t(K4A_DEPTH_MODE_OFF + mDepthModeInt),
						K4A_FRAMES_PER_SECOND_30, mDevice );
		mKinectCamera->setCoordinateSystem( cika::CoordinateSystem::RIGHT_HANDED_Y_UP );
		mKinectCamera->setUnits( cika::Unit::METER );
		mKinectCamera->startCapture( cika::CaptureFormat() );
		mKinectCamera->startImu();

		mCameraFps = mKinectCamera->getFps();

		auto layout = gl::VboMesh::Layout().usage( GL_STREAM_DRAW ).interleave().attrib( geom::Attrib::POSITION, 3 ).attrib( geom::Attrib::COLOR, 3 );
		mPointsVboMesh = gl::VboMesh::create( mKinectCamera->getColorWidth() * mKinectCamera->getColorHeight(), GL_POINTS, { layout } );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void MultiCamDepthViewApp::setup()
{
	ImGui::Initialize();

	setWindowSize( 1280, 720 );

	mDevices = cika::Device::getDeviceList();
	for( auto& device : mDevices ) {
		mCameraInstances.emplace_back( device );
		mCameraInstances.back().setup();
	}

	mCamera = CameraPersp();
	mCamera.setEyePoint( vec3(0, 0, 1) );
	mCamera.lookAt( vec3(0, 0, -1) );
	mCamera.setPerspective( 45, getWindowAspectRatio(), 0.1f, 80.0f );
	mCamUi = CameraUi( &mCamera, getWindow() );
}

void MultiCamDepthViewApp::doCameraInstanceGui( CameraInstance &camera, size_t index )
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

	static vector<string> geomNames = { "Depth", "Color" };
	ImGui::Combo( "Camera Geometry", &camera.mGeomColor, geomNames );

	// Camera FPS
	ImGui::LabelText( "FPS", std::to_string(camera.mCameraFps).c_str() );

	ImGui::DragFloat3( "Translation", &camera.mTranslation, 0.01f, -10.0f, 10.0f, "%.2f", 1.0f );
	ImGui::DragFloat3( "Rotation", &camera.mEulerAngles, 0.1f, -180.0f, 180.0f, "%.2f", 1.0f );

	ImGui::Checkbox( "Draw Points", &camera.mDrawPoints );
	ImGui::Checkbox( "Draw Color Frustum", &camera.mDrawColorFrustum );
	ImGui::Checkbox( "Draw Depth Frustum", &camera.mDrawDepthFrustum );
	ImGui::Checkbox( "Draw Coordinate Frame", &camera.mDrawCoordinateFrame );
	ImGui::DragFloat2( "Frustum Range", &camera.mFrustumRange, 0.1f, 0.0f, 20.0f, "%.2f", 1.0f );

	ImGui::PopID();
}

void MultiCamDepthViewApp::doGui()
{
	ImGui::Begin( "Parameters" );

	int index = 0;
	for( auto &camera : mCameraInstances ) {
		doCameraInstanceGui( camera, index );
		++index;
	}
	
	ImGui::End();
}

void MultiCamDepthViewApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
}

void CameraInstance::update()
{
	try {
		if( mKinectCamera && mKinectCamera->update() ) {
			float *pointsPtr = (float*)mPointsVboMesh->getVertexArrayVbos()[0]->mapWriteOnly();
			uint32_t numPoints;
			if( mGeomColor )
				numPoints = mKinectCamera->getPointsRgbGeomColor( pointsPtr, true ); // OpenGL Y-up coords, mm -> meters, ignoreInvalid
			else
				numPoints = mKinectCamera->getPointsRgbGeomDepth( pointsPtr, true ); // OpenGL Y-up coords, mm -> meters, ignoreInvalid
			mPointsVboMesh->getVertexArrayVbos()[0]->unmap();
			mPointsVboMesh->updateNumVertices( numPoints );

			// IMU
			if( mKinectCamera->updateImu() )
				mImuSample = mKinectCamera->getImuSample();
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void MultiCamDepthViewApp::update()
{
	doGui();

	for( auto &camera : mCameraInstances )
		camera.update();
}

void MultiCamDepthViewApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::enableAlphaBlending();
	gl::color( ColorA8u( 255, 255, 255, 255 ) );

	gl::ScopedMatrices mtx_;
	gl::setMatrices( mCamera );

	for( auto& camera : mCameraInstances ) {
		gl::ScopedMatrices mtx_;
		gl::translate( camera.mTranslation );
		gl::rotate( toRadians( camera.mEulerAngles.x ), vec3( 1, 0, 0 ) );
		gl::rotate( toRadians( camera.mEulerAngles.y ), vec3( 0, 1, 0 ) );
		gl::rotate( toRadians( camera.mEulerAngles.z ), vec3( 0, 0, 1 ) );
		if( camera.mPointsVboMesh && camera.mDrawPoints )
			gl::draw( camera.mPointsVboMesh );

		gl::color( ColorA8u( 255, 128, 64, 255 ) );
		if( camera.mDrawColorFrustum )
			gl::drawFrustum( camera.mKinectCamera->getColorFrustum( camera.mFrustumRange.x, camera.mFrustumRange.y ) );
		if( camera.mDrawDepthFrustum )
			gl::drawFrustum( camera.mKinectCamera->getDepthFrustum( camera.mFrustumRange.x, camera.mFrustumRange.y ) );

		// draw origin coordinate frame
		if( camera.mDrawCoordinateFrame ) {
			gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
			float coordinateFrameScale = 0.2f;
			gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale );
		}
	}
}

CINDER_APP( MultiCamDepthViewApp, RendererGl )
