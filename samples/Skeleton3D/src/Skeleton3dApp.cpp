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

class Skeleton3dApp : public App {
  public:
	void setupCamera();
	void setup() override;
	void doGui();
	void update() override;
	void draw() override;
	void resize() override;

	std::unique_ptr<cika::Camera>		mKinectCamera;
	std::filesystem::path				mPlaybackPath;
	std::unique_ptr<cika::BodyTracker>	mBodyTracker;

	gl::TextureRef						mColorTex;
	gl::VboMeshRef						mPointsVboMesh;

	CameraPersp		mCamera;
	CameraUi		mCamUi;

	int						mCameraDeviceIndex = 0;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 2; // PERFORMANCE
	int						mCameraFps;
	bool					mDrawCameraVideo = false;
	bool					mDrawFrustum = false;
	bool					mDrawPoints = true;
	bool					mDrawSkeletons = true;
	bool					mDrawJointVelocity = true;
	bool					mDrawJointOrientation = true;
	bool					mDrawBoundingBox = true;
	float					mTemporalSmoothing = 0.0f;

	std::vector<cika::Body>			mBodies; // <id, bounding box, texture>
};

void Skeleton3dApp::setupCamera()
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
		mBodyTracker->setTemporalSmoothing( mTemporalSmoothing );

		mCameraFps = mKinectCamera->getFps();

		auto layout = gl::VboMesh::Layout().usage( GL_STREAM_DRAW ).interleave().attrib( geom::Attrib::POSITION, 3 ).attrib( geom::Attrib::COLOR, 3 );
		mPointsVboMesh = gl::VboMesh::create( mKinectCamera->getColorWidth() * mKinectCamera->getColorHeight(), GL_POINTS, { layout } ); 

		mColorTex = gl::Texture::create( Surface8u( mKinectCamera->getColorWidth(), mKinectCamera->getColorHeight(), false ), gl::Texture::Format().loadTopDown() );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void Skeleton3dApp::setup()
{
	ImGui::Initialize();

//	cika::setLogMessageLevel( K4A_LOG_LEVEL_INFO );

	setupCamera();
	mCamera = CameraPersp();
	mCamera.setEyePoint( vec3(0, 0, 1) );
	mCamera.lookAt( vec3(0, 0, -1) );
	mCamera.setPerspective( 45, getWindowAspectRatio(), 0.2f, 200 );
	mCamUi = CameraUi( &mCamera, getWindow() );
}

void Skeleton3dApp::doGui()
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

		ImGui::EndGroup();

		ImGui::Checkbox( "Draw Frustum", &mDrawFrustum );
		ImGui::Checkbox( "Draw Points", &mDrawPoints );
		ImGui::Checkbox( "Draw Skeletons", &mDrawSkeletons );
		ImGui::Checkbox( "Draw Joint Velocity", &mDrawJointVelocity );
		ImGui::Checkbox( "Draw Joint Orientation", &mDrawJointOrientation );
		ImGui::Checkbox( "Draw Bounding Boxes", &mDrawBoundingBox );
		ImGui::Checkbox( "Draw Video", &mDrawCameraVideo );

		if( ImGui::DragFloat( "Temporal Smoothing", &mTemporalSmoothing, 0.01f, 0.0f, 1.0f, "%.2f" ) )
			mBodyTracker->setTemporalSmoothing( mTemporalSmoothing );

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

void Skeleton3dApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
}

void Skeleton3dApp::update()
{
	doGui();

	try {	
		if( mKinectCamera && mKinectCamera->update() ) {
			mColorTex->update( mKinectCamera->getImageColor() );

			// Point Cloud
			float *pointsPtr = (float*)mPointsVboMesh->getVertexArrayVbos()[0]->mapWriteOnly();
			uint32_t numPoints = (uint32_t)mKinectCamera->getPointsRgbGeomDepth( pointsPtr, true );
			mPointsVboMesh->getVertexArrayVbos()[0]->unmap();
			mPointsVboMesh->updateNumVertices( numPoints );

			if( mBodyTracker->update() )
				mBodies = mBodyTracker->getBodies();
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void Skeleton3dApp::draw()
{
	gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
	gl::enableAlphaBlending();
	gl::disableDepthRead();

	if( mDrawCameraVideo ) {
		gl::color( ColorA8u( 255, 255, 255, 128 ) );
		gl::draw( mColorTex, Rectf( 0, 0, mKinectCamera->getColorWidth() / 2.0f, mKinectCamera->getColorHeight() / 2.0f ) );
	}

	gl::ScopedMatrices mtx_;
	gl::setMatrices( mCamera );

	if( mDrawPoints ) { // draw 3D points
		gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
		gl::color( ColorA8u( 255, 255, 255, 128 ) );
		if( mPointsVboMesh )
			gl::draw( mPointsVboMesh );
	}

	{ // draw skeleton
		gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
		if( mDrawSkeletons ) {
			for( auto &body : mBodies ) {
				gl::color( sColors[body.id % sColors.size()] );

				for( auto &bone : cika::getSkeletonBones() )
					gl::drawLine( body.joints[bone.first].position, body.joints[bone.second].position );

				for( size_t j = 0; j < K4ABT_JOINT_COUNT; ++j )
					;//gl::drawSphere( body.joints[j].position, 0.01f );

				if( mDrawJointOrientation ) {
					for( size_t j = 0; j < K4ABT_JOINT_COUNT; ++j ) {
						gl::ScopedModelMatrix mtx_;
						gl::translate( body.joints[j].position );
						gl::rotate( body.joints[j].orientation );
						const float coordinateFrameScale = 0.1f;
						gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale);
					}
				}

				if( mDrawBoundingBox )
					gl::drawStrokedCube( body.extents );
			}
		}

		if( mDrawJointVelocity ) { // draw velocity
			gl::color( ColorA8u( 255, 128, 64, 255 ) );
			float perFrameTimeScale = 1 / 30.0f;
			for( auto &body : mBodies ) {
				for( auto joint : cika::getSkeletonJoints() )
					gl::drawLine( body.joints[joint].position, body.joints[joint].position + body.joints[joint].velocity * perFrameTimeScale );
			}
		}
	}

	float coordinateFrameScale = 0.2f;
	gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale );
}

CINDER_APP( Skeleton3dApp, RendererGl )
