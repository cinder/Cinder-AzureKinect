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

class DepthViewApp : public App {
  public:
	void setupCamera();
	void setup() override;
	void doGui();
	void update() override;
	void savePlyFile( std::filesystem::path path );
	void draw() override;
	void resize() override;

	std::unique_ptr<cika::Camera>		mKinectCamera;
	std::filesystem::path				mPlaybackPath;
	gl::TextureRef						mColorTex;

	gl::VboMeshRef						mPointsVboMesh;

	cika::ImuSample						mImuSample;

	CameraPersp		mCamera;
	CameraUi		mCamUi;

	int						mCameraDeviceIndex = 0;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 4;
	int						mCameraFps;

	bool					mDrawFrustum = true;
	bool					mDrawCameraVideo = false;
	int						mGeomColor = 0;
};

void DepthViewApp::setupCamera()
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
		mKinectCamera->startImu();

		mCameraFps = mKinectCamera->getFps();

		auto layout = gl::VboMesh::Layout().usage( GL_STREAM_DRAW ).interleave().attrib( geom::Attrib::POSITION, 3 ).attrib( geom::Attrib::COLOR, 3 );
		mPointsVboMesh = gl::VboMesh::create( mKinectCamera->getColorWidth() * mKinectCamera->getColorHeight(), GL_POINTS, { layout } ); 

		mColorTex = gl::Texture::create( Surface8u( mKinectCamera->getColorWidth(), mKinectCamera->getColorHeight(), false ), gl::Texture::Format().loadTopDown() );
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void DepthViewApp::setup()
{
	ImGui::Initialize();

	setWindowSize( 1280, 720 );

	setupCamera();
	mCamera = CameraPersp();
	mCamera.setEyePoint( vec3(0, 0, 1) );
	mCamera.lookAt( vec3(0, 0, -1) );
	mCamera.setPerspective( 45, getWindowAspectRatio(), 0.1f, 100.0f );
	mCamUi = CameraUi( &mCamera, getWindow() );
}

void DepthViewApp::doGui()
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

		static vector<string> geomNames = { "Depth", "Color" };
		ImGui::Combo( "Camera Geometry", &mGeomColor, geomNames );

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

	if( mKinectCamera && ImGui::Button( "Save PLY" ) ) {
		auto path = getSaveFilePath( "", { "ply" } );
		if( ! path.empty() )
			savePlyFile( path );
	}

	ImGui::End();
}

void DepthViewApp::savePlyFile( std::filesystem::path path )
{
	// big enough for viz in depth geom of in color geom
	std::unique_ptr<float[]> pointsData( new float[mKinectCamera->getColorWidth() * mKinectCamera->getColorHeight() * 6] );
	size_t numPoints = mKinectCamera->getPointsRgbGeomDepth( pointsData.get(), true );

	FILE *f = fopen( path.string().c_str(), "wb" );
	fprintf( f, "ply\nformat binary_little_endian 1.0\n" );
	fprintf( f, "element vertex %d\n", (int)numPoints );
	fprintf( f, "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n" );
	fprintf( f, "element face 0\nproperty list uchar int vertex_indices\nend_header\n" );
	for( size_t i = 0; i < numPoints; ++i ) {
		uint8_t color[3];
		color[0] = (uint8_t)(pointsData[i * 6 + 3] * 255.0f);
		color[1] = (uint8_t)(pointsData[i * 6 + 4] * 255.0f);
		color[2] = (uint8_t)(pointsData[i * 6 + 5] * 255.0f);
		fwrite( &pointsData[i * 6], sizeof(float), 3, f );
		fwrite( &color, sizeof(uint8_t), 3, f );
	}
	fclose( f );
}

void DepthViewApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
}

void DepthViewApp::update()
{
	doGui();

	try {
		if( mKinectCamera && mKinectCamera->update() ) {
			mColorTex->update( mKinectCamera->getImageColor() );

			// Point Cloud
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

void DepthViewApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::enableAlphaBlending();
//	gl::disableAlphaBlending();
	gl::color( ColorA8u( 255, 255, 255, 255 ) );
	if( mDrawCameraVideo )
		gl::draw( mColorTex, Rectf( 0, 0, mKinectCamera->getColorWidth() / 2.0f, mKinectCamera->getColorHeight() / 2.0f ) );

	gl::ScopedMatrices mtx_;
	gl::setMatrices( mCamera );

	// draw origin coordinate frame
	gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
	float coordinateFrameScale = 0.2f;
	gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale );

	// draw acceleration from IMU
	gl::color( ColorA8u( 255, 255, 0, 255 ) );
	float accelVectorScale = 0.15f;
	gl::drawVector( vec3( 0 ), glm::normalize( mImuSample.accelerometer ) * accelVectorScale, 0.2f * accelVectorScale, 0.05f * accelVectorScale );

	if( mDrawFrustum ) {
		gl::color( ColorA8u( 255, 128, 64, 255 ) );
		if( mGeomColor )
			gl::drawFrustum( mKinectCamera->getColorFrustum( 0.2f, 4.0f ) );
		else
			gl::drawFrustum( mKinectCamera->getDepthFrustum( 0.2f, 4.0f ) );
	}

	if( mPointsVboMesh )
		gl::draw( mPointsVboMesh );
}

CINDER_APP( DepthViewApp, RendererGl )
