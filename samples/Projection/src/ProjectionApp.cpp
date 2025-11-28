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

class ProjectionApp : public App {
  public:
	void setupCamera();
	void setup() override;
	void doGui();
	void mouseMove( MouseEvent event ) override;
	void update() override;
	void draw() override;
	void resize() override;

	std::unique_ptr<cika::Camera>		mKinectCamera;
	std::filesystem::path				mPlaybackPath;
	gl::TextureRef						mColorTex, mDepthTex;

	gl::VboMeshRef						mPointsVboMesh;
	gl::GlslProgRef						mGrayScaleGlsl;

	CameraPersp		mCamera;
	CameraUi		mCamUi;

	int						mCameraDeviceIndex = 0;
	int						mCameraResolutionInt = 1; // HD720
	int						mDepthModeInt = 4;
	int						mCameraFps;

	bool					mDrawFrustum = true;
	int						mGeomColor = 0;

	Rectf					mColorVideoRect, mDepthVideoRect;
	ivec2					mMouseMovePos; // stored in the geometry of either the color or depth map
	k4a_calibration_type_t	mMouseMoveType;

	// Hover distance
	float					mHoverDistanceCm = 0.0f;
	bool					mHoverValid = false;
};

void ProjectionApp::setupCamera()
{
	mKinectCamera.reset();
	mColorTex.reset();
	mDepthTex.reset();

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
		mDepthTex = gl::Texture::create( Channel16u( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() ), gl::Texture::Format().loadTopDown() );

		resize(); // recalc video rects on resolution change
	}
	catch(...) {
		CI_LOG_E( "Error initializing camera." );
	}
}

void ProjectionApp::setup()
{
	ImGui::Initialize();

	setWindowSize( 1280, 720 );

	mGrayScaleGlsl = gl::GlslProg::create( loadAsset( "drawTexture.vert" ), loadAsset( "grayScale.frag" ) );

	setupCamera();
	mCamera = CameraPersp();
	mCamera.setEyePoint( vec3(0, 0, 1) );
	mCamera.lookAt( vec3(0, 0, -1) );
	mCamera.setPerspective( 45, getWindowAspectRatio(), 0.2f, 20.0f );
	mCamUi = CameraUi( &mCamera, getWindow() );
}

void ProjectionApp::doGui()
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

		ImGui::Separator();
		if( mHoverValid ) {
			ImGui::Text( "Distance: %.1f cm (%.1f in.)", mHoverDistanceCm, mHoverDistanceCm / 2.54f );
		}
		else {
			ImGui::TextDisabled( "Hover over image to measure" );
		}

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

void ProjectionApp::resize()
{
	mCamera.setAspectRatio( getWindowWidth() / (getWindowHeight() / 2.0f) );
	float colorScale = std::min( 1.0f, (getWindowHeight() / 2.0f) / mKinectCamera->getColorHeight() );
	mColorVideoRect = Rectf( 0, 0, mKinectCamera->getColorWidth() * colorScale, mKinectCamera->getColorHeight() * colorScale );
	float depthScale = (mKinectCamera->getColorHeight() * colorScale) / mDepthTex->getHeight();
	mDepthVideoRect = Rectf( 0, 0, mKinectCamera->getDepthWidth() * depthScale, mKinectCamera->getDepthHeight() * depthScale ) + vec2( mColorVideoRect.getWidth(), 0 );
}

void ProjectionApp::mouseMove( MouseEvent event )
{
	if( mColorVideoRect.contains( event.getPos() ) ) {
		mMouseMovePos = ivec2( vec2(0.5f) + (vec2(event.getPos()) * (mColorTex->getWidth() / (float)mColorVideoRect.getWidth()) ) );
		mMouseMoveType = K4A_CALIBRATION_TYPE_COLOR;
	}
	else if( mDepthVideoRect.contains( event.getPos() ) ) {
		mMouseMovePos = ivec2( vec2(0.5f) + (vec2(event.getPos()) - mDepthVideoRect.getUpperLeft()) * ( mDepthTex->getWidth() / (float)mDepthVideoRect.getWidth() ) );
		mMouseMoveType = K4A_CALIBRATION_TYPE_DEPTH;
	}
}

void ProjectionApp::update()
{
	doGui();

	try {
		if( mKinectCamera && mKinectCamera->update() ) {
			mColorTex->update( mKinectCamera->getImageColor() );
			mDepthTex->update( mKinectCamera->getImageDepth() );

			// Point Cloud
			float *pointsPtr = (float*)mPointsVboMesh->getVertexArrayVbos()[0]->mapWriteOnly();
			uint32_t numPoints;
			if( mGeomColor )
				numPoints = mKinectCamera->getPointsRgbGeomColor( pointsPtr, true ); // OpenGL Y-up coords, mm -> meters, ignoreInvalid
			else
				numPoints = mKinectCamera->getPointsRgbGeomDepth( pointsPtr, true ); // OpenGL Y-up coords, mm -> meters, ignoreInvalid
			mPointsVboMesh->getVertexArrayVbos()[0]->unmap();
			mPointsVboMesh->updateNumVertices( numPoints );

			// Calculate distance to hovered point (same logic as draw() uses for the 3D sphere)
			vec3 hoverPos3d;
			if( mMouseMoveType == K4A_CALIBRATION_TYPE_COLOR ) {
				if( mGeomColor )
					hoverPos3d = mKinectCamera->color2dToColor3d( mMouseMovePos, &mHoverValid );
				else
					hoverPos3d = mKinectCamera->color2dToDepth3d( mMouseMovePos, &mHoverValid );
			}
			else {
				if( mGeomColor )
					hoverPos3d = mKinectCamera->depth2dToColor3d( mMouseMovePos, &mHoverValid );
				else
					hoverPos3d = mKinectCamera->depth2dToDepth3d( mMouseMovePos, &mHoverValid );
			}
			if( mHoverValid ) {
				// Distance in meters, convert to cm
				mHoverDistanceCm = glm::length( hoverPos3d ) * 100.0f;
			}
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void ProjectionApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::enableAlphaBlending();
	gl::disableDepthRead();
	gl::color( ColorA8u( 255, 255, 255, 255 ) );

	// draw color
	gl::draw( mColorTex, mColorVideoRect );

	{ // draw depth
		gl::ScopedGlslProg glsl_( mGrayScaleGlsl );
		mGrayScaleGlsl->uniform( "uRange", vec2( mKinectCamera->getDepthRange() ) / vec2( 65535.0f, 65535.0f ) );
		gl::ScopedTextureBind tex( mDepthTex );
		gl::drawSolidRect( mDepthVideoRect, vec2( 0, 0 ), vec2( 1, 1 ) );
	}

	if( mMouseMoveType == K4A_CALIBRATION_TYPE_COLOR ) {
		gl::color( ColorA8u( 255, 255, 0, 255 ) );
		vec2 drawColorPos = vec2( mMouseMovePos ) * ( mColorVideoRect.getHeight() / mColorTex->getHeight() );
		gl::drawSolidCircle( drawColorPos, 5.0f );
		vec2 drawDepthPos = mKinectCamera->color2dToDepth2d( mMouseMovePos );
		gl::drawSolidCircle( drawDepthPos * vec2( mDepthVideoRect.getHeight() / mDepthTex->getHeight() ) + mDepthVideoRect.getUpperLeft(), 5.0f );
	}
	else if( mMouseMoveType == K4A_CALIBRATION_TYPE_DEPTH ) {
		gl::color( ColorA8u( 0, 255, 255, 255 ) );
		vec2 drawColorPos = mKinectCamera->depth2dToColor2d( mMouseMovePos ) * ( mColorVideoRect.getHeight() / mColorTex->getHeight() );
		if( mColorVideoRect.contains( drawColorPos ) )
			gl::drawSolidCircle( drawColorPos, 5.0f );
		vec2 drawDepthPos = vec2( mMouseMovePos ) * ( mDepthVideoRect.getHeight() / mDepthTex->getHeight() );
		gl::drawSolidCircle( drawDepthPos + mDepthVideoRect.getUpperLeft(), 5.0f );
	}

	// draw point cloud
	{
		gl::ScopedMatrices mtx_;
		gl::ScopedViewport vp_{ ivec2( getWindowWidth(), getWindowHeight() - mColorVideoRect.getHeight() ) };
		gl::setMatrices( mCamera );

		// draw origin coordinate frame
		gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color() ) );
		float coordinateFrameScale = 0.2f;
		gl::drawCoordinateFrame( coordinateFrameScale, 0.2f * coordinateFrameScale, 0.05f * coordinateFrameScale );

		if( mDrawFrustum ) {
			gl::color( ColorA8u( 255, 128, 64, 255 ) );
			if( mGeomColor )
				gl::drawFrustum( mKinectCamera->getColorFrustum( 0.2f, 10.0f ) );
			else
				gl::drawFrustum( mKinectCamera->getDepthFrustum( 0.2f, 10.0f ) );
		}

		if( mPointsVboMesh )
			gl::draw( mPointsVboMesh );

		// draw projected sphere
		bool valid = false;
		vec3 drawPos;
		if( mMouseMoveType == K4A_CALIBRATION_TYPE_COLOR ) {
			gl::color( ColorA8u( 255, 255, 0, 255 ) );
			if( mGeomColor )
				drawPos = mKinectCamera->color2dToColor3d( mMouseMovePos, &valid );
			else
				drawPos = mKinectCamera->color2dToDepth3d( mMouseMovePos, &valid );
		}
		else if( mMouseMoveType == K4A_CALIBRATION_TYPE_DEPTH ) {
			gl::color( ColorA8u( 0, 255, 255, 255 ) );
			if( mGeomColor )
				drawPos = mKinectCamera->depth2dToColor3d( mMouseMovePos, &valid );
			else
				drawPos = mKinectCamera->depth2dToDepth3d( mMouseMovePos, &valid );
		}
		if( valid ) {
			gl::drawSphere( drawPos, 0.01f );
			gl::drawLine( vec3( 0 ), drawPos );
		}
	}
}

CINDER_APP( ProjectionApp, RendererGl( RendererGl::Options().msaa( 16 ) ) )
