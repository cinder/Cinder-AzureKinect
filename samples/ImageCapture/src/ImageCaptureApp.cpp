#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"

#include "CinderKinectAzure.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const int DRAW_BORDER = 5;

class ImageCaptureApp : public App {
  public:
	void setup() override;
	void update() override;
	void draw() override;

	std::unique_ptr<cika::Camera>		mKinectCamera;
	gl::TextureRef						mColorTex, mInfraredTex, mDepthTex;
	ci::gl::GlslProgRef					mHeatMapGlsl, mGrayScaleGlsl;
};

void ImageCaptureApp::setup()
{
	//cika::setLogMessageLevel( k4a_log_level_t::K4A_LOG_LEVEL_TRACE );

	vector<cika::Device> devices = cika::Device::getDeviceList();
	for( auto& device : devices )
		console() << "Detected Device: " << device.getSerialNumber() << std::endl;

	mKinectCamera = make_unique<cika::Camera>( K4A_COLOR_RESOLUTION_720P, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_FRAMES_PER_SECOND_30 );
	mKinectCamera->startCapture();

	// heat map shader
	mHeatMapGlsl = gl::GlslProg::create( loadAsset( "drawTexture.vert" ), loadAsset( "heatMap.frag" ) );
	vec2 depthRangeNormalized = vec2( mKinectCamera->getDepthRange() ) / vec2( 65535.0f );
	mHeatMapGlsl->uniform( "uDepthMin", depthRangeNormalized.x );
	mHeatMapGlsl->uniform( "uDepthMax", depthRangeNormalized.y );

	// infrared shader
	mGrayScaleGlsl = gl::GlslProg::create( loadAsset( "drawTexture.vert" ), loadAsset( "grayScale.frag" ) );
	vec2 infraredRangeNormalized = vec2( mKinectCamera->getInfraredRange() ) / vec2( 65535.0f );
	mGrayScaleGlsl->uniform( "uScaleMin", infraredRangeNormalized.x );
	mGrayScaleGlsl->uniform( "uScaleMax", infraredRangeNormalized.y );

	// textures
	mColorTex = gl::Texture::create( nullptr, GL_RGB, mKinectCamera->getColorWidth(), mKinectCamera->getColorHeight(), gl::Texture::Format().loadTopDown() );
	mInfraredTex = gl::Texture::create( Channel16u( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() ), gl::Texture::Format().loadTopDown() );
	mDepthTex = gl::Texture::create( Channel16u( mKinectCamera->getDepthWidth(), mKinectCamera->getDepthHeight() ), gl::Texture::Format().loadTopDown() );

	setWindowSize( std::max( mKinectCamera->getColorWidth(), mKinectCamera->getDepthWidth() * 2) + DRAW_BORDER, mKinectCamera->getColorHeight() + mKinectCamera->getDepthHeight() + DRAW_BORDER );
}

void ImageCaptureApp::update()
{
	try {
		if( mKinectCamera && mKinectCamera->update() ) {
			mColorTex->update( mKinectCamera->getImageColor() );
			mInfraredTex->update( mKinectCamera->getImageInfrared() );
			mDepthTex->update( mKinectCamera->getImageDepth() );
		}
	}
	catch(...) {
		CI_LOG_E( "Error grabbing new frame" );
	}
}

void ImageCaptureApp::draw()
{
	int32_t nativeDrawWidth = std::max( mKinectCamera->getColorWidth(), mKinectCamera->getDepthWidth() * 2) + DRAW_BORDER;
	int32_t nativeDrawHeight = mKinectCamera->getColorHeight() + mKinectCamera->getDepthHeight() + DRAW_BORDER;
	float drawScale = std::min( getWindowWidth() / (float)nativeDrawWidth, getWindowHeight() / (float)nativeDrawHeight ); 

	gl::clear( Color( 0.1f, 0.1f, 0.1f ) );

	gl::ScopedModelMatrix mtx_;

	if( mColorTex ) {
		gl::draw( mColorTex, Rectf( mColorTex->getBounds()).scaled( drawScale ) );
		gl::translate( 0, mKinectCamera->getColorHeight() * drawScale + DRAW_BORDER ); // offset other images
	}

	if( mInfraredTex && mDepthTex ) {
		{
			gl::ScopedGlslProg glslHeat( mHeatMapGlsl );
			gl::ScopedTextureBind texDepth( mDepthTex );
			gl::drawSolidRect( Rectf( 0, 0, mDepthTex->getWidth() * drawScale, mDepthTex->getHeight() * drawScale ), vec2( 0, 0 ), vec2( 1, 1 ) );
			gl::translate( DRAW_BORDER, 0 );
		}

		{
			gl::ScopedGlslProg glslGray( mGrayScaleGlsl );
			gl::ScopedTextureBind texInfra( mInfraredTex );
			Rectf drawRect = Rectf( mDepthTex->getWidth() * drawScale, 0, (mInfraredTex->getWidth() + mDepthTex->getWidth()) * drawScale, mInfraredTex->getHeight() * drawScale  );
			gl::drawSolidRect( drawRect, vec2( 0, 0 ), vec2( 1, 1 ) );
		}
	}
}

CINDER_APP( ImageCaptureApp, RendererGl )
