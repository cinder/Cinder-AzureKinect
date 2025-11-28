#include "CinderKinectAzureGl.h"

#include "cinder/gl/gl.h"
#include "cinder/Log.h"

using namespace cinder;

namespace cika {

// Default vertex shader for depth-table based point cloud rendering
static const char* POINTCLOUD_VERT_SHADER = R"(
#version 330 core

uniform mat4 ciModelViewProjection;
uniform usampler2DRect uTexDepthBuffer;
uniform sampler2DRect uTexColorBuffer;
uniform sampler2DRect uDepthTable;

layout(location = 0) in vec2 ciPosition;

out vec4 vColor;

void main() {
	vec2 texCoord = ciPosition;

	// Sample depth value (16-bit unsigned, stored in red channel)
	float depth = float(texelFetch(uTexDepthBuffer, ivec2(texCoord)).r);

	// Sample the depth table ray direction
	vec3 ray = texelFetch(uDepthTable, ivec2(texCoord)).rgb;

	// Reconstruct 3D position: ray * depth (negate Y to match CPU code)
	vec3 position = vec3(ray.x * depth, -ray.y * depth, ray.z * depth);

	// Sample color (Cinder handles BGRX->RGBA conversion on upload)
	vColor = texelFetch(uTexColorBuffer, ivec2(texCoord));

	// Discard invalid points (depth == 0)
	if (depth == 0.0) {
		gl_Position = vec4(0.0, 0.0, -1000.0, 1.0); // Move off-screen
	} else {
		gl_Position = ciModelViewProjection * vec4(position, 1.0);
	}
}
)";

static const char* POINTCLOUD_FRAG_SHADER = R"(
#version 330 core

in vec4 vColor;
out vec4 oColor;

void main() {
	oColor = vColor;
}
)";

// ============================================================================
// PointCloudGl Implementation
// ============================================================================

PointCloudGl::PointCloudGl( glm::ivec2 depthRes )
	: mDepthRes( depthRes )
{
	// Create depth texture (16-bit unsigned integer, rectangle texture)
	gl::Texture2d::Format depthFormat;
	depthFormat.setTarget( GL_TEXTURE_RECTANGLE );
	depthFormat.setInternalFormat( GL_R16UI );
	depthFormat.setDataType( GL_UNSIGNED_SHORT );
	depthFormat.setMagFilter( GL_NEAREST );
	depthFormat.setMinFilter( GL_NEAREST );
	depthFormat.loadTopDown( true );
	mDepthTex = gl::Texture2d::create( depthRes.x, depthRes.y, depthFormat );

	// Create color texture (BGRA, rectangle texture)
	gl::Texture2d::Format colorFormat;
	colorFormat.setTarget( GL_TEXTURE_RECTANGLE );
	colorFormat.setInternalFormat( GL_RGBA8 );
	colorFormat.setMagFilter( GL_LINEAR );
	colorFormat.setMinFilter( GL_LINEAR );
	colorFormat.loadTopDown( true );
	mColorTex = gl::Texture2d::create( depthRes.x, depthRes.y, colorFormat );

	// Create depth table texture (RGB32F for ray directions, rectangle texture)
	gl::Texture2d::Format tableFormat;
	tableFormat.setTarget( GL_TEXTURE_RECTANGLE );
	tableFormat.setInternalFormat( GL_RGB32F );
	tableFormat.setDataType( GL_FLOAT );
	tableFormat.setMagFilter( GL_NEAREST );
	tableFormat.setMinFilter( GL_NEAREST );
	tableFormat.loadTopDown( true );
	mDepthTableTex = gl::Texture2d::create( depthRes.x, depthRes.y, tableFormat );

	// Create VBO mesh with positions corresponding to each pixel
	auto layout = gl::VboMesh::Layout().usage( GL_STATIC_DRAW ).attrib( geom::Attrib::POSITION, 2 );
	mPointsMesh = gl::VboMesh::create( depthRes.x * depthRes.y, GL_POINTS, { layout } );

	std::vector<glm::vec2> positions( depthRes.x * depthRes.y );
	for( int32_t y = 0, idx = 0; y < depthRes.y; ++y ) {
		for( int32_t x = 0; x < depthRes.x; ++x, ++idx ) {
			positions[idx] = glm::vec2( static_cast<float>( x ), static_cast<float>( y ) );
		}
	}
	mPointsMesh->getVertexArrayVbos()[0]->copyData( positions.size() * sizeof( glm::vec2 ), positions.data() );

	createDefaultShader();
}

void PointCloudGl::createDefaultShader()
{
	try {
		mShader = gl::GlslProg::create( POINTCLOUD_VERT_SHADER, POINTCLOUD_FRAG_SHADER );
		mShader->uniform( "uTexDepthBuffer", 0 );
		mShader->uniform( "uTexColorBuffer", 1 );
		mShader->uniform( "uDepthTable", 2 );
	}
	catch( const gl::GlslProgCompileExc& e ) {
		CI_LOG_E( "PointCloudGl shader compile error: " << e.what() );
		throw;
	}
}

void PointCloudGl::setShader( const gl::GlslProgRef& shader )
{
	mShader = shader;
}

void PointCloudGl::updateDepthTable( const Surface32f& depthTable )
{
	if( depthTable.getWidth() != mDepthRes.x || depthTable.getHeight() != mDepthRes.y ) {
		CI_LOG_W( "Depth table size mismatch" );
		return;
	}
	mDepthTableTex->update( depthTable );
}

void PointCloudGl::updateDepth( const Channel16u& depthImage )
{
	if( depthImage.getWidth() != mDepthRes.x || depthImage.getHeight() != mDepthRes.y ) {
		CI_LOG_W( "Depth image size mismatch" );
		return;
	}
	mDepthTex->update( depthImage );
}

void PointCloudGl::updateColor( const Surface8u& colorImage )
{
	if( colorImage.getWidth() != mDepthRes.x || colorImage.getHeight() != mDepthRes.y ) {
		CI_LOG_W( "Color image size mismatch" );
		return;
	}
	mColorTex->update( colorImage );
}

void PointCloudGl::draw()
{
	gl::ScopedGlslProg shaderScope( mShader );
	gl::context()->setDefaultShaderVars();

	gl::ScopedTextureBind tex0( mDepthTex, 0 );
	gl::ScopedTextureBind tex1( mColorTex, 1 );
	gl::ScopedTextureBind tex2( mDepthTableTex, 2 );

	gl::draw( mPointsMesh );
}

// ============================================================================
// ImageGl Implementation
// ============================================================================

ImageGl::ImageGl( glm::ivec2 res, Type type, bool useTextureRectangle )
	: mResolution( res )
	, mType( type )
{
	gl::Texture2d::Format format;
	if( useTextureRectangle ) {
		format.setTarget( GL_TEXTURE_RECTANGLE );
	}
	format.loadTopDown( true );

	switch( type ) {
		case Type::COLOR_BGRA:
			format.setInternalFormat( GL_RGBA8 );
			format.setMagFilter( GL_LINEAR );
			format.setMinFilter( GL_LINEAR );
			break;
		case Type::DEPTH_16U:
		case Type::IR_16U:
			format.setInternalFormat( GL_R16UI );
			format.setDataType( GL_UNSIGNED_SHORT );
			format.setMagFilter( GL_NEAREST );
			format.setMinFilter( GL_NEAREST );
			break;
		case Type::DEPTH_32F:
			format.setInternalFormat( GL_RGB32F );
			format.setDataType( GL_FLOAT );
			format.setMagFilter( GL_NEAREST );
			format.setMinFilter( GL_NEAREST );
			break;
	}

	mTexture = gl::Texture2d::create( res.x, res.y, format );
}

void ImageGl::update( const Surface8u& surface )
{
	if( mType != Type::COLOR_BGRA ) {
		CI_LOG_W( "ImageGl::update(Surface8u) called on non-color texture" );
		return;
	}
	mTexture->update( surface );
}

void ImageGl::update( const Channel16u& channel )
{
	if( mType != Type::DEPTH_16U && mType != Type::IR_16U ) {
		CI_LOG_W( "ImageGl::update(Channel16u) called on non-16bit texture" );
		return;
	}
	glBindTexture( mTexture->getTarget(), mTexture->getId() );
	glTexSubImage2D( mTexture->getTarget(), 0, 0, 0, channel.getWidth(), channel.getHeight(),
					 GL_RED_INTEGER, GL_UNSIGNED_SHORT, channel.getData() );
}

void ImageGl::update( const Surface32f& surface )
{
	if( mType != Type::DEPTH_32F ) {
		CI_LOG_W( "ImageGl::update(Surface32f) called on non-32f texture" );
		return;
	}
	mTexture->update( surface );
}

// ============================================================================
// PointCloudVboGl Implementation
// ============================================================================

PointCloudVboGl::PointCloudVboGl( size_t maxPoints )
	: mMaxPoints( maxPoints )
	, mNumPoints( 0 )
{
	// Create VBO with interleaved position (3 floats) and color (3 floats)
	auto layout = gl::VboMesh::Layout()
		.usage( GL_STREAM_DRAW )
		.interleave()
		.attrib( geom::Attrib::POSITION, 3 )
		.attrib( geom::Attrib::COLOR, 3 );

	mVboMesh = gl::VboMesh::create( static_cast<uint32_t>( maxPoints ), GL_POINTS, { layout } );
}

size_t PointCloudVboGl::update( const float* xyzRgbData, size_t numPoints )
{
	numPoints = std::min( numPoints, mMaxPoints );
	mNumPoints = numPoints;

	if( numPoints > 0 ) {
		float* ptr = static_cast<float*>( mVboMesh->getVertexArrayVbos()[0]->mapWriteOnly() );
		std::memcpy( ptr, xyzRgbData, numPoints * 6 * sizeof( float ) );
		mVboMesh->getVertexArrayVbos()[0]->unmap();
	}

	mVboMesh->updateNumVertices( static_cast<uint32_t>( numPoints ) );
	return numPoints;
}

size_t PointCloudVboGl::updateFromCamera( Camera& camera, bool useColorGeometry, bool finitePointsOnly )
{
	float* ptr = static_cast<float*>( mVboMesh->getVertexArrayVbos()[0]->mapWriteOnly() );

	size_t numPoints;
	if( useColorGeometry ) {
		numPoints = camera.getPointsRgbGeomColor( ptr, finitePointsOnly );
	} else {
		numPoints = camera.getPointsRgbGeomDepth( ptr, finitePointsOnly );
	}

	mVboMesh->getVertexArrayVbos()[0]->unmap();

	numPoints = std::min( numPoints, mMaxPoints );
	mNumPoints = numPoints;
	mVboMesh->updateNumVertices( static_cast<uint32_t>( numPoints ) );

	return numPoints;
}

void PointCloudVboGl::draw()
{
	if( mNumPoints > 0 ) {
		gl::draw( mVboMesh );
	}
}

} // namespace cika
