#pragma once

#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/Vao.h"
#include "cinder/gl/Batch.h"

#include "CinderKinectAzure.h"

namespace cika {

//! Helper class for GPU-accelerated point cloud rendering using depth table lookups
class PointCloudGl {
  public:
	//! Constructs a point cloud renderer for the given depth resolution
	PointCloudGl( glm::ivec2 depthRes );
	~PointCloudGl() = default;

	//! Updates the depth table texture from the camera's precomputed depth table
	void updateDepthTable( const cinder::Surface32f& depthTable );
	//! Updates the depth image texture (16-bit depth values)
	void updateDepth( const cinder::Channel16u& depthImage );
	//! Updates the color image texture (BGRA or warped to depth geometry)
	void updateColor( const cinder::Surface8u& colorImage );

	//! Draws the point cloud using the current depth/color/table textures
	void draw();

	//! Returns the depth texture (GL_TEXTURE_RECTANGLE, R16UI)
	cinder::gl::TextureRef getDepthTexture() const { return mDepthTex; }
	//! Returns the color texture (GL_TEXTURE_RECTANGLE, RGBA8)
	cinder::gl::TextureRef getColorTexture() const { return mColorTex; }
	//! Returns the depth table texture (GL_TEXTURE_RECTANGLE, RGB32F)
	cinder::gl::TextureRef getDepthTableTexture() const { return mDepthTableTex; }
	//! Returns the shader used for rendering
	cinder::gl::GlslProgRef getShader() const { return mShader; }

	//! Sets a custom shader (must have uniforms: uTexDepthBuffer, uTexColorBuffer, uDepthTable)
	void setShader( const cinder::gl::GlslProgRef& shader );

  private:
	void createDefaultShader();

	glm::ivec2					mDepthRes;
	cinder::gl::TextureRef		mDepthTex;
	cinder::gl::TextureRef		mColorTex;
	cinder::gl::TextureRef		mDepthTableTex;
	cinder::gl::VboMeshRef		mPointsMesh;
	cinder::gl::GlslProgRef		mShader;
};

//! Helper class for efficiently uploading Kinect images to GL textures
class ImageGl {
  public:
	//! Image type determines the texture format
	enum class Type {
		COLOR_BGRA,		//!< 8-bit BGRA color image
		DEPTH_16U,		//!< 16-bit unsigned depth image
		IR_16U,			//!< 16-bit unsigned IR image
		DEPTH_32F		//!< 32-bit float depth image (for depth table)
	};

	//! Constructs an image helper for the given resolution and type
	ImageGl( glm::ivec2 res, Type type, bool useTextureRectangle = true );
	~ImageGl() = default;

	//! Updates the texture from an 8-bit BGRA surface
	void update( const cinder::Surface8u& surface );
	//! Updates the texture from a 16-bit channel (depth or IR)
	void update( const cinder::Channel16u& channel );
	//! Updates the texture from a 32-bit float surface (depth table)
	void update( const cinder::Surface32f& surface );

	//! Returns the GL texture
	cinder::gl::TextureRef getTexture() const { return mTexture; }

	//! Returns the resolution
	glm::ivec2 getResolution() const { return mResolution; }

  private:
	cinder::gl::TextureRef	mTexture;
	glm::ivec2				mResolution;
	Type					mType;
};

//! Helper class for rendering colored point clouds from CPU data
class PointCloudVboGl {
  public:
	//! Constructs a VBO-based point cloud renderer with capacity for maxPoints
	PointCloudVboGl( size_t maxPoints );
	~PointCloudVboGl() = default;

	//! Updates the point cloud from interleaved XYZ+RGB float data (6 floats per point)
	//! Returns the number of points updated
	size_t update( const float* xyzRgbData, size_t numPoints );

	//! Updates directly from camera using getPointsRgbGeomDepth or getPointsRgbGeomColor
	size_t updateFromCamera( Camera& camera, bool useColorGeometry = false, bool finitePointsOnly = true );

	//! Draws the point cloud
	void draw();

	//! Returns the VBO mesh for custom rendering
	cinder::gl::VboMeshRef getVboMesh() const { return mVboMesh; }

	//! Returns the current number of valid points
	size_t getNumPoints() const { return mNumPoints; }

  private:
	cinder::gl::VboMeshRef	mVboMesh;
	size_t					mMaxPoints;
	size_t					mNumPoints;
};

} // namespace cika
