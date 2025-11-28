# Azure Kinect CinderBlock

CinderBlock for Azure Kinect (K4A) camera integration with Cinder. Support for the Azure Kinect, the [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/) and the [Orbbec Femto Mega](https://www.orbbec.com/products/tof-camera/femto-mega/) (both untested).

<img width="578" height="509" alt="image" src="https://github.com/user-attachments/assets/a7afe540-59da-400e-9f8f-68d69772d1cb" />


## Requirements

- [Azure Kinect SDK v1.4.1](https://download.microsoft.com/download/3/d/6/3d6d9e99-a251-4cf3-8c6a-8e108e960b4b/Azure%20Kinect%20SDK%201.4.1.exe)
- [Body Tracking SDK v1.1.2](https://www.microsoft.com/en-us/download/confirmation.aspx?id=104221) (optional, for skeleton tracking)
- Visual Studio 2022 (v143 toolset) / C++20
- 0.9.4dev Cinder

## Installation

1. Clone/copy to `<Cinder>/blocks/Cinder-AzureKinect`
2. Install the Azure Kinect SDK
3. Install the Body Tracking SDK if using skeleton features

## Project Setup

### Using the Props File

Each sample imports `samples/CinderKinectAzure.props` which defines SDK paths:

```xml
<K4A_SDK_PATH>C:\Program Files\Azure Kinect SDK v1.4.1</K4A_SDK_PATH>
<K4ABT_SDK_PATH>C:\Program Files\Azure Kinect Body Tracking SDK</K4ABT_SDK_PATH>
```

To use in your project:
1. Copy `samples/CinderKinectAzure.props` to your project's `proj/` folder
2. Edit paths if your SDK is installed elsewhere
3. In your `.vcxproj`, add to PropertySheets:
```xml
<Import Project="..\CinderKinectAzure.props" />
```

### Required Libraries

Link against:
- `k4a.lib` - Core SDK
- `k4arecord.lib` - Recording/playback support
- `k4abt.lib` - Body tracking (if using `CinderKinectAzureBodyTracking.h`)

### Include Directories

```
$(K4A_SDK_PATH)\sdk\include
$(K4ABT_SDK_PATH)\sdk\include  // for body tracking
```

### Post-Build DLL Copy

Copy SDK DLLs to output directory:
```
copy /y "$(K4A_SDK_PATH)\sdk\windows-desktop\amd64\release\bin\*.dll" $(OutDir)
```

## Usage

### Basic Camera Setup

```cpp
#include "CinderKinectAzure.h"

// Create camera with resolution and depth mode
auto camera = std::make_unique<cika::Camera>(
    K4A_COLOR_RESOLUTION_720P,
    K4A_DEPTH_MODE_NFOV_UNBINNED
);

// Set coordinate system (default is Kinect-native: Z forward, Y down)
camera->setCoordinateSystem( cika::CoordinateSystem::RIGHT_HANDED_Y_UP ); // OpenGL style
camera->setUnits( cika::Unit::METER );

// Start capture with preprocessing options
camera->startCapture( cika::CaptureFormat().imageColorGeomDepth() );

// In update loop
if( camera->update() ) {
    auto colorImage = camera->getImageColor();
    auto depthImage = camera->getImageDepth();
}
```

### Playback from Recording

```cpp
std::filesystem::path path = "recording.mkv";
auto camera = std::make_unique<cika::Camera>( path, true ); // loop=true
```

### Point Cloud Generation

```cpp
// Enable point cloud preprocessing
camera->startCapture( cika::CaptureFormat().pointsRgbGeomDepth() );

// Get points (XYZ + RGB interleaved floats)
std::vector<float> points( camera->getDepthWidth() * camera->getDepthHeight() * 6 );
uint32_t numPoints = camera->getPointsRgbGeomDepth( points.data(), true );
```

### Body Tracking

```cpp
#include "CinderKinectAzureBodyTracking.h"

auto tracker = std::make_unique<cika::BodyTracker>( camera.get() );

// In update loop
if( tracker->update() ) {
    auto bodies = tracker->getBodies();
    for( const auto& body : bodies ) {
        auto headPos = body.joints[K4ABT_JOINT_HEAD].position;
    }
}
```

### GPU-Accelerated Rendering (CinderKinectAzureGl)

```cpp
#include "CinderKinectAzureGl.h"

// Create GL point cloud renderer
glm::ivec2 depthRes( camera->getDepthWidth(), camera->getDepthHeight() );
auto pointCloudGl = std::make_unique<cika::PointCloudGl>( depthRes );

// Initialize depth table (once after camera setup)
pointCloudGl->updateDepthTable( camera->getDepthTableGeomDepth() );

// In update loop
pointCloudGl->updateColor( camera->getImageColorGeomDepth() );
pointCloudGl->updateDepth( camera->getImageDepth() );

// In draw
pointCloudGl->draw();
```

## Samples

| Sample | Description |
|--------|-------------|
| DepthView | Basic point cloud from CPU |
| DepthViewGPU | GPU-accelerated point cloud using depth table shader |
| ImageCapture | Capture and save frames |
| Projection | 2D/3D coordinate projection |
| Skeleton2D | 2D body tracking overlay |
| Skeleton3D | 3D body tracking visualization |
| MultiCamDepthView | Multiple camera support |
| MultiCamImageCapture | Multi-camera capture |

## File Structure

```
kinect_azure/
├── include/
│   ├── CinderKinectAzure.h          # Core camera API
│   ├── CinderKinectAzureBodyTracking.h  # Body tracking
│   └── CinderKinectAzureGl.h        # GL helpers (decoupled)
├── src/
│   ├── CinderKinectAzure.cpp
│   ├── CinderKinectAzureBodyTracking.cpp
│   └── CinderKinectAzureGl.cpp
└── samples/
    ├── CinderKinectAzure.props      # SDK path configuration
    └── [sample folders]/
```

## Notes

- The GL functionality is separated into `CinderKinectAzureGl.h/.cpp` to allow headless use without OpenGL dependencies.
- IMU data is available via `startImu()`, `updateImu()`, and `getImuSample()`.
- Depth tables (`getDepthTableGeomDepth()`) are lazily computed and cached.
