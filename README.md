# Panomersive Viewer

The Optera™ IMM Panomersive Viewer is intended to demo the IMM Series camera user experience.

## Minimum System Requirements
The following information describes the IMM Panomersive Viewer minimum system requirements:

**Operating System**: Windows 7 or higher

**Low Resolution**
* Video Memory: 1 GB
* System Memory: 4 GB
* Processor: Intel® Core™ i5 2.6 GHz or better

**High Resolution**
* Video Memory: 2 GB
* System Memory: 8 GB
* Processor: Intel Core i7 2.0 GHz or better
    
## Launching the Application
Upon initially launching the Viewer, a static test image is displayed. This image provides a full 3-D scene which is useful for understanding the various views
and PTZ controls available in the application and IMM Series camera.

The following drop-down menus are available:
* Media
* View Layout
* Options
* Help

## Media Menu
Use the Media tab to connect to live camera streams or launch local test files. Changing the source applies to all windows (panoramic and immersive
views).

### OPEN CAMERA STREAM (LIVE)
Opening Camera Stream (Live) accesses the Connect to a Camera dialog. Complete the following fields to access a camera:
1. Enter the Camera IP Address.
2. (Optional) Enter the User-Defined Camera Name.
3. (Optional) Select the Use High-Resolution Streams check box. By default, the camera’s low-resolution mosaic stream is used. If High-Resolution is
selected, each immersive view is displayed in high-resolution.

**NOTE:** Using the High-Resolution option requires a computer with advanced processing capabilities.

4. Click OK to connect to the IMM Series camera.

**NOTE:** You can change the Camera’s tilt angle by selecting Adjust Camera tilt angle from the Options menu.

### OPEN LOCAL TEST FILES
Launches a sample scene for viewing a panoramic view and experiencing the immersive views.

## View Layout Menu
The View Layout menu provides options for viewing different combinations of panoramic and immersive views.

**NOTE:** Only the Immersive view allows you to PTZ in the different areas of the scene. The panoramic view is intended to give you persistent situational
awareness at a glance. It also provides an overview context for the immersive views.

**Optimize Panoramic for Pelco 270 Cameras:** Displays the 270 degree camera panoramic view with smoothed edges.

**Immersive (Default):** Displays one immersive view in which you can PTZ across the entire scene.

**Side-by-Side Immersive:** Divides the main window into two side-by-side PTZ views.

**Panoramic:** Displays a full panoramic view that fills the full window. This panorama is not PTZ enabled.

**Panoramic over Immersive:** Displays a panoramic view above a single immersive view.

**Panoramic over Side-by-Side Immersives:** Displays a panoramic view above two immersive, PTZ enabled views.

**Multiscreen Demo Mode (Press Esc to exit):** If you have only one monitor, the immersive view displays in full screen mode. If you have multiple
monitors, the panoramic view displays on the primary monitor, and every other monitor displays an independent immersive view. For example, if you have
four monitors, one panoramic view and three immersive views will display.

### VIRTUAL PTZ CONTROL
Allows you to control the pan/tilt/zoom (PTZ) and position tracking using a mouse.

#### PAN AND TILT
Pan and tilt are controlled by clicking the left mouse button and dragging left/right (pan) or up/down (tilt). The mouse controls the direction that the content
should move, rather than the direction of the virtual camera view. For example, dragging the mouse left will move the content to the left and the view to the
right; dragging the mouse up will tilt the content up by tilting the view area down.

Double-clicking the left mouse button re-centers the view. The area in which you click moves to the center of the view, and the mouse automatically moves
to the center. Therefore, the image remains over the selected point.

Bring the view back to the default starting position by pressing C.

#### ZOOM
Zooming into the scene is controlled by clicking the left mouse button and rolling the mouse wheel up/down to select an area.

When zooming in with the mouse wheel, the view will automatically zoom over the position in which the mouse is placed.

#### POSITION TRACKING
Position tracking is controlled by holding down the right mouse button, which displays the position tracking indicator (red square) over the selected point, as
well as the spherical and screen coordinates. When multiple views are displayed, the position tracking indicator is draw in all views, allowing you to locate
the same position in each of the independent PTZ immersive views and the panoramic view.

Pressing the S key on your keyboard while the position tracking indicator is drawn in all views, allows the spherical and screen coordinates to display on the
tracked point in both the panoramic and immersive views.

## Options Menu
The Options menu provides different user definable PTZ options. In addition, you can display the frame count, and render and adjust the camera tilt.

**NOTE:** The Options menu is designed for developers who intend to use this demo application as a toolkit to create VMS plug-ins.

**No PTZ Limiting:** PTZ motions are unconstrained by the available video stream.

**Limit view center to data area:** Prevents the center of the view area from moving outside of the field of view. You will notice that data continuously
displays in the center of your screen. The demo application prevents you from panning when the center of the view reaches the edge of the field of view
area.

**Limit whole view to data area (Default):** You cannot pan outside of the immersive scene. When this mode is selected, the Advanced view limiting
options menu becomes available. Select one or multiple of the following options to change the panning functionality.
* **Auto zoom-in upon approaching data edge:** Automatically zooms in when the mouse reaches the center of the scene. In this mode, you can
continue to pan throughout the scene, as desired.

* **Auto zoom-out upon leaving data edge:** Automatically zooms in, which effectively gives you more room to pan, thereby allowing you to
continue to pan to the side without leaving the field of view.

* **Auto pan/tilt on zoom out from data edge (Default):** Automatically pans/tilts away from the data edge so you can successfully zoom out. With
this mode turned off, you cannot zoom out because that would widen the view, moving it beyond the data area.

**Show Frame-Count Overlay:** Toggles the number of frames processed in each view. This is particularly useful for trouble-shooting a live camera
connection.

**Rendering Mode:** Select between three different rendering modes (particularly of interest to developers):
* **Direct (Default):** Renders views directly to the window through OpenGL. This offers the best rendering performance, but may not be viable for
some VMS platforms where rendered frames may need to be passed through other code for post-processing and display.

* **Framebuffer:** Renders views to a memory buffer, and then displays by the application through normal Windows drawing (BitBlt) oper- ations).

* **Raw Video:** Bypasses the immersive view toolkit entirely. Decompressed frames (from VLC) are simply drawn to the window without any de-
warping or PTZ functionality. When multiple views are open, each view will receive one stream from the camera.

**Optimization Bias:** Select between three different optimization modes.
* **Image Quality:** Optimizes for image quality. This option is intended for PCs with fast processors, requiring significant processing power
particularly at high resolutions.

* **Processing Resources:** Optimizes for speed. Minor rendering artifacts may be present in the panoramic view or when zoomed out in the
immersive view. Artifacts appear as jagged edges or similar aliasing issues in areas of fine detail within in the scene. However, zooming in on the
affected area eliminates rendering artifacts.

* **Automatic (Default):** Automatically chooses between quality and performance options. Image quality is the default if the host PC has a capable
processor.

**NOTES:** When using Automatic mode, you can determine the actual mode by enabling Show Frame-Count Overlay.

### ADJUST CAMERA TILT
You can get a more natural orientation of the image in an immersive view by adjusting the tilt angle bar. First pan the image in the immersive view all the
way to the right or all the way to the left. Then adjust the tilt angle so vertical edges in the view appear vertical and horizon lines appear horizontal.

Cameras can be mounted with a tilt angle. For example, roof-top cameras are typically pointing down at some angle. When the tilt angle is not 0 (horizontal),
the objects in the view appear properly vertical only in the center of the view. At the left and right edges of the view, objects appear to be leaning sideways.

You can correct this effect by rotating the view as the camera pans left and right. From the Options menu, select Adjust Camera Tilt, click Override the
camera’s reported tilt angle, and then move the horizontal bar vertically for the preferred camera tilt.

- - - -

# Immersive Experience Toolkit

## Overview
The purpose of this section is to explain what the Immersive Experience Toolkit is, and how it is intended to be used within VMS software. 

## Background
Pelco’s Optera, multi-sensor cameras each produce multiple video streams -- 3 to 6 depending on the model.  These streams are of two types: full-resolution component streams (2 to 5 of them), showing unique, non-overlapping portions of the camera’s total field of view; and a lower-resolution “mosaic” stream, which combines all of the component streams into a single large, but lower than full resolution image.

Although all streams from these cameras can be viewed independently with standard video client software, the individual streams provide a warped, incomplete view of the scene.  The intended viewing experience for these cameras requires special de-warping software that can combine the multiple component streams into a single, “immersive experience” presentation.  This software has been packaged into the Immersive Experience Toolkit so that it can be incorporated into a wide variety of VMS client solutions with a minimum of engineering effort.

## The Immersive Experience Toolkit
The Immersive Experience Toolkit (or ImExTk, “I-Mex-T-K”) contains the special de-warping software that is needed to provide the full “Panomersive” viewing experience for Pelco’s Optera cameras.  The purpose of the ImExTk is to make it as easy as possible to add full support of these new cameras to any VMS software, whether Pelco’s own or 3rd-party solutions.

At the core of the toolkit is the ImExTk DLL, which is intended to be directly usable, as-is, by VMS software.  For a detailed programmer’s guide to this DLL, see “ImExTk DLL Interface Guide.html,” but as a high-level overview, is major features are:
* Combines multiple streams into a single, de-warped view with controllable pan, tilt and zoom.
* Provides multiple view types: immersive (virtual PTZ) or situational awareness (Mercator or “fisheye” projections, showing all video data at once).
* Supports multiple, simultaneous views of one or more cameras, each with independent virtual PTZ control.
* Includes functions for mapping points between view and world coordinates, so that positions can be correlated between multiple views of the same camera.
* Can be configured to automatically limit virtual PTZ controls to the camera’s field of view.
* Utilizes OpenGL for hardware-accelerated rendering directly into video hardware or into client-supplied RAM buffers.
* Has a simple ‘C’ language programming interface, enabling it to be used by applications written in a variety of languages including C++, C#, or Java.

### Relationship to VMS Software
Figure 1 below shows how the ImExTk software is intended to fit into a larger VMS product.   Specifically, it shows how video data is streamed from a video frame source (e.g., a camera, or a previously stored recording) into the ImExTk DLL, and rendered into a Window in the VMS’s client GUI using OpenGL to draw directly into video memory.  It also shows how the GUI can simultaneously send “View Control” commands to the ImExTk DLL in order to achieve virtual PTZ of the resulting view.

Note that while this diagram depicts a single view, the ImExTk DLL actually supports multiple frame sources and multiple views of each frame source (up to resource and performance limits of the host computer) so that one can create multiple, independently controllable views of one or more Optera cameras.
 
Figure 1: Immersive Experience Architecture
The key take-away from Figure 1 is that the ImExTk software is primarily responsible for view generation, while the VMS software is responsible for accessing, decoding and time-synchronizing all video streams and then providing that video data (along with certain metadata, described later) to the ImExTk software.  Further, since the View Generator relies on OpenGL, the VMS is responsible for some amount of OpenGL setup and coordination as described later in this document.

## Toolkit Design Considerations
### Portability
The ImExTk needs to be useful to any and all VMS providers that support Pelco’s multi-sensor cameras.  This obviously includes Pelco’s own VMS software, but it also includes 3rd-party solutions that run on a variety of platforms and are implemented in a variety of programming languages.

In particular, the software that makes up the ImExTk DLL needs to be as portable as possible.  In practice this means that it should be easily compiled for on a variety of platforms (e.g., Windows, Linux , Mac,).  To achieve that goal, the following design criteria apply to the ViewGen library:
* Internally developed in C/C++ , with ‘public’ interface written in C, to allow easy binding to most other languages (e.g., C#, Java)
* Minimum dependency on 3rd-party code.  The ViewGen library depends on the following 3rd-party components, which are each available on a wide variety of platforms.
  * C++11 standard libraries
  * OpenGL: libopengl and the OpenGL Extension Wrangler (glew) library

#### Relationship to Pelco SDK
In order to meet its portability/reusability goals, the ImExTk is entirely independent of the Pelco SDK.  However, it is expected that the functionality of the ImExTk will ultimately be provided as part of the Pelco SDK.  That would most likely involve the creation of a thin SDK “wrapper” around the ViewGen library, so that its public interface would be consistent with other SDK components and would make use of SDK-specific data types as appropriate.  Inside this wrapper, however, it would ideally use the ImExTk code as-is, in order to make it easy to incorporate future updates of the ImExTk into the SDK.

### OpenGL Hardware Acceleration
In order for the ViewGen library to efficiently generate multiple de-warped views from multiple high-resolution video streams, it needs to be able to use graphics acceleration hardware on the client GUI.  To that end, OpenGL was chosen as the most ubiquitous, cross-platform interface available for that purpose.

However, while OpenGL itself is very portable, the specifics of creating an OpenGL context for a particular hardware/software environment are very much platform-specific, so this design choice does force some OpenGL requirements onto the VMS code.

### View Layout Metadata
Multi-sensor camera models vary in their number of component video streams as well as in how each stream is intended to be viewed relative to other streams from the same camera.  For example, one stream might be intended to be rendered in a specific portion of the left side of the immersive view, while another stream is the rendered on the right.  Moreover, cameras can produce “mosaic” streams, from which the ImExTk must extract multiple sub-frames, each rendered to a different portion of the immersive view.

The VMS software must access this metadata and use it to configure the target View Generator object.  The camera makes this data available in a number of ways, but it is up to the VMS to store this data along with archived recordings so that it is always available for immersive playback of recorded content. One of the ways the camera provides this data is by embedding it into H264 user-data packets, so it is expected that most VMS recorders will be able to automatically store this extra layout information without any changes to their software.

Note that the VMS should not need to be aware of the format of this data.  The ImExTk provides functions to facilitate the extraction of this layout metadata from an H264 stream into simple character strings that the VMS.  The VMS can then supply these metadata strings along with decoded video frames to the ImExTk without ever needing to know anything about the string’s contents.

## VMS Requirements
The VMS requirements have been described indirectly already, but to summarize, the VMS must support the following:
* OpenGL: VMS code is expected to be able to create all necessary OpenGL contexts using whatever platform/language-specific are required – e.g., wgl for windows.
  * Certain ImExTk DLL functions must be called with a currently selected OpenGL context. 
  * Thread-safety between multiple OpenGL contexts is the responsibility of the VMS.
* H264 video transmission and decoding:  While the ImExTk provides some functionality for parsing H264 user-data packets, it otherwise does not deal with compressed video.  The VMS is expected to handle all transmission and decoding of video data.
* Time-synchronization of multiple video streams: The decoded video data frames that the VMS provides to the ImExTk DLL originate from multiple RTP streams.  It is the VMS’s responsibility to perform time-based synchronization between these streams so that the video frames from each stream are rendered together at the appropriate time.
