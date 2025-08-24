/*
 *	PlayStation 4 Camera Capture standalone application
 *	Converted from Cinder app to standalone C++ application
 *	Copyright (C) 2013,2014 Antonio Jose Ramos Marquez (aka bigboss) @psxdev
 *on twitter
 *
 *  Repository https://github.com/bigboss-ps3dev/PS4EYECam
 *  some parts are based on PS3EYECamera driver
 *https://github.com/inspirit/PS3EYEDriver some parts were commited to ps4eye
 *https://github.com/ps4eye/ps4eye
 *
 *  This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public Licence as published by
 *	the Free Software Foundation; either version 2 of the Licence, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *	GNU General Public Licence for more details.
 *
 *	You should have received a copy of the GNU General Public Licence
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
 *
 */

#include "ps4eye.h"
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <thread>

using namespace std;
using namespace cv;

class PS4EYECapture {
public:
  PS4EYECapture();
  ~PS4EYECapture();

  bool initialize();
  void run();
  void shutdown();

private:
  void eyeUpdateThreadFn();
  void processFrame();
  void handleKeyInput();

  ps4eye::PS4EYECam::PS4EYERef eye;
  bool mShouldQuit;
  std::thread mThread;

  uint8_t *frame_rgb_left;
  uint8_t *frame_rgb_right;
  Mat frameLeft, frameRight;

  // FPS measurement
  chrono::steady_clock::time_point lastTime;
  uint32_t frameCount;
  float fps;

  // Settings
  bool showRightCamera;
  bool saveFrames;
  int frameCounter;
};

// Global instance for signal handling
PS4EYECapture *g_capture = nullptr;

void signalHandler(int signum) {
  cout << "\nInterrupt signal (" << signum << ") received. Shutting down..."
       << endl;
  if (g_capture) {
    g_capture->shutdown();
  }
  exit(signum);
}

// Color conversion functions from original code
void convert_opencv_to_RGB(uint8_t *in, uint8_t *out, int size_x, int size_y) {
  Mat yuv(size_y, size_x, CV_8UC2, in);
  Mat rgb(size_y, size_x, CV_8UC3, out);
  cvtColor(yuv, rgb, COLOR_YUV2BGR_YUY2);
}

PS4EYECapture::PS4EYECapture()
    : mShouldQuit(false), frame_rgb_left(nullptr), frame_rgb_right(nullptr),
      frameCount(0), fps(0.0f), showRightCamera(false), saveFrames(false),
      frameCounter(0) {
  lastTime = chrono::steady_clock::now();
}

PS4EYECapture::~PS4EYECapture() { shutdown(); }

bool PS4EYECapture::initialize() {
  using namespace ps4eye;

  cout << "Initializing PS4 Eye Camera..." << endl;

  // List out the devices
  vector<PS4EYECam::PS4EYERef> devices(PS4EYECam::getDevices());
  cout << "Found " << devices.size() << " camera(s)" << endl;

  if (devices.empty()) {
    cout << "No PlayStation Camera device found on this system" << endl;
    return false;
  }

  eye = devices.at(0);

  // Set firmware path (you may need to adjust this path)
  eye->firmware_path = "firmware.bin"; // Adjust path as needed

  // Try firmware upload - this might be necessary for proper operation
  cout << "Uploading firmware..." << endl;
  try {
    eye->firmware_upload();
    cout << "Firmware upload completed successfully" << endl;
  } catch (...) {
    cout << "Firmware upload failed or not needed" << endl;
  }

  // Initialize camera
  // mode 0: 60,30,15,8 fps 1280x800
  // mode 1: 120,60,30,15,8 fps 640x400
  // mode 2: 240,120,60,30 fps 320x192
  if (!eye->init(0, 60)) { // mode 1, 60 fps
    cout << "Camera initialization failed" << endl;
    return false;
  }

  cout << "Camera initialized: " << eye->getWidth() << "x" << eye->getHeight()
       << " @ " << (int)eye->getFrameRate() << " fps" << endl;

  // Allocate frame buffers
  int frameSize = eye->getWidth() * eye->getHeight() * 3;
  frame_rgb_left = new uint8_t[frameSize];
  frame_rgb_right = new uint8_t[frameSize];

  frameLeft = Mat(eye->getHeight(), eye->getWidth(), CV_8UC3, frame_rgb_left);
  frameRight = Mat(eye->getHeight(), eye->getWidth(), CV_8UC3, frame_rgb_right);

  memset(frame_rgb_left, 0, frameSize);
  memset(frame_rgb_right, 0, frameSize);

  eye->rightflag = 0; // Start with left camera
  eye->start();

  // Create and launch the update thread
  mThread = thread(&PS4EYECapture::eyeUpdateThreadFn, this);

  cout << "Camera started successfully!" << endl;
  cout << "\nControls:" << endl;
  cout << "  'r' - Toggle right camera view" << endl;
  cout << "  's' - Save current frame" << endl;
  cout << "  'f' - Toggle continuous frame saving" << endl;
  cout << "  'q' - Quit" << endl;
  cout << "  ESC - Quit" << endl;

  return true;
}

void PS4EYECapture::eyeUpdateThreadFn() {
  while (!mShouldQuit) {
    bool res = ps4eye::PS4EYECam::updateDevices();
    if (!res)
      break;
    this_thread::sleep_for(chrono::milliseconds(1));
  }
}

void PS4EYECapture::processFrame() {
  if (!eye)
    return;

  bool isNewFrame = eye->isNewFrame();

  // Debug output to see what's happening
  static int debugCounter = 0;
  if (debugCounter++ % 300 == 0) { // Print every ~5 seconds at 60fps
    cout << "isNewFrame: " << (isNewFrame ? "true" : "false")
         << ", streaming: " << (eye->isStreaming() ? "true" : "false") << endl;
  }

  if (isNewFrame) {
    eye->check_ff71();
    eyeframe *frame = eye->getLastVideoFramePointer();

    // Convert frames
    convert_opencv_to_RGB(frame->videoLeftFrame, frame_rgb_left,
                          eye->getWidth(), eye->getHeight());

    if (showRightCamera) {
      convert_opencv_to_RGB(frame->videoRightFrame, frame_rgb_right,
                            eye->getWidth(), eye->getHeight());
    }

    // Update FPS
    frameCount++;
    auto currentTime = chrono::steady_clock::now();
    auto elapsed =
        chrono::duration_cast<chrono::milliseconds>(currentTime - lastTime);

    if (elapsed.count() >= 1000) { // Update every second
      fps = frameCount * 1000.0f / elapsed.count();
      frameCount = 0;
      lastTime = currentTime;
    }

    // Display frames
    if (showRightCamera) {
      Mat combined;
      hconcat(frameLeft, frameRight, combined);

      // Add FPS text
      string fpsText = "FPS: " + to_string((int)fps);
      putText(combined, fpsText, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1,
              Scalar(0, 255, 0), 2);
      putText(combined, "Left Camera", Point(10, combined.rows - 20),
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
      putText(combined, "Right Camera",
              Point(eye->getWidth() + 10, combined.rows - 20),
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

      imshow("PS4 Eye Camera - Stereo View", combined);

      if (saveFrames) {
        string filename = "frame_stereo_" + to_string(frameCounter++) + ".jpg";
        imwrite(filename, combined);
      }
    } else {
      // Add FPS text
      string fpsText = "FPS: " + to_string((int)fps);
      putText(frameLeft, fpsText, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1,
              Scalar(0, 255, 0), 2);
      putText(frameLeft, "Left Camera", Point(10, frameLeft.rows - 20),
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

      imshow("PS4 Eye Camera - Left View", frameLeft);

      if (saveFrames) {
        string filename = "frame_left_" + to_string(frameCounter++) + ".jpg";
        imwrite(filename, frameLeft);
      }
    }
  } else {
    // Even if no new frame, show the window with a message
    static bool windowCreated = false;
    if (!windowCreated) {
      // Create a black frame with text indicating no data
      Mat noDataFrame = Mat::zeros(eye->getHeight(), eye->getWidth(), CV_8UC3);
      putText(noDataFrame, "No video data received", Point(50, eye->getHeight()/2),
              FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
      putText(noDataFrame, "Check USB connection and permissions", Point(50, eye->getHeight()/2 + 40),
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
      imshow("PS4 Eye Camera - Left View", noDataFrame);
      windowCreated = true;
    }
  }
}

void PS4EYECapture::handleKeyInput() {
  int key = waitKey(1) & 0xFF;

  switch (key) {
  case 'r':
  case 'R':
    showRightCamera = !showRightCamera;
    eye->rightflag = showRightCamera ? 1 : 0;
    cout << "Right camera " << (showRightCamera ? "enabled" : "disabled")
         << endl;
    destroyAllWindows(); // Close current windows
    break;

  case 's':
  case 'S': {
    string timestamp =
        to_string(chrono::duration_cast<chrono::seconds>(
                      chrono::system_clock::now().time_since_epoch())
                      .count());

    if (showRightCamera) {
      Mat combined;
      hconcat(frameLeft, frameRight, combined);
      string filename = "ps4eye_stereo_" + timestamp + ".jpg";
      imwrite(filename, combined);
      cout << "Saved stereo frame: " << filename << endl;
    } else {
      string filename = "ps4eye_left_" + timestamp + ".jpg";
      imwrite(filename, frameLeft);
      cout << "Saved left frame: " << filename << endl;
    }
    break;
  }

  case 'f':
  case 'F':
    saveFrames = !saveFrames;
    cout << "Continuous frame saving " << (saveFrames ? "enabled" : "disabled")
         << endl;
    if (saveFrames) {
      frameCounter = 0;
    }
    break;

  case 'q':
  case 'Q':
  case 27: // ESC key
    mShouldQuit = true;
    break;
  }
}

void PS4EYECapture::run() {
  while (!mShouldQuit) {
    processFrame();
    handleKeyInput();

    this_thread::sleep_for(chrono::milliseconds(16)); // ~60 FPS display rate
  }
}

void PS4EYECapture::shutdown() {
  cout << "Shutting down..." << endl;

  mShouldQuit = true;

  if (mThread.joinable()) {
    mThread.join();
  }

  if (eye) {
    eye->shutdown();
  }

  if (frame_rgb_left) {
    delete[] frame_rgb_left;
    frame_rgb_left = nullptr;
  }

  if (frame_rgb_right) {
    delete[] frame_rgb_right;
    frame_rgb_right = nullptr;
  }

  destroyAllWindows();
  cout << "Shutdown complete." << endl;
}

int main() {
  // Set up signal handlers
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  PS4EYECapture capture;
  g_capture = &capture;

  if (!capture.initialize()) {
    cout << "Failed to initialize PS4 Eye camera" << endl;
    return -1;
  }

  cout << "\nStarting camera capture..." << endl;
  capture.run();

  return 0;
}
