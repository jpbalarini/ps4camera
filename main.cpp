#include "libuvc/libuvc.h"
#include <cstdlib>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <atomic>

// Global pointer to track allocated memory for cleanup
static uint8_t *g_accumulated_frame = NULL;

// Global variables for OpenCV display
static bool g_display_enabled = true;
static const char* LEFT_WINDOW_NAME = "PS4 Eye - Left Camera";
static const char* RIGHT_WINDOW_NAME = "PS4 Eye - Right Camera";

// Thread-safe handoff of frames from the UVC callback to the main thread
static std::mutex g_frame_mutex;
static std::atomic<bool> g_frame_ready(false);
static cv::Mat g_latest_left;
static cv::Mat g_latest_right;

static inline void publish_frame_for_display(const cv::Mat& left_image,
                                             const cv::Mat& right_image) {
  std::lock_guard<std::mutex> lock(g_frame_mutex);
  left_image.copyTo(g_latest_left);
  right_image.copyTo(g_latest_right);
  g_frame_ready.store(true, std::memory_order_release);
}

// Function to display stereo images using OpenCV
void display_stereo_images(const cv::Mat& left_image, const cv::Mat& right_image) {
  if (!g_display_enabled) {
    printf("Display disabled, skipping...\n");
    return;
  }

  printf("Checking image dimensions: left=%dx%d, right=%dx%d\n",
         left_image.cols, left_image.rows, right_image.cols, right_image.rows);

  try {
    // Display left and right images in separate windows
    printf("Showing left image...\n");
    cv::imshow(LEFT_WINDOW_NAME, left_image);
    printf("Showing right image...\n");
    cv::imshow(RIGHT_WINDOW_NAME, right_image);

    // Process OpenCV events (non-blocking)
    printf("Processing OpenCV events...\n");
    cv::waitKey(1);
    printf("OpenCV events processed\n");
  } catch (const cv::Exception& e) {
    printf("OpenCV display error: %s\n", e.what());
    printf("Disabling display for remaining frames...\n");
    g_display_enabled = false;
  } catch (...) {
    printf("Unknown error in OpenCV display, disabling display...\n");
    g_display_enabled = false;
  }
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  enum uvc_frame_format *frame_format = (enum uvc_frame_format *)ptr;
  static int frame_count = 0;
  static uint8_t *accumulated_frame = NULL;
  static size_t accumulated_size = 0;

  printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, "
         "ptr = %p\n",
         frame->frame_format, frame->width, frame->height, frame->data_bytes,
         ptr);

  // PS4 Eye camera frame format:
  // Each row contains: header(32) + audio(64) + left_image(1280*2) +
  // right_image(1280*2) + interleave(840*2) Total per row: 32 + 64 + 2560 +
  // 2560 + 1680 = 6896 bytes Expected frame: 3448x2x808 = 5,571,968 bytes total

  const int HEADER_SIZE = 32;
  const int AUDIO_SIZE = 64;
  const int LEFT_IMAGE_SIZE = 1280 * 2;  // 2560 bytes
  const int RIGHT_IMAGE_SIZE = 1280 * 2; // 2560 bytes
  const int INTERLEAVE_SIZE = 840 * 2;   // 1680 bytes
  const int ROW_SIZE = HEADER_SIZE + AUDIO_SIZE + LEFT_IMAGE_SIZE +
                       RIGHT_IMAGE_SIZE + INTERLEAVE_SIZE;
  const int IMAGE_HEIGHT = 800;  // Actual image data height
  const int FRAME_HEIGHT = 808;  // Total frame height from camera
  const int IMAGE_WIDTH = 1280;
  const int EXPECTED_FRAME_SIZE = ROW_SIZE * FRAME_HEIGHT; // 5,571,968 bytes

  // Initialize accumulation buffer if needed
  if (!accumulated_frame) {
    accumulated_frame = (uint8_t *)malloc(EXPECTED_FRAME_SIZE);
    g_accumulated_frame = accumulated_frame; // Keep global reference for cleanup
    if (!accumulated_frame) {
      printf("Failed to allocate frame buffer\n");
      return;
    }
    printf("Allocated buffer for %d bytes\n", EXPECTED_FRAME_SIZE);
  }

  // Check if this chunk would overflow our buffer
  if (accumulated_size + frame->data_bytes > EXPECTED_FRAME_SIZE) {
    printf("Frame overflow, resetting (had %zu bytes, got %lu more)\n",
           accumulated_size, frame->data_bytes);
    accumulated_size = 0;
  }

  // Accumulate this chunk
  memcpy(accumulated_frame + accumulated_size, frame->data, frame->data_bytes);
  accumulated_size += frame->data_bytes;

  printf("Accumulated %zu/%d bytes (%.1f%%)\n", accumulated_size,
         EXPECTED_FRAME_SIZE,
         (float)accumulated_size / EXPECTED_FRAME_SIZE * 100.0f);

  // Check if we have a complete frame
  if (accumulated_size < EXPECTED_FRAME_SIZE) {
    return; // Wait for more chunks
  }

  printf("Complete frame received! Processing frame %d...\n", frame_count);

  printf("Creating OpenCV matrices...\n");
  // Create OpenCV matrices for left and right images (YUV format, then convert
  // to RGB)
  cv::Mat left_yuv(IMAGE_HEIGHT, IMAGE_WIDTH,
                   CV_8UC2); // YUV422 format (2 bytes per pixel)
  cv::Mat right_yuv(IMAGE_HEIGHT, IMAGE_WIDTH,
                    CV_8UC2); // YUV422 format (2 bytes per pixel)
  cv::Mat left_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);  // RGB output
  cv::Mat right_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3); // RGB output
  printf("OpenCV matrices created successfully\n");

  // Extract left and right images from each row using accumulated frame
  int rows_written = 0;

  for (int row = 0; row < IMAGE_HEIGHT; row++) {
    int row_offset = row * ROW_SIZE;

    // Skip header and audio data
    int left_offset = row_offset + HEADER_SIZE + AUDIO_SIZE;
    int right_offset = left_offset + LEFT_IMAGE_SIZE;

    // Safety check to prevent buffer overrun
    if (right_offset + RIGHT_IMAGE_SIZE > accumulated_size) {
      printf("Warning: Row %d would exceed buffer bounds, stopping at %d rows\n", row, rows_written);
      break;
    }

    // Extract left image data (YUV422 format - 2 bytes per pixel)
    uint8_t *left_yuv_row_ptr = left_yuv.ptr<uint8_t>(row);
    memcpy(left_yuv_row_ptr, accumulated_frame + left_offset, LEFT_IMAGE_SIZE);

    // Extract right image data (YUV422 format - 2 bytes per pixel)
    uint8_t *right_yuv_row_ptr = right_yuv.ptr<uint8_t>(row);
    memcpy(right_yuv_row_ptr, accumulated_frame + right_offset,
           RIGHT_IMAGE_SIZE);

    rows_written++;
  }

  printf("Extracted %d rows of image data\n", rows_written);

  // Convert YUV to BGR - PS4 Eye uses YUY2 format
  printf("Converting YUV to BGR...\n");
  cv::cvtColor(left_yuv, left_image, cv::COLOR_YUV2BGR_YUY2);
  printf("Left image converted\n");
  cv::cvtColor(right_yuv, right_image, cv::COLOR_YUV2BGR_YUY2);
  printf("Right image converted\n");

  // Handoff images to the main thread for display (macOS requires GUI on main thread)
  publish_frame_for_display(left_image, right_image);

  // Save as PNG files
  char left_filename[64], right_filename[64];
  snprintf(left_filename, sizeof(left_filename), "left_image_%06d.png",
           frame_count);
  snprintf(right_filename, sizeof(right_filename), "right_image_%06d.png",
           frame_count);

  // bool left_saved = cv::imwrite(left_filename, left_image);
  // bool right_saved = cv::imwrite(right_filename, right_image);

  // if (left_saved && right_saved) {
  //   printf("Saved stereo PNG images: %s and %s (%dx%d, %d rows)\n",
  //          left_filename, right_filename, IMAGE_WIDTH, IMAGE_HEIGHT,
  //          rows_written);
  // } else {
  //   printf("Failed to save PNG images\n");
  // }

  // Reset for next frame
  accumulated_size = 0;
  frame_count++;

  if (frame->sequence % 30 == 0) {
    printf(" * got image %u\n", frame->sequence);
  }
}

// Cleanup function to free allocated memory and close OpenCV windows
void cleanup_callback_memory() {
  if (g_accumulated_frame) {
    free(g_accumulated_frame);
    g_accumulated_frame = NULL;
  }

  // Close OpenCV windows
  if (g_display_enabled) {
    cv::destroyWindow(LEFT_WINDOW_NAME);
    cv::destroyWindow(RIGHT_WINDOW_NAME);
    cv::destroyAllWindows();
  }
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  // Check for --no-display argument
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--no-display") == 0) {
      g_display_enabled = false;
      printf("Display disabled via command line argument\n");
      break;
    }
  }

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev, 0, 0,
      NULL); /* filter devices: vendor_id, product_id, "serial_num" */

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      puts("Device opened");

      // Initialize OpenCV windows for display
      if (g_display_enabled) {
        try {
          cv::namedWindow(LEFT_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
          cv::namedWindow(RIGHT_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
          puts("OpenCV display windows initialized");
        } catch (const cv::Exception& e) {
          printf("Failed to initialize OpenCV windows: %s\n", e.what());
          printf("Continuing without display...\n");
          g_display_enabled = false;
        } catch (...) {
          printf("Failed to initialize OpenCV windows (unknown error)\n");
          printf("Continuing without display...\n");
          g_display_enabled = false;
        }
      }

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      enum uvc_frame_format frame_format;
      int width = 640;
      int height = 480;
      int fps = 30;

      switch (format_desc->bDescriptorSubtype) {
      case UVC_VS_FORMAT_MJPEG:
        frame_format = UVC_COLOR_FORMAT_MJPEG;
        break;
      case UVC_VS_FORMAT_FRAME_BASED:
        frame_format = UVC_FRAME_FORMAT_H264;
        break;
      default:
        frame_format = UVC_FRAME_FORMAT_YUYV;
        break;
      }

      if (frame_desc) {
        width = frame_desc->wWidth;
        height = frame_desc->wHeight;
        fps = 10000000 / frame_desc->dwDefaultFrameInterval;
        // width = 1748;
        // height = 408;
        // fps = 120;
      }

      printf("\nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat,
             width, height, fps);

      /* Try to negotiate first stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl,                     /* result stored in ctrl */
          frame_format, width, height, fps /* width, height, fps */
      );

      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res,
                   "get_mode"); /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void *) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void *)12345, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");

          /* enable auto exposure - see uvc_set_ae_mode documentation */
          puts("Enabling auto exposure ...");
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            puts(" ... enabled auto exposure");
          } else if (res == UVC_ERROR_PIPE) {
            /* this error indicates that the camera does not support the full AE
             * mode; try again, using aperture priority mode (fixed aperture,
             * variable exposure time) */
            puts(" ... full AE not supported, trying aperture priority mode");
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res =
                uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture "
                              "priority mode");
            } else {
              puts(" ... enabled aperture priority auto exposure mode");
            }
          } else {
            uvc_perror(
                res,
                " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }

          /* Stream until user presses 'q' or ESC key */
          if (g_display_enabled) {
            puts("Press 'q' or ESC to quit streaming...");
            int key = 0;
            while (key != 'q' && key != 27) { // 27 is ESC key
              // If a new frame is ready, display it on the main thread
              if (g_frame_ready.load(std::memory_order_acquire)) {
                cv::Mat left, right;
                {
                  std::lock_guard<std::mutex> lock(g_frame_mutex);
                  if (!g_latest_left.empty() && !g_latest_right.empty()) {
                    left = g_latest_left.clone();
                    right = g_latest_right.clone();
                  }
                  g_frame_ready.store(false, std::memory_order_release);
                }
                if (!left.empty() && !right.empty()) {
                  cv::imshow(LEFT_WINDOW_NAME, left);
                  cv::imshow(RIGHT_WINDOW_NAME, right);
                }
              }
              key = cv::waitKey(1) & 0xFF; // process events and poll for quit
            }
          } else {
            sleep(10); /* stream for 10 seconds if no display */
          }

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device
   * handles, and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");

  // Clean up any allocated memory from callback
  cleanup_callback_memory();

  return 0;
}
