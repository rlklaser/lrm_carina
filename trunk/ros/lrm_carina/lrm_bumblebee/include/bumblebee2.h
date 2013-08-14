

#ifndef BUMBLEBEE2_HH
#define BUMBLEBEE2_HH

#include <dc1394/dc1394.h>

// ROS include
#include <sensor_msgs/Image.h>


namespace bumblebee2
{
  //! Macro for defining an exception with a given parent
  //  (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

  //! A standard Camera1394 exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  class Camera1394
  {
  public:
    Camera1394 ();
    ~Camera1394 ();

    int open (const char* guid,
	      const char* video_mode,
	      float fps,
	      int iso_speed,
	      const char* bayer,
	      const char* method);
    int close();

    void readData (sensor_msgs::Image &image);

#if 0
    int setZoom (unsigned int);
    int setFocus (int);
    int setIris (int);
#endif

    int setBrightness (unsigned int);
    int setExposure (unsigned int);
    int setShutter (int);
    int setGain (int);
    int setWhiteBalance (const char*);

    std::string device_id_;

  private:
    // device identifier
    dc1394camera_t * camera_;
      
    dc1394framerate_t frameRate_;
    dc1394video_mode_t videoMode_;
    dc1394speed_t isoSpeed_;
    dc1394color_filter_t BayerPattern_;
    dc1394bayer_method_t BayerMethod_;
    bool DoBayerConversion_;

    void SafeCleanup();

    void findFrameRate(float);
    void findVideoMode(const char*);
    void findIsoSpeed(int);
    void findBayerFilter(const char*, const char*);

    void uyvy2rgb (unsigned char *src, unsigned char *dest,
                   unsigned long long int NumPixels);
  };

#define CLIP(in, out)				\
  {						\
    in = in < 0 ? 0 : in;			\
    in = in > 255 ? 255 : in;			\
    out=in;					\
  }
  
};

#endif
