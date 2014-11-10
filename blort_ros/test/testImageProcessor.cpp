#include <blort/Tracker/ImageProcessor.h>
#include <blort/TomGine/tgEngine.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "test_sobel_shader");
  std::string lenna_path = ros::package::getPath("blort_ros") + "/test/lenna.png";
  /* This create an OpenGL context to work with */
  TomGine::tgEngine window(512, 512, 1.0f, 0.1, "Sobel shader test", false);

  /* Load image first */
  Tracking::Texture in;
  in.load(lenna_path.c_str());
  Tracking::Texture out;
  float w = static_cast<float>(in.getWidth());
  float h = static_cast<float>(in.getHeight());

  std::string shader_path = ros::package::getPath("blort_ros") + "/Tracker/shader/";
  g_Resources->SetShaderPath(shader_path.c_str());

  Tracking::ImageProcessor ip;
  ip.init(w,h);

  in.save("in.png");

  ip.flipUpsideDown(&in, &out);
  out.save("out_flipUpsideDown.png");

  ip.copy(&in, &out);
  out.save("out_copy.png");

  ip.rectification(&in, &out);
  out.save("out_rectification.png");

  ip.gauss(&in, &out);
  out.save("out_gauss.png");

  ip.sobel(&in, &out, 0.1, true, false);
  out.save("out_sobel.png");

  ip.thinning(&in, &out);
  out.save("out_thinning.png");

  ip.spreading(&in, &out);
  out.save("out_spreading.png");

  return 0;
}
