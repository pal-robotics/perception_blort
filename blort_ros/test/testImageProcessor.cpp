#include <blort/Tracker/ImageProcessor.h>
#include <blort/TomGine/tgEngine.h>

#include <ros/ros.h>
#include <ros/package.h>

void testProcess(Tracking::Texture & out, const std::string & process_name)
{
  Tracking::Texture ref;
  {
    std::string ref_path = ros::package::getPath("blort_ros") + "/test/lenna_" + process_name + ".png";
    ref.load(ref_path.c_str());
  }
  if(out.getWidth() != ref.getWidth())
  {
    std::cerr << "Output of " << process_name << " has a difference width from the reference" << std::endl;
    return;
  }
  if(out.getHeight() != ref.getHeight())
  {
    std::cerr << "Output of " << process_name << " has a difference height from the reference" << std::endl;
    return;
  }
  unsigned char * out_data = new unsigned char[out.getWidth()*out.getHeight()*3];
  memset(out_data, 0, out.getWidth()*out.getHeight()*3);
  out.getImageData(out_data);
  unsigned char * ref_data = new unsigned char[ref.getWidth()*ref.getHeight()*3];
  memset(ref_data, 0, ref.getWidth()*ref.getHeight()*3);
  ref.getImageData(ref_data);
  int res = memcmp(out_data, ref_data, out.getWidth()*out.getHeight()*3);
  if(res != 0)
  {
    /* Compute a distance between the two images if the match is not perfect */
    unsigned int out_g = 0; unsigned int ref_g = 0;
    unsigned int distance = 0;
    for(size_t i = 0; i < out.getWidth()*out.getHeight(); ++i)
    {
      out_g = (out_data[3*i] + out_data[3*i+1] + out_data[3*i+2]);
      ref_g = (ref_data[3*i] + ref_data[3*i+1] + ref_data[3*i+2]);
      distance += ref_g > out_g ? ref_g - out_g : out_g - ref_g;
    }
    double d = static_cast<double>(distance)/static_cast<double>(out.getWidth()*out.getHeight());
    if(d > 1e-1)
    {
      std::cerr << "Output data of " << process_name << " does not match the reference" << std::endl;
      std::cout << "Average distance: " << d << std::endl;
    }
  }
  delete[] out_data;
  delete[] ref_data;
}

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
  testProcess(out, "flipUpsideDown");

  ip.copy(&in, &out);
  out.save("out_copy.png");
  testProcess(out, "copy");

  ip.rectification(&in, &out);
  out.save("out_rectification.png");
  testProcess(out, "rectification");

  ip.gauss(&in, &out);
  out.save("out_gauss.png");
  testProcess(out, "gauss");

  ip.sobel(&in, &out, 0.1, true, false);
  out.save("out_sobel.png");
  testProcess(out, "sobel");

  ip.thinning(&in, &out);
  out.save("out_thinning.png");
  testProcess(out, "thinning");

  ip.spreading(&in, &out);
  out.save("out_spreading.png");
  testProcess(out, "spreading");

  return 0;
}
