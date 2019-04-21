#include "myslam/system.h"
#include "myslam/config.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace myslam;

int main(int argc, char **argv)
{
  myslam::Config::setParameterFile(argv[1]);
  System system_;

  string dataset_dir = myslam::Config::get<string>("dataset_dir");
  cout << "dataset: " << dataset_dir << endl;
  ifstream fin(dataset_dir + "/associate.txt");
  if (!fin)
  {
    cout << "please generate the associate file called associate.txt!" << endl;
    return 1;
  }

  vector<string> rgb_files, depth_files;
  vector<double> rgb_times, depth_times;
  while (!fin.eof())
  {
    string rgb_time, rgb_file, depth_time, depth_file;
    fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
    rgb_times.push_back(atof(rgb_time.c_str()));
    depth_times.push_back(atof(depth_time.c_str()));
    rgb_files.push_back(dataset_dir + "/" + rgb_file);
    depth_files.push_back(dataset_dir + "/" + depth_file);

    if (fin.good() == false)
      break;
  }

  myslam::Camera::Ptr camera(new myslam::Camera);

  cout << "read total " << rgb_files.size() << " entries" << endl;
  for (int i = 0; i < rgb_files.size(); i++)
  {
    cout << "****** loop " << i << " ******" << endl;
    Mat color = cv::imread(rgb_files[i]);
    Mat depth = cv::imread(depth_files[i], -1);
    if (color.data == nullptr || depth.data == nullptr)
    {
      cout << "xxxxxxxxxxxxxx no more data xxxxxxxxxxx \n";
      break;
    }

    system_.Process(color, depth, rgb_times[i], camera);
  }

  return 0;
}