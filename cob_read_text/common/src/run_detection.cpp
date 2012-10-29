#include <cob_read_text/text_detect.h>
#include <iostream>

int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cout << "not enought input: detect_text <image> <correlation> <dictionary>" << std::endl;
    return -1;
  }

  bool eval = false;
  if (argc > 4)
    if ((std::string)argv[4] == "eval")
      eval = true;

  bool enableOCR = true;
  if (argc > 5)
	  if ((std::string)argv[5] == "OCRoff")
		  enableOCR = false;

  DetectText detector = DetectText(eval, enableOCR);
  detector.readLetterCorrelation(argv[2]);
  detector.readWordList(argv[3]);

  ros::init(argc, argv, "run_detection");
  ros::NodeHandle nh;
  detector.setParams(nh);
  detector.detect(std::string(argv[1]));

  ros::shutdown();

  return 0;
}
