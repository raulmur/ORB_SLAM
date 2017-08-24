#include "vio/timegrabber.h"
void testTimeGrabber(std::string timeFile)
{
    vio::TimeGrabber tg(timeFile);
    double timestamp = tg.readTimestamp(0);
    std::cout << "expected timestamp for 0 frame 0 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1799);
    std::cout << "expected timestamp for 1799 frame 1.864921e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1800);
    std::cout << "expected timestamp for 1800 frame 1.865959e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(4540);
    std::cout << "expected timestamp for 4540 frame 4.705816e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(4541);
    std::cout << "expected timestamp for 4541 frame NULL and timestamp read from file "<< timestamp<< std::endl;

}
int main(){

    vio::TimeGrabber tg("../test/dilili_video_timestamps.txt", 1);

    double timestamp = tg.extractTimestamp(50, false);
    std::cout<< "image index timestamp at 1 "<< timestamp <<std::endl;

    timestamp = tg.extractTimestamp(100, false);
    std::cout<< "image index timestamp at 100 "<< timestamp <<std::endl;
	testTimeGrabber("../test/times_kitti_seq00.txt");
	return 0;
}
