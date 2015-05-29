#include <string>
#include <vector>
#include "units.h"

using namespace std;

namespace eccv14 {
    vector<Pose> ReadAnnotationFile(string fp);
}

namespace linemod {
    Pose ReadRotTransFiles(string rp, string tp);
    cv::Mat ReadDepthFile(string fp);
    
    class ImageIterator{
    public:
        ImageIterator();
        ImageIterator(string dir, string indices_path);
        string directory;
        vector<string> indices;
        size_t current_index;
        void Next();
        cv::Mat GetImage() const;
        cv::Mat GetDepth() const;
        Pose GetPose() const;
    };
}
