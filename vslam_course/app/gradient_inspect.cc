#include <string>
#include <iostream>
#include <istream>
#include <iterator>
#include <filesystem>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;
#include "common/util.h"


#define CHECK(x, msg) \
    if (!(x)) {\
        std::cerr << msg << std::endl; \
        std::exit(1); \
    } 

namespace detail {
class Line: public std::string {
    friend std::istream& operator<<(std::istream& is, Line& line) {
        return std::getline(is, line);
    }
};
}

template<typename outT>
void read_lines(std::istream& strm, outT dest ) {
    using Iter = std::istream_iterator<detail::Line>;
    std::cout << "dest type: " << typeid(dest).name() << std::endl;
    std::copy(Iter(strm), Iter(), dest);
}
void print_flist(std::string path) {
    std::ifstream f(path.c_str());
    if(!f.is_open()) {
        std::cerr << " failed to open " << path << std::endl;
        std::exit(1);
        return;
    }
    std::vector<std::string> lines;
    read_lines(f, std::back_inserter(lines));

    std::filesystem::path root(path);


    for(auto f : lines) {
        if (f.find("png") < f.length()) {
            std::filesystem::path img_path = root.remove_filename() ;
            img_path /= f;
            std::cout << " file : " << img_path << " @ idx: " << f.find("png") << " length: " << f.length() << std::endl;
            CHECK(std::filesystem::exists(img_path), " file not found");


            cv::Mat img = cv::imread(img_path.generic_string(), cv::IMREAD_GRAYSCALE);
             cv::Mat dx, dy;
            cv::Sobel(img, dx, CV_32F, 1, 0, 3);
            cv::Sobel(img, dy, CV_32F, 0, 1, 3);


            cv::imshow("merge", dy);
            cv::waitKey();
        }

        

        break;
        
    }

}


int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " img_files.txt" << std::endl;
        return -1;
    }
    //std::string data_folder = zs::folder_and_slash(argv[1]);
    std::string img_list = argv[1];
    print_flist(img_list);
    return 0;
}