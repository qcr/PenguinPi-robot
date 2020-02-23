#include <iostream>

#include "penguinpi.h"
#include "rtstream.h"

using namespace std;

namespace PenguinPi {


RTStream :: RTStream (const char * addr) : stream_(new cv::VideoCapture(addr)) { }

int RTStream :: start_stream(){
    while(!(*stream_).isOpened()){ usleep(50); }
    this->thread_ = std::thread(&RTStream::decode_frames, this);
}

int RTStream :: decode_frames(){

    cv::Mat buffer;

    if(!(*stream_).isOpened()){
        cerr << "Error opening video stream or file" << endl;
        return -1;
    }

    while(1){
        *stream_ >> buffer;

        if (!buffer.empty()) { 
            mutex_.lock();
            frame_ = buffer.clone();
            mutex_.unlock();
        }

        usleep(SLEEP_BETWEEN_FRAME_FETCH_US);
    }

    return 0;
}

int RTStream :: read(cv::Mat &frame){

    mutex_.lock();
    frame = frame_.clone();
    mutex_.unlock();
    return 0;

}

bool RTStream :: isOpened(){
    return (*stream_).isOpened();
}

RTStream :: ~RTStream(){
    thread_.join();
    stream_.release();
}


}


