#define SLEEP_BETWEEN_FRAME_FETCH_US    (10)

class RTStream {
    private:
        std::unique_ptr<cv::VideoCapture> stream_;
        cv::Mat frame_;
        std::mutex mutex_;  
        std::thread thread_;
        int decode_frames();
      
    public:
        RTStream (const char * addr);
        int read(cv::Mat &frame);
        int start_stream();
        bool isOpened();
        ~RTStream ();
};