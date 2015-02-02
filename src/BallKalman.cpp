#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>

void GetKalmanPredict (cv::Point &center);

int main(int argc, const char * argv[])
{
//	std::cout << "work damnit" << std::endl;
    CvCapture* capture = cvCreateFileCapture("/Users/agata/Desktop/CodingProjects/BTSimple/BallTracking/Fail.MOV"); //change this to be the path to your video file

    //IplImage* frame;// = NULL;
    cv::Mat frame;

    if(!capture)
    {
        printf("Video Not Opened\n");
        return -1;
    }

    CvSize size = cvSize(
                     (int)cvGetCaptureProperty( capture,
                                               CV_CAP_PROP_FRAME_WIDTH),
                     (int)cvGetCaptureProperty( capture,
                                               CV_CAP_PROP_FRAME_HEIGHT)
                     );
    //CvVideoWriter *writer = cvCreateVideoWriter("/Users/agata/Desktop/vidtry.mov",CV_FOURCC('m', 'p', '4', 'v'),15,size,1);

    int width = (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH);
    int height = (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT);
    double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    int frame_count = (int)cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);

    printf("Video Size = %d x %d\n",width,height);
    printf("FPS = %f\nTotal Frames = %d\n",fps,frame_count);

    cv::Mat hsvimage;
    cv::Mat threshimage, erodeimage, dilateimage, imblur;
    cv::Point center, center_K;
    cv::Mat KalmA, KalmB, KalmC, KalmEz, KalmEx, KalmQ, Kalmu, KalmP, KalmK;
    float t = 1.0;
    float A[] = {1.0, 0.0, t, 0.0,
    		0.0, 1.0, 0.0, t,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0};
    KalmA = cv::Mat(4, 4, CV_32FC1, &A);

    float t2 = (t*t)/2;
    float B[] = {t2, t2, t, t};
    KalmB = cv::Mat(4, 1, CV_32FC1, &B);

    float C[] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    KalmC = cv::Mat(2, 4, CV_32FC1, &C);

    float sig_x=0.1;
    float Ez[] = {sig_x, 0, 0, sig_x};
    KalmEz = cv::Mat(2, 2, CV_32FC1, &Ez);

    float Ex []= {pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0, 0,
    		0, pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0,
			pow(t,3.0)/2.0, 0, pow(t,2.0), 0,
			0, pow(t,3.0)/2, 0, pow(t,2.0)};
    KalmEx = cv::Mat(4, 4, CV_32FC1, &Ex);

    float u=0.05;
    //Kalmu = cv::Mat(1, 2, CV_32FC1, &u);

    float Q[] = {0.0, 0.0, 0.0, 0.0};
    KalmQ = cv::Mat(4, 1, CV_32FC1, &Q);
    cv::Mat Q_est = KalmQ;
    KalmP = KalmEx;

    int radius;
    std::vector <float> radii;
    for (int i = 0; i<frame_count; i++) /// if we were to grab this video from a live feed we would change to while(1)
    {
        frame = (cv::Mat)cvQueryFrame(capture);

        //converting to HSV color space
        cv::cvtColor(frame, hsvimage, CV_BGR2HSV);
        cv::inRange(hsvimage, cv::Scalar(75, 100, 0), cv::Scalar(130, 255, 255), threshimage); //


        //getting rid of noise
        cv::erode(threshimage, erodeimage, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        cv::dilate(erodeimage, dilateimage, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        cv::GaussianBlur( dilateimage, imblur, cv::Size(9, 9), 2, 2 );

        /// hough transform to find the circles
        cv::vector<cv::Vec3f> circles;
        cv::HoughCircles( imblur, circles, CV_HOUGH_GRADIENT, 1, 300, 50, 10 );



        //std::cout << circles.size() << std::endl; //for more than one cirle: for( size_t i = 0; i < circles.size(); i++ )
        if (circles.size()>0){
            center.x = cvRound(circles[0][0]); center.y= cvRound(circles[0][1]);

            //cv::circle( frame, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// circle center
          //

            if (i<10){
            	radius= cvRound(circles[0][2]);
            }

            radii.push_back((float)circles[0][2]);

            if (i>9){
            	radii.erase (radii.begin());
                float sum = std::accumulate(radii.cbegin(), radii.cend(), 0.0);
                float avg = sum/10.0;

                radius = cvRound(avg);
                float prop = (float)circles[0][2]/avg*100.0;
                if (prop>120 || prop <80){
                    float sum_new = std::accumulate(radii.cbegin(), radii.cbegin()+8, 0.0);
                    float avg_new = sum/9.0;
                    radius = cvRound(avg_new);
                    radii.pop_back();
                    radii.push_back(avg_new);

                }
                std::cout << prop << std::endl;

            }

            float diff_x = center.x-center_K.x;
            float diff_y = center.y-center_K.y;
            float dist = cv::sqrt(diff_x*diff_x + diff_y*diff_y);
            //std::cout << radius << std::endl;
            if (dist > 100 && i>0){
            	float small_noise_x = (float)std::rand()/RAND_MAX;
            	float small_noise_y = (float)std::rand()/RAND_MAX;
            	center_K.x = Q_est.at<float>(0) + small_noise_x;
            	center_K.y = Q_est.at<float>(1) + small_noise_y;
            }
            else{
            	center_K.x = cvRound(circles[0][0]); center_K.y= cvRound(circles[0][1]);
            }

        }

        else{
        	float small_noise_x = (float)std::rand()/RAND_MAX;
        	float small_noise_y = (float)std::rand()/RAND_MAX;
        	center_K.x = Q_est.at<float>(0) + small_noise_x;
        	center_K.y = Q_est.at<float>(1) + small_noise_y;

        	//std::cout << "else case"<< std::endl;
        }
        //Predict next state with the last state and predicted motion
        Q_est = KalmA * Q_est + KalmB * u;

        // Predict next covariance
        KalmP = KalmA * KalmP * KalmA.t() + KalmEx;

        // Kalman Gain
        KalmK = KalmP*KalmC.t()*((KalmC*KalmP*KalmC.t()+KalmEz).inv());

        //Update the state estimate
        float ctr [] = {center_K.x*1.0, center_K.y*1.0};
        cv::Mat Q_loc_meas = cv::Mat(2, 1, CV_32FC1, &ctr);
        Q_est = Q_est + KalmK * (Q_loc_meas - KalmC * Q_est);

        //update covariance estimation
        KalmP =  (cv::Mat::eye(4, 4, CV_32FC1)-KalmK*KalmC)*KalmP;

//        center_K.x = Q_est.at<float>(0);
//        center_K.y = Q_est.at<float>(1);
        //std::cout << center << std::endl;
        cv::circle( frame, center_K, 3, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle( frame, center_K, radius, cv::Scalar(0,0,255), 3, 8, 0 ); // circle outline

        IplImage iplimg = frame;
        cvShowImage("Example2",&iplimg);
        //cvWriteFrame(writer, &iplimg );

        char c = cvWaitKey(1);
        if( c == 27 ) break;


    }


    cvReleaseCapture(&capture);
    //cvReleaseVideoWriter( &writer );
    cvDestroyWindow( "Example2" );

    return 0;
}

void GetKalmanPredict(cv::Point &center){
	cv::Mat KalmA, KalmB, KalmC, KalmEz, KalmEx, KalmQ, Kalmu, KalmP, KalmK;
	float t = 0.1;
	float A[] = {1.0, 0, t, 0,
			0, 1.0, 0, t,
			0, 0, 1.0, 0,
			0, 0, 0, 1.0};
	KalmA = cv::Mat(4, 4, CV_32FC1, &A);

	float t2 = (t*t)/2;
	float B[] = {t2, t2, t, t};
	KalmB = cv::Mat(4, 1, CV_32FC1, &B);

	float C[] = {1, 0, 0, 0, 0, 1, 0, 0};
	KalmC = cv::Mat(2, 4, CV_32FC1, &C);

	float sig_x=0.1;
	float Ez []= {sig_x, 0, 0, sig_x};
	KalmEz = cv::Mat(2, 4, CV_32FC1, &Ez);

	float Ex[] = {pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0, 0,
				0, pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0,
				pow(t,3.0)/2.0, 0, pow(t,2.0), 0,
				0, pow(t,3.0)/2, 0, pow(t,2.0)};
	KalmEx = cv::Mat(4, 4, CV_32FC1, &Ex);

	float u[] = {1.5, 1.5};
	Kalmu = cv::Mat(1, 2, CV_32FC1, &u);

	float Q[] = {0, 0, 0, 0};
	KalmQ = cv::Mat(4, 1, CV_32FC1, &Q);
	cv::Mat Q_est = KalmQ;

	KalmP = KalmEx;

	//LOOP SHOULD START HERE
	//Predict next state with the last state and predicted motion
	Q_est = KalmA * Q_est + KalmB * Kalmu;

	// Predict next covariance
	KalmP = KalmA * KalmP * KalmA.t() + KalmEx;

	// Kalman Gain
	KalmK = KalmP*KalmC.t()*((KalmC*KalmP*KalmC.t()+KalmEz).inv());

	//Update the state estimate
	cv::Mat Q_loc_meas = cv::Mat(2, 1, CV_32FC1, &center);
	Q_est = Q_est + KalmK * (Q_loc_meas - KalmC * Q_est);

	//update covariance estimation
	KalmP =  (cv::Mat::eye(4, 4, CV_32FC1)-KalmK*KalmC)*KalmP;


}
