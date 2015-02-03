#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>

typedef cv::vector<cv::Point2f> Pointlist;

void GetBallCenter (Pointlist &centers, cv::Mat &frame, int &rad_new);
void CombineImages (cv::Mat &frame1, cv::Mat &frame2, cv::Mat &out_image);
void GetKalmanPredict (cv::Point &center);

int main(int argc, const char * argv[])
{
//	std::cout << "work damnit" << std::endl;
    CvCapture* capture = cvCreateFileCapture("/Users/agata/Desktop/CodingProjects/BTSimple/BallTracking/Fail.MOV"); //change this to be the path to your video file
    CvCapture* capture2 = cvCreateFileCapture("/Users/agata/Desktop/Fail_track.mov");
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
    //size.height = size.height;
    size.width = size.width*2;
    CvVideoWriter *writer = cvCreateVideoWriter("/Users/agata/Desktop/vidtry_combo.mov",CV_FOURCC('m', 'p', '4', 'v'),15,size,1);

    int width = (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH);
    int height = (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT);
    double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    int frame_count = (int)cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);

    printf("Video Size = %d x %d\n",width,height);
    printf("FPS = %f\nTotal Frames = %d\n",fps,frame_count);

    cv::Point center, center_K;
    cv::Mat KalmA, KalmB, KalmC, KalmEz, KalmEx, KalmQ, Kalmu, KalmP, KalmK;
    float t = 1.0;
    float A[] = {1.0, 0.0, t, 0.0,
    			 0.0, 1.0, 0.0, t,
				 0.0, 0.0, 1.0, 0.0,
				 0.0, 0.0, 0.0, 1.0};
    KalmA = cv::Mat(4, 4, CV_32FC1, &A);

    float t2 = (t*t)/2.0;
    float B[] = {t2, t2, t, t};
    KalmB = cv::Mat(4, 1, CV_32FC1, &B);

    float C[] = {1.0, 0.0, 0.0, 0.0,
    			 0.0, 1.0, 0.0, 0.0};
    KalmC = cv::Mat(2, 4, CV_32FC1, &C);

    float sig_x=1.0;
    float Ez[] = {sig_x, 0, 0, sig_x};
    KalmEz = cv::Mat(2, 2, CV_32FC1, &Ez);

    float Ex []= {pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0, 0,
    			  0, pow(t,4.0)/4.0, 0, pow(t,3.0)/2.0,
				  pow(t,3.0)/2.0, 0, pow(t,2.0), 0,
				  0, pow(t,3.0)/2.0, 0, pow(t,2.0)};
    KalmEx = cv::Mat(4, 4, CV_32FC1, &Ex);
    float noise_mag = 0.1;
    KalmEx = KalmEx*pow(noise_mag,2.0);

    float u=0.005;
    //Kalmu = cv::Mat(1, 2, CV_32FC1, &u);

    float Q[] = {0.0, 0.0, 0.0, 0.0};
    KalmQ = cv::Mat(4, 1, CV_32FC1, &Q);
    cv::Mat Q_est = KalmQ;
    KalmP = KalmEx;

    int radius;
    std::vector <float> radii;

    Pointlist centers;
    int rad_new;
    cv::Point ctr_new;

    for (int i = 0; i<frame_count; i++) /// if we were to grab this video from a live feed we would change to while(1)
    {
    	frame = (cv::Mat)cvQueryFrame(capture);


    	GetBallCenter (centers, frame, rad_new);
    	ctr_new = centers[0];

        //std::cout << circles.size() << std::endl; //for more than one cirle: for( size_t i = 0; i < circles.size(); i++ )
        if (centers.size()>0){
            //center.x = cvRound(circles[0][0]); center.y= cvRound(circles[0][1]);
        	center.x = ctr_new.x; center.y= ctr_new.y;


            if (i<10){
            	radius= (float)rad_new;
            }

            radii.push_back((float)rad_new);

            if (i>9){
            	radii.erase (radii.begin());
                float sum = std::accumulate(radii.cbegin(), radii.cend(), 0.0);
                float avg = sum/10.0;

                radius = cvRound(avg);
                float prop = (float)rad_new/avg*100.0;
                if (prop>120 || prop <80){
                    radii.pop_back();
                    radii.push_back(radii.back());
                    float sum_new = std::accumulate(radii.cbegin(), radii.cend(), 0.0);
                    float avg_new = sum_new/10.0;
                    radius = cvRound(avg_new);
                }
                //std::cout << prop << "::" << rad_new << "::" << radius << std::endl;

            }

            float diff_x = center.x-center_K.x;
            float diff_y = center.y-center_K.y;
            float dist = cv::sqrt(diff_x*diff_x + diff_y*diff_y);
            //std::cout << radius << std::endl;
            if (dist > 15 && i>0){
            	float small_noise_x = (float)std::rand()/RAND_MAX;
            	float small_noise_y = (float)std::rand()/RAND_MAX;
            	center_K.x = Q_est.at<float>(0) + small_noise_x;
            	center_K.y = Q_est.at<float>(1) + small_noise_y;
            }
            else{
            	center_K.x = ctr_new.x; center_K.y= ctr_new.y;
            }

            std::cout << dist << std::endl;
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
        cv::circle( frame, center_K, (int)radius, cv::Scalar(0,0,255), 2, 8, 0 );
        //cv::circle( frame, center_K, radius, cv::Scalar(0,0,255), 3, 8, 0 ); // circle outline



        IplImage frame1 = frame;
        cv::Mat frame_two = (cv::Mat)cvQueryFrame(capture2);
        cv::Mat out_image;
        CombineImages (frame_two, frame, out_image);
        IplImage frameOut =out_image;
        cvShowImage("Example2",&frameOut);
        cvWriteFrame(writer, &frameOut );

        char c = cvWaitKey(1);
        if( c == 27 ) break;


    }


    cvReleaseCapture(&capture);
    cvReleaseVideoWriter( &writer );
    cvDestroyWindow( "Example2" );

    return 0;
}

void GetBallCenter (Pointlist &centers, cv::Mat &frame, int &rad_new){


    cv::Mat hsvimage, threshimage, erodeimage, dilateimage, imblur;
	//converting to HSV color space
	cv::cvtColor(frame, hsvimage, CV_BGR2HSV);
	cv::inRange(hsvimage, cv::Scalar(75, 100, 0), cv::Scalar(130, 255, 255), threshimage); //


	//getting rid of noise
	cv::erode(threshimage, erodeimage, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::dilate(erodeimage, dilateimage, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::GaussianBlur( dilateimage, imblur, cv::Size(9, 9), 2, 2 );

	/// Detect edges using canny
	cv::Mat canny_output;
	cv::Canny( imblur, canny_output, 100, 100*2, 3 );

	/// Find contours
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	/// Approximate contours to polygons + get bounding rects and circles
	cv::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
	cv::vector<float>radius2( contours.size() );
	Pointlist centers_in( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{ approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
		cv::minEnclosingCircle( (cv::Mat)contours_poly[i], centers_in[i], radius2[i] );
	}

	centers = centers_in;
	rad_new = radius2[0];

	//ctr_new = centers[0];
}

void CombineImages (cv::Mat &frame1, cv::Mat &frame2, cv::Mat &out_image){

	CvSize new_window_size;// = CvSize(new_width, new_height);
	new_window_size.width = frame1.cols + frame2.cols;
	new_window_size.height = frame1.rows;
	cv::Mat roi;

	out_image = cv::Mat(new_window_size, CV_8UC3);
	roi = cv::Mat(out_image, cv::Rect(0, 0, frame1.cols, frame1.rows));
	frame1.copyTo(roi);
	roi = cv::Mat(out_image, cv::Rect(frame1.cols, 0, frame2.cols, frame2.rows));
	frame2.copyTo(roi);

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
