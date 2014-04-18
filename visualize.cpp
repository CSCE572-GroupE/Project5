#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

void imageReceived(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr cvImagePtr;
    try
    {
        cvImagePtr = cv_bridge::toCvCopy(image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat &image_matrix = cvImagePtr->image;
    int width = image_matrix.cols;
    int height = image_matrix.rows;

    ROS_INFO_STREAM("IMAGE SIZE: " << width << ", " << height);

    cv::imshow("unedited_window", image_matrix);


    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){

            try {
                cv::Vec3b &pixel = image_matrix.at<cv::Vec3b>(j,i);
                if ((pixel[0] < 190 && pixel[1] < 225 && pixel[2] < 225) || (pixel[0] + pixel[1] + pixel[2] > 620) || j < height/2 || (i < width/2 - 20 || i > width/2 + 20) ){
            //    if (pixel[0] > 180 && pixel[1] > 25 && pixel[1] < 200 && pixel[2] < 50){
                    pixel[0] = 0;
                    pixel[1] = 0;
                    pixel[2] = 0;
                } else {
//                    ROS_INFO("image[%d,%d] = (r=%d,g=%d,b=%d)", i, j, pixel[2], pixel[1], pixel[0]);

                }
            } catch(int e){
                ROS_INFO_STREAM("ERROR");
            }
        }
    }

    ROS_INFO_STREAM("BEFORE DISPLAY");

    cv::imshow("edited_window", image_matrix);

    ROS_INFO_STREAM("END OF CALLBACK");

   // cv::Vec3b pixel = mat.at<cv::Vec3b>(i,j);
   // ROS_INFO("image[%d,%d] = (r=%d,g=%d,b=%d)", i, j, pixel[2], pixel[1], pixel[0])

//    cv::Vec3b &pixel = mat.at<cv::Vec3b>(i,j);
//    pixel[0] = 0;
//    pixel[1] = 0;
//    pixel[2] = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize");
    ros::param::set("image_transport", "theora");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, &imageReceived);

    cvNamedWindow("unedited_window", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("edited_window", CV_WINDOW_AUTOSIZE);
    cvStartWindowThread();
	ros::spin();

    cvDestroyWindow("unedited_window");
    cvDestroyWindow("edited_window");
}

