#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>

using namespace std;

struct ColorPoint {
    cv::Vec3b color;
    int x;
    int y;

    ColorPoint(cv::Vec3b col, int x_val, int y_val){
        color = col;
        x = x_val;
        y = y_val;
    }

};
const int RATE = 10;
const int VIEW_FINDER_WIDTH = 200;
const int VIEW_FINDER_HEIGHT = 75;
const float VIEW_FINDER_MIN_FILL_FACTOR = .0075f;
bool hasCallback = false;
vector<ColorPoint> colorPoints;


void imageReceived(const sensor_msgs::ImageConstPtr& image)
{
    hasCallback = false;
    colorPoints.clear();
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

    cv::imshow("unedited_window", image_matrix);


    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){

            try {
                //bgr
                cv::Vec3b &pixel = image_matrix.at<cv::Vec3b>(j,i);
                if ((j>height/2) && ((pixel[0] > 125 && pixel[1] < pixel[0]/1.5 && pixel[2] < pixel[0]/1.5) ||
                                     (pixel[0] > 175 && pixel[1] < pixel[0] && pixel[2] < pixel[0]/2 ))){

                    if (i > (width/2 - VIEW_FINDER_WIDTH/2) && i < (width/2 + VIEW_FINDER_WIDTH/2) && j > height - VIEW_FINDER_HEIGHT){
                        ColorPoint cp = ColorPoint(image_matrix.at<cv::Vec3b>(j,i), i, j);
                        colorPoints.push_back(cp);
                    }

                } else {
                    pixel[0] = 0;
                    pixel[1] = 0;
                    pixel[2] = 0;
                }
//                if ((pixel[0] < 190 && pixel[1] < 225 && pixel[2] < 225) || (pixel[0] + pixel[1] + pixel[2] > 620) || j < height/2){ // || (i < width/2 - 20 || i > width/2 + 20) ){
//            //    if (pixel[0] > 180 && pixel[1] > 25 && pixel[1] < 200 && pixel[2] < 50){
//                    pixel[0] = 0;
//                    pixel[1] = 0;
//                    pixel[2] = 0;
//                } else {
//                    ROS_INFO("image[%d,%d] = (r=%d,g=%d,b=%d)", i, j, pixel[2], pixel[1], pixel[0]);

//                }
            } catch(int e){
                ROS_INFO_STREAM("ERROR");
            }
        }
    }

    //ROS_INFO_STREAM("BEFORE DISPLAY");

    cv::imshow("edited_window", image_matrix);

    ROS_INFO_STREAM("END OF CALLBACK");

    hasCallback = true;

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

    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);

    ros::Rate rate(RATE);
    while(ros::ok()){
        if (hasCallback){
            geometry_msgs::Twist twistObject;
            ROS_INFO_STREAM("POINT COUNT: " << colorPoints.size());
            if(colorPoints.size() > (VIEW_FINDER_WIDTH * VIEW_FINDER_HEIGHT * VIEW_FINDER_MIN_FILL_FACTOR) ){
                // move straight
                int sectionACount = 0, sectionBCount = 0, sectionCCount = 0;
                //TODO: save global width
                float startX = 640.0f/2.0f - VIEW_FINDER_WIDTH/2.0f;
                for (int i = 0; i < colorPoints.size(); i++){
                    ColorPoint cp = colorPoints.at(i);
                    if (cp.x < startX + VIEW_FINDER_WIDTH/3.0f){
                        sectionACount++;
                    } else if (cp.x < startX + VIEW_FINDER_WIDTH * 2.0f/3.0f) {
                        sectionBCount++;
                    } else {
                        sectionCCount++;
                    }
                }
                int largest_section = 0;
                if (sectionACount > sectionBCount && sectionACount > sectionCCount){
                    largest_section = 1;
                } else if (sectionBCount > sectionACount && sectionBCount > sectionCCount){
                    largest_section = 2;
                } else {
                    largest_section = 3;
                }
//                int largest_section = sectionACount > sectionBCount ? (sectionACount > sectionCCount ? 1 : (sectionBCount > sectionCCount ? 2 : 3)) : (sectionBCount > sectionCCount ? 2 : (sectionACount > sectionCCount ? 1 : 3));
                switch(largest_section){
                    case 1:
                        ROS_INFO_STREAM("LEFT");
                        twistObject.angular.z = M_PI/8.0f/RATE;
                        twistObject.linear.x = 1.0f/RATE;
                        break;
                    case 2:
                        ROS_INFO_STREAM("STRAIGHT");
                        twistObject.linear.x = 2.0f/RATE;
                        break;
                    case 3:
                        ROS_INFO_STREAM("RIGHT");
                        twistObject.angular.z = -M_PI/8.0f/RATE;
                        twistObject.linear.x = 1.0f/RATE;
                        break;
                    default:
                        break;
                }

            } else if (colorPoints.size() >= 0){
                //turn
                ROS_INFO_STREAM("NOT ENOUGH TAPE");
                twistObject.angular.z = M_PI/2.0f/RATE;
            } else {
                twistObject.linear.x = -1;
            }
            twistPublisher.publish(twistObject);
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("\n");

    }
    //ros::spin();

    cvDestroyWindow("unedited_window");
    cvDestroyWindow("edited_window");
}

