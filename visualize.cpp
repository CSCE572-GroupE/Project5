#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

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

const int EXTERNAL_VIEW_FINDER_WIDTH = 640;
const int EXTERNAL_VIEW_FINDER_HEIGHT = 125;

bool allow_change_of_direction = true;
int turn_value = 1;

const float VIEW_FINDER_MIN_FILL_FACTOR = .0075f;
bool hasCallback = false;
vector<ColorPoint> colorPoints;
vector<ColorPoint> outerColorPoints;
vector<int> recentHistoryBias;

bool bump_sensor_valid = true;
bool diagnostics_valid = true;


bool motionAllowed(){
    return bump_sensor_valid && diagnostics_valid;
}

void bumpWheeldropMessageReceived(std_msgs::Int32 bumpWheeldropObject){
    int bumpWheeldrop = bumpWheeldropObject.data;
    if (bumpWheeldrop != 0){
        bump_sensor_valid = false;
    } else {
        bump_sensor_valid = true;
    }
}

void diagnosticMessageRecieved(diagnostic_msgs::DiagnosticArray diagnosticArray){
    bool issue_found = false;
    for (int i = 0; i < sizeof(diagnosticArray.status); i++){
        diagnostic_msgs::DiagnosticStatus status = diagnosticArray.status[i];
        if (status.level == 1){
            ROS_INFO_STREAM("WARNING: " << status.name << " -- " << status.message);
            issue_found = true;
        } else if (status.level == 2){
            ROS_INFO_STREAM("ERROR: " << status.name << " -- " << status.message);
            issue_found = true;
        }
    }
    if (issue_found){
        diagnostics_valid = false;
    } else {
        diagnostics_valid = true;
    }
}



void imageReceived(const sensor_msgs::ImageConstPtr& image)
{
    hasCallback = false;
    colorPoints.clear();
    outerColorPoints.clear();
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
                if ((j>height/2) && ((pixel[0] > 50 && pixel[1] < pixel[0]/1.5 && pixel[2] < pixel[0]/1.5) ||
                                     (pixel[0] > 175 && pixel[1] < pixel[0] && pixel[2] < pixel[0]/2) ||
                                     ((pixel[0] > 250 && pixel[1] < pixel[0] && pixel[2] < 200) && pixel[0] + pixel[1] + pixel[2] < 600))
                                     ){

                    //ROS_INFO("image[%d,%d] = (r=%d,g=%d,b=%d)", i, j, pixel[2], pixel[1], pixel[0]);
                    if (i > (width/2 - VIEW_FINDER_WIDTH/2) && i < (width/2 + VIEW_FINDER_WIDTH/2) && j > height - VIEW_FINDER_HEIGHT){
                        ColorPoint cp = ColorPoint(image_matrix.at<cv::Vec3b>(j,i), i, j);
                        colorPoints.push_back(cp);
                    } else if (i > (width/2 - EXTERNAL_VIEW_FINDER_WIDTH/2) && i < (width/2 + EXTERNAL_VIEW_FINDER_WIDTH/2) && j > height - EXTERNAL_VIEW_FINDER_HEIGHT){
                        ColorPoint cp = ColorPoint(image_matrix.at<cv::Vec3b>(j,i), i, j);
                        outerColorPoints.push_back(cp);
                    }

                } else {
                    pixel[0] = 0;
                    pixel[1] = 0;
                    pixel[2] = 0;
                }
            } catch(int e){
                ROS_INFO_STREAM("ERROR");
            }
        }
    }
    cv::imshow("edited_window", image_matrix);

    ROS_INFO_STREAM("END OF CALLBACK");

    hasCallback = true;
}

void saveOuterBias(){
    int error_factor = 2;
    int sectionACount = 0, sectionBCount = 0, sectionCCount = 0;
    float startX = 640.0f/2.0f - EXTERNAL_VIEW_FINDER_WIDTH/2.0f;

    for (int i = 0; i < outerColorPoints.size(); i++){
        ColorPoint cp = outerColorPoints.at(i);
        if (cp.x < startX + EXTERNAL_VIEW_FINDER_WIDTH/3.0f){
            sectionACount++;
        } else if (cp.x < startX + EXTERNAL_VIEW_FINDER_WIDTH * 2.0f/3.0f) {
            sectionBCount++;
        } else {
            sectionCCount++;
        }
    }
    int largest_section = 0;
    if (sectionACount * error_factor > sectionBCount && sectionACount * error_factor > sectionCCount){
        largest_section = 1;
    } else if (sectionCCount * error_factor > sectionACount && sectionCCount * error_factor > sectionBCount){
        largest_section = 3;
    } else {
        largest_section = 2;
    }

    if (recentHistoryBias.size() > 35){
        recentHistoryBias.erase(recentHistoryBias.begin());
    }
    recentHistoryBias.push_back(largest_section);
}

int determineBestTurn(){
    int sectionACount = 0, sectionBCount = 0, sectionCCount = 0;
    for (int i = 0; i < recentHistoryBias.size(); i++){
        int turnDirection = recentHistoryBias.at(i);
        if (turnDirection == 1){
            sectionACount++;
        } else if (turnDirection == 2){
            sectionBCount++;
        } else {
            sectionCCount++;
        }
    }
    int return_value = 0;
    if (sectionACount > sectionBCount && sectionACount > sectionCCount){
        return_value = 1;
    } else if (sectionBCount > sectionACount && sectionBCount > sectionCCount){
        return_value = 1;
    } else {
        return_value = -1;
    }
    return return_value;
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

    ros::Subscriber diagnosticSubscriber = nh.subscribe("/diagnostics_agg", 1000, &diagnosticMessageRecieved);
    ros::Subscriber bumpWheeldropSubscriber = nh.subscribe("/bump_wheeldrop", 1000, &bumpWheeldropMessageReceived);
    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);

    ros::Rate rate(RATE);
    while(ros::ok()){
        if (hasCallback){
            geometry_msgs::Twist twistObject;
            ROS_INFO_STREAM("POINT COUNT: " << colorPoints.size());


            saveOuterBias();
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
                switch(largest_section){
                    case 1:
                        ROS_INFO_STREAM("LEFT");
                        twistObject.angular.z = M_PI/8.0f/RATE;
                        twistObject.linear.x = 2.0f/RATE;
                        break;
                    case 2:
                        ROS_INFO_STREAM("STRAIGHT");
                        twistObject.linear.x = 3.0f/RATE;
                        break;
                    case 3:
                        ROS_INFO_STREAM("RIGHT");
                        twistObject.angular.z = -M_PI/8.0f/RATE;
                        twistObject.linear.x = 2.0f/RATE;
                        break;
                    default:
                        break;
                }
                allow_change_of_direction = true;

            } else if (colorPoints.size() >= 0){
                //turn
                ROS_INFO_STREAM("NOT ENOUGH TAPE");
                if (allow_change_of_direction){
                    turn_value = determineBestTurn();
                }
                allow_change_of_direction = false;
                twistObject.angular.z = turn_value *  M_PI/RATE;
            } else {
                twistObject.linear.x = -1;
            }

            if (!motionAllowed()){
                twistObject.linear.x = 0;
                twistObject.angular.z = 0;
            }

            twistPublisher.publish(twistObject);
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("\n");

    }
    cvDestroyWindow("unedited_window");
    cvDestroyWindow("edited_window");
}

