#include "robot_gui/robot_gui_class.h"
#include "ros/init.h"

RobotGui::RobotGui() {
  ros::NodeHandle nh;
  info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>("/robot_info", 2, &RobotGui::infoCallback, this); // robot information subscriber
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 2, &RobotGui::odomCallback, this); // odometry subscriber
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10); // Twist Publisher
  service_client = nh.serviceClient<std_srvs::Trigger>(service_name); // Client for get distance 
}

void RobotGui::infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &imsg) {
  info_data_ = *imsg;
  ROS_DEBUG("info_data: %s", imsg->data_field_02.c_str() );
}

void RobotGui::odomCallback(
    const nav_msgs::Odometry::ConstPtr &omsg) {
  ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", omsg->pose.pose.position.x,
            omsg->pose.pose.position.y, omsg->pose.pose.position.z);
  odom_data_ = *omsg;
}


void RobotGui::run() {
  cv::Mat frame = cv::Mat(1000, 400, CV_8UC3);
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

 // TELEOPERATION BUTTONS
    
    if (cvui::button(frame, 140, 275, " Forward ")) {
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
    }

    if (cvui::button(frame, 140, 315, "   Stop  ")) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    if (cvui::button(frame, 65, 315, " Left ")) {
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
    }

    if (cvui::button(frame, 240, 315, " Right ")) {
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
    }

    if (cvui::button(frame, 140, 355, "Backward")) {
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
    }
    twist_pub_.publish(twist_msg);

 // CURRENT VELOCITY (Linear Velocity in x direction and Angular Velocity in Z direction)
    
    cvui::window(frame, 20, 400, 130, 60, "Linear velocity:");
    cvui::printf(frame, 30, 430, 0.5, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);
    cvui::window(frame, 200, 400, 130, 60, "Angular velocity:");
    cvui::printf(frame, 210, 430, 0.5, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

 // ROBOT INFORMATION
    
    cvui::window(frame, 10, 10, 350, 250, "Topic: " + info_topic_name);
    cvui::printf(frame, 15, 35, 0.5, 0xffffff,"Info received:");
    cvui::printf(frame, 15, 60, 0.5, 0xffffff,"1. %s", info_data_.data_field_01.c_str());
    cvui::printf(frame, 15, 85, 0.5, 0xffffff,"2. %s", info_data_.data_field_02.c_str());
    cvui::printf(frame, 15, 110, 0.5, 0xffffff,"3. %s", info_data_.data_field_03.c_str());
    cvui::printf(frame, 15, 135, 0.5, 0xffffff,"4. %s", info_data_.data_field_04.c_str());
    cvui::printf(frame, 15, 160, 0.5, 0xffffff,"5. %s", info_data_.data_field_05.c_str());
    cvui::printf(frame, 15, 185, 0.5, 0xffffff,"6. %s", info_data_.data_field_06.c_str());
    cvui::printf(frame, 15, 210, 0.5, 0xffffff,"7. %s", info_data_.data_field_07.c_str());
    cvui::printf(frame, 15, 235, 0.5, 0xffffff,"8. %s", info_data_.data_field_08.c_str());

 // ODOMETRY INFORMATION

    cvui::window(frame, 10, 460, 350, 200, "Topic: " + odom_topic_name);
    cvui::printf(frame, 15, 485, 0.5, 0xffffff,"Odometry Information");
    cvui::printf(frame, 15, 520, 1, 0xffffff,"1. X = %0.2f", odom_data_.pose.pose.position.x);
    cvui::printf(frame, 15, 555, 1, 0xffffff,"2. Y = %0.2f", odom_data_.pose.pose.position.y);
    cvui::printf(frame, 15, 590, 1, 0xffffff,"3. Z = %0.2f", odom_data_.pose.pose.position.z);

 // CALLING SERVICE TO GET DISTANCE

    cvui::window(frame, 50, 720, 250, 100, "Service: " + service_name);
    if (cvui::button(frame, 130, 675, "Call Service")) {
      if (service_client.call(srv_req)) {
        ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        last_service_call_msg = srv_req.response.message;
        service_call_counter++;
      } else {
        last_service_call_msg = "Service call failed.";
        service_call_counter = 0;
      }
    }

    if (not last_service_call_msg.empty()) {
      cvui::printf(frame, 60, 750, 0.5, 0xffffff, "Total Distance Travelled: ");
      cvui::printf(frame, 75, 780, 0.5, 0x00ff00, "%s",
                   last_service_call_msg.c_str());
    }

    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
  }
}