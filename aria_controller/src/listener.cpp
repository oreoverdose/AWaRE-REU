 #include <Aria.h>
 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>

 #include <nav_msgs/Odometry.h>
 #include <tf/transform_broadcaster.h>

 ArRobot robot;
 
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg){ 
   //mm/s vs m/s
   robot.setVel(msg->linear.x*1e3);
   //deg/s vs rad/s
   robot.setRotVel(msg->angular.z*180/3.141592);
   
   ROS_INFO("DATA: posX-%f posY-%f vel-%f yaw-%f",robot.getX(),robot.getY(),robot.getVel(),robot.getTh());
   ROS_INFO("RECEIVED: vel- %f, angVel-%f",msg->linear.x*1e3,msg->angular.z);
 }
 
 int main(int argc, char** argv){
   //Initialize Aria and a parser for the commands
   Aria::init();
   ArArgumentParser parser(&argc,argv); 
 
   //Connect the robot with the parser
   ArRobotConnector robotConnector(&parser,&robot);
   if (!robotConnector.connectRobot()){ 
	if (!parser.checkHelpAndWarnUnparsed()){
		ArLog::log(ArLog::Terse,"error");
	} else {
		ArLog::log(ArLog::Terse,"error");
		Aria::logOptions();
		Aria::exit(1);
	}
  }
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()){    
    Aria::logOptions();
    exit(1);
  }

  // Define a keyHandler  
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);

  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  // Give time to finish all the settings
  ArUtil::sleep(4000);
  printf("Ready\n");

  //starts the robot
  robot.runAsync(true);

  robot.comInt(ArCommands::ENABLE, 1);

  // Ros initialization
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber chatter_sub = n.subscribe("/cmd_vel", 100, chatterCallback);
  
  //ros::Publisher theta_pub = n.advertise<std_msgs::Float32>("theta",1);
  //ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("vel",1);
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1);
  
  nav_msgs::Odometry odom;
  bool meow = true;
  while(meow){
  	odom.header.stamp = ros::Time::now();
  	odom.header.frame_id = "odom";
  	//set the posiiton
  	odom.pose.pose.position.x = robot.getX();
  	odom.pose.pose.position.y = robot.getY();
  	odom.pose.pose.position.z = 0.0;
  	
  	//////////////////FIX////////////////////
  	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot.getTh()*3.141592/180);
  	odom.pose.pose.orientation.w = robot.getTh()*3.141592/180;
  	////////////////////////////////////////  
  	
  	odom.child_frame_id = "base_link";
  	odom.twist.twist.linear.x = robot.getVel()/1e3;
  	odom.twist.twist.linear.y = 0.0; //hmm...
  	odom.twist.twist.angular.z = robot.getRotVel()*3.141592/180;
  	
  	odom_pub.publish(odom);
  	
  	ros::spinOnce();
  }
  
  //stop the robot
  robot.waitForRunExit();
  Aria::exit(0);
 }
