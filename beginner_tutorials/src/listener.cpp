 #include <Aria.h>
 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <std_msgs/String.h>

 ArRobot robot;
 
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
 {
   ROS_INFO("Received %f %f %f %f %f %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
   robot.setVel(msg->linear.x);
   robot.setRotVel(msg->angular.z);
 }
 
 int main(int argc, char** argv)
 {
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
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
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
  ros::spin();
  
  //stop the robot
  robot.waitForRunExit();
  Aria::exit(0);
 }
