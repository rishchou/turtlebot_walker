#include "../include/turtlebot_walker/turtlebotWalker.hpp"


int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "turtlebot_walker");
  // Create the turtlebotWalker object
  turtlebotWalker walker;
  // Run the turtlebotWalker behaviour
  walker.navigateBot();
  return 0;
}
