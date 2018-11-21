/**
 *  MIT License
 *
 *  Copyright (c) 2018 Rishabh Choudhary
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    main.cpp
 *  @author  Rishabh Choudhary
 *  @copyright MIT License
 *  @date    11/19/2018
 *  @version 1.0
 *  
 *  @brief  ENPM808X : Assignment to implement walker algorithm for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This file will run the turtlebot_walker node 
 *
 */

#include "../include/turtlebot_walker/turtlebotWalker.hpp"

/**
 * @brief      main function
 *
 * @param[in]  argc  
 * @param[in]  argv  
 *
 * @return     0 for successful execution
 */
int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "turtlebot_walker");
  // Create the turtlebotWalker object
  turtlebotWalker walker;
  // Navigate the turtlebotWalker 
  walker.navigateBot();
  return 0;
}
