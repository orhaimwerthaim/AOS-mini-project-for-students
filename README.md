# Task 3
In this task, you will take your first steps using the AOS. The AOS advantage is mainly in robotic domains where the code effects are probabilistic, and the current state of the world is not observable to the robot. Decision-making in these domains is complex, and the AOS helps the robot engineer in this challenging task. The engineer should document: a) the robot's skills, b) The robot's environment, c) The robot's goals, and the AOS will automatically schedule the needed skills to reach the required goals.

* Before you start, you must read the [AOS documentation instructions](https://github.com/orhaimwerthaim/AOS-WebAPI/blob/master/docs/version2/AOS_documentation_manual.md), [sending HTTP requests to the AOS](https://github.com/orhaimwerthaim/AOS-WebAPI/blob/master/docs/version2/sending_HTTP_requests_to_the_AOS.md), and [practical instruction to integrate a project](https://github.com/orhaimwerthaim/AOS-WebAPI/blob/master/docs/version2/practical_instruction_to_integrate_a_project.md).</br>
</br>
We will start with a simple task where the skills have deterministic effects, and the state of the world is known.</br>
Your goal is to print 'hello world' on the screen. You are given two basic skills, one that prints a given letter to the screen and another that can print the same letter multiple times.</br>

## Task 3.1.1 Solving the problem with coding (Not by the AOS)
[First submission: 7pts from total course grade with 3.1.2 environment file]</br>
Write a ROS node (using python) that calls the needed services to perform task 3.1.1.</br>
Activate all the services (you can use the launch file) and your node.</br>
</br></br>
You need to submit:
* Your ROS node code.
* A screenshot of the shell output when running it (printing the 'hello world' letters)

Detailed instructions:
* Download the basic skills from this repository located in 'task 3/letter_printer'.
* Place the 'letter_printer' folder in your ~/catkin_ws/src/ </br>
Change permissions for the skill files (`repeat_letter.py` and `single_letter.py`) in `/home/or/catkin_ws/src/letter_printer/scripts` so they can be executed as programs [see](https://askubuntu.com/questions/484718/how-to-make-a-file-executable).
* Build your ROS workspace using `cd ~/catkin_ws && catkin_make` 
* Test the skills by running their ROS nodes (you can do it with the prewritten launch file `roslaunch letter_printer servers.launch`).
* Check that they are working by invoking them from the command line using `rosservice call`
*  Create a ROS node the calls the different services to print 'hello world' (see [ROS service client tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)). 

## Task 3.1.2 
Your job is to solve the problem using the AOS. You will need to document the skills, environment, and goal.
To give you an easy start, we supply partial documentation that may have some errors in it (see the `printHelloWorld_template` directory). </br></br>

Detailed instructions: 
* Document each of the skills and write an environment file.
* run the AOS server (run the shell command `cd ~/AOS/AOS-WebAPI/bin/Debug/net5.0 && ./WebApiCSharp`)
* import the `task3.1_postman_collection.json` to POSTMAN and send the "run printer task" request to run your project (you will need to make some changes in the request body).

### Documentation instructions:</br>
You should define a single state variable that stores the printed letters.</br>
If more than 12 letters are printed, we reach a terminal state, and a penalty of -100 is given.</br>
If the 'hello world' is printed, a reward of 100 is given, and we reach a terminal state.</br>
* Possible values for skill parameters are: </br>
  * The 'single_letter' skill and the 'repeat_letter' skill have a 'letter' parameter that can take- 'h', 'e', 'l', 'o', ' ', 'w', 'r', 'd'. </br>
  * The 'repeat_letter' skill 'times' parameter can get :'1' or '2'.</br>
* The rewards/costs are:</br>
  * Applying 'single_letter' skill costs -1, and -1.9 for the `repeat_letter` skill.
  * And the rewards for terminal states.
</br> </br>

### Tips:
* You can use the POSTMAN 'run printer task' request to test your domain (it is in the collection you imported).
* See the WebAPI [documentation manual](https://github.com/orhaimwerthaim/AOS-WebAPI/blob/master/docs/version2/AOS_documentation_manual.md).
* You can run an internal simulation (it simulates execution without activating the skills).
* You can send the 'Get Execution Outcome' request to see how real or simulated execution of the robot occurred (sequences of state, action, observation, next state,...). 
* Once the project is integrated, you can send the 'Get Possible Actions' request and receive the possible actions (descriptions and IDs).
* After you document the project, you can test how actions are working on the robot by sending ''run printer task'' with "ActionsToSimulate" array containing the sequence of actions to run (make sure you are not running an internal simulation). 
* Errors can be spotted by looking at the terminal where the AOS is running. </br>
  
### What you need to submit
* the corrected documentation of the Environment file.
