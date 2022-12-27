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
[Second submission: 7pts]</br>
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
* the documentation.
* a print of the "Get Execution Outcome" response, sent after a succsefful run of the problem.
* a link to a video screen recording of you sending the request so we see that everything is running well (Since you are not running a real robot or a Gazebo simulation, the skills will print their output on the terminal where the AOS server is running.).



## Task 3.2
In this task, you will meet some of the challenges of decision making under uncertainty.</br> </br>



#### Problem description:
Every afternoon, a mother returns from work, picks up her baby from daycare and comes home.
She wants to rest a bit before his big brothers come, but her baby wants to play, and he is crying for her to come. She activates her home assistant robot and sleeps quietly.</br>
The robot is programmed to bring the baby toys he enjoys. He has four toys, which we refer to by their colors: green, blue, black, and red. The baby plays with each toy for a certain amount of time, depending on his mood.</br>
The location of each toy depends on where the baby left them yesterday.</br>
The robot can navigate between the four possible locations of toys and the baby's location (they are described as locations 0-4, where 4 is the baby's location). The pick skill must know the exact toy type for a successful pick ('green,' 'blue,' 'black,' or 'red'), and the baby does not allow the robot to pick the toys he received. The robot cannot pick more than one toy at a time and must be located at the toy location. The pick skill accurately reports success or failure if these conditions were met. A successful pick cost is -1, but when it fails, it takes more time and costs -2.</br>
The place skill return success if the robot is actually holding a toy. The place skill cost is -3 when it fails (if not holding a toy) and -1 otherwise. navigate skill cost is -3, with an additional penalty of -1 if asked to navigate its current location (which may cause orientation loss).</br>
</br>


How the toys are ordered:</br>
Every evening the cute baby throws the green toy to one of the locations. The probability of the green toy to be placed in the locations is [0:0.1, 1:0.05, 2:0.8, 3:0.05]. Next, he throws the blue toy to one of the remaining locations with a probability of [0:0.7, 1:0.1, 2:0.1, 3:0.1] (the probability of the already occupied location is evenly distributed to the other locations), then he throws to the black toy (both remaining locations have the same chance here). The red toy is thrown to the fourth location.

Remember that when using automated planning, you need to describe the problem, not solve it. Use the "InitialBeliefStateAssignments" in the environment file to describe the initial location of each object. Just tell the story using code :).</br>
</br>
As said, the baby plays with each toy for a different number of minutes, depending on its mood. The mother observed the baby's behavior and saw that he plays with his toys for periods of [10,20,10,40] minutes. First, he thinks about how much he likes the green toy and assigns a play time for it with a discreet distribution of p([10=0.8,20=0.05,10=0.1,40=0.05]), then for the blue toy with p([10=0.1,20=0.7,10=0.1,40=0.1]), next it uniformly selects a period for the black, and the red toy receives the remaining period.
These periods are rewards given when the robot places each toy on the baby's lap (they are the positive place skill reward in these cases).
</br></br>
The robot is allowed to use the pick skill only six times.
The task ends (terminal states) when the robot gives the baby all of the toys, or when it uses all of its pick actions and is not currently holding a toy, or if it uses the pick skill more times than allowed. The robot's initial location is near the child. </br>
To solve this problem, you will write a default policy.</br>

### Preliminary instructions:
Download the environment to your catkin/src directory.</br>
Place the `objects` directory (located inside `task3_env`) in your `~/.gazebo/models` directory (override old files if needed). </br>
Build the code using `catkin_make` </br>
Run the launch file, and see how exposed services affect the environment.</br> 
The Gazebo and Rviz will start after running the `task3_env` launch file. The environment starts so that only the robot is visible. After the first skill call, the toys and child objects are generated. The toys are generated as colored balls and the child as a blue box. When picking a toy, it is teleported to an area outside the room. If a toy is placed near the child, it is teleported to a different area outside the room (not to deal with obstacles avoidance).</br>
</br>
Commands you should be able to run after `catkin_make`:</br>
to run the environment: `roslaunch task3_env task3_env.launch`</br>
to navigate to locations (0-4): `rosservice call /navigate "location: 0"`</br>
to pick toys when the robot is near them (toy_type can take 'green',`blue`,`black` or `red`): `rosservice call /pick "toy_type: 'green'"` </br>
to place a toy: `rosservice call /place "{}" `</br>
to receive internal information on the real state of the world (you are not allowed to use this service in your code): `rosservice call /info "{}" `</br>


### 3.2.1 Connect the AOS to the robot skills
[Second submission: 8pts]</br>
In this first phase, you are requested to create and test the basic building blocks for connecting a robot to the AOS.</br>
The documentation should include an environment file, an SD file, and AM for each skill. Your goal is to allow the AOS to activate a sequence of actions without automatic decision-making. Since we do not activate the decision making algorithm, and just want to see that you can activate skills using the AOS at this phase,  
you don't need to model correctly the effects of each skill and the probability of observations.
Therefore, the environment, SD, and AM files can be almost empty. You will modify them
in the next step. </br>

Steps:</br>
* Document the robot's environment and skills.
* Send an HTTP  "integration" request (with internal simulation).
* Send an HTTP "get possible actions" request.
* Send an HTTP "integration" request with a sequence of actions for execution (you should video record their execution).
</br></br>

Skills description:</br>
* See the services srv files that describe their parameter.
* See the `skills_server.py` (located in `task3_env/scripts`) for the services paths.

#### You need to submit the following:
* The documentation.</br>
* A link to the video recording on youtube.</br>

### 3.2.2 Finish the documentation and run the robot
[Second submission: 11.5pts]</br>
In this phase, you are requested to finalize the project documentation and run the robot to solve the task. Now, you need to properly model the environment, effects, and observations based on
the description below.</br>

#### You need to submit the following:
* The documentation.
* A link to the video recording on youtube.
* Run the experiment ten times and submit the output of calling `rosservice call /info "{}" after the robot stops at each experiment. 


## Task 3.2.3 Solving the problem with coding (Not by the AOS)
[Second submission: 11.5pts]</br>
Write a ROS node (using python) that calls the needed services to perform task 3.2.2.</br>
Activate all the services (you can use the launch file) and your node.</br>
</br></br>
You need to submit the following:
* Your code.
* A YouTube link to a video recording of your code running and operating the robot.
* Run the experiment ten times and submit the output of calling `rosservice call /info "{}" after the robot stops at each experiment.
