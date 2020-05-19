# Robot-autonomy-for-camera-control-assistance

<b> Development update 1/3/20 </b>

Developed a system for providing better camera viewpoints for high/low level control of robotic arm in nursing tasks.
<pre>
─ Camera switching for reducing the cognitive workload of the user
─ Framework for autonomous \camera control using robotic arm
─ Detecting and following the moving object in manipulation hand while keeping a safe distance to avoid collision
</pre>
<pre>
─ Three modes of operation
▪ Joint state control
▪ Cartesian control
▪ Objective control
</pre>
<b> Dynamic weight update of objective functions for control of camera arm </b>


<b> RelaxedIK Solver </b>

Welcome to RelaxedIK! <i> RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion </i> (http://www.roboticsproceedings.org/rss14/p43.html)

Video of presentation at RSS 2018 (RelaxedIK part starts around 12:00) :
https://youtu.be/bih5e9MHc88?t=737

Video explaining relaxedIK
https://youtu.be/AhsQFJzB8WQ

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.


<b> Dependencies </b>

kdl urdf parser:
<div> >> sudo apt-get install ros-[your ros distro]-urdfdom-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-parser-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-conversions </div> 
<div> >> numba version 0.47.0 </div>
<div> >> llvm-config version 6.0.0 </div>
<div> >> llvmlite version 0.31.0 </div>
<div> >> python-fcl 0.0.12 </div>
<div> >> scikit-learn 0.20.4 </div>
<br>

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl


scikit learn:
http://scikit-learn.org/stable/index.html


<b> Tutorial </b>

<pre> roslaunch relaxed_ik gazebo.launch </pre>
<pre> rosrun qt_kinova_pkg qt_kinova_pkg </pre>

<b> Video demonstration </b>

[![Watch the video]](mp.png)(https://www.youtube.com/watch?v=lwVXYiFdlOc&feature=youtu.be)
