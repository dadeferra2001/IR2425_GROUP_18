
## 👋 Introduction
This is the first assignment for the course _Intelligent robotics_.

The group members are:
- Ferrari Davide
- Tomasi Danny
- Vettoretti Giacomo


## 📖 Instructions
First of all you need to download the package:
~~~
cd ~/catkin_ws/src
git clone https://github.com/dadeferra2001/IR2425_GROUP_18.git
~~~

Afterward, you can build the package:
~~~
cd ..
catkin build
~~~

Once you built, you can start launch the simulation, navigation stack and april tag server/generator using:
~~~
roslaunch assignment1 setup.launch
~~~

Finally, you can run node A and B (action_client.cpp and action_server.cpp) using a single launch file. This will proceed to get the target IDs from the service and will handle the navigation:
~~~
roslaunch assignment1 run.launch
~~~