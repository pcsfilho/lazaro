# Lazaro V-REP
This repository presents a project which establishes a connection between the client side developed with the help of python language and the VREP tool. It implemented the logic in a robotnic should find a black cube on the maze.

## Tools and Requirements

**Vrep**:
[V-REP](http://www.coppeliarobotics.com/) is a robot simulator, with integrated development environment.Controllers can be written in C/C++, Python, Java, Lua, Matlab or Octave. The version used in this project was V-REP 3.6.2.

**Python**:
Python version v3.6 or greater 

**Operational system**
Linux 64x

## Run
- **First first you must download the project in git:**

  ```bash
    $ git clone https://github.com/pcsfilho/lazaro.git
    ```
- **After downloading the project you must enter the folder:**

    ```bash
        $ cd lazaro
    ```

- **You must open the scene in VREP. The scene is located in the vrep_content folder and is named challenge_scenario.ttt.**
- **After opening the scene and starting the scene, you can start the python script for the control and execution of the map search.**:

    ```bash
        $ python python_workspace/main
    ```

## Project
This project is divided into two main classes:
- **Simulation**
    - Represents the Map with its respective attributes and methods.
        - Atributes:
            - clientID : Parameter id of the client with api remote
        - Methods:
            - start_connection(server, port): Method that initiates the connection with server
            - stop_connection(server): Method that stop the connection with server
        
- **Robotnik**
Represents the robotnik with attributes and sensors.
    - Atributes:
        - sensor_vision:
        - proximity_sensor_1: This proximity sensor is responsible for detecting collisions in forward movements
        - proximity_sensor_2: This proximity sensor is responsible for detecting collisions in left movements
        - proximity_sensor_3: This proximity sensor is responsible for detecting collisions in right move
        - vision_sensor: This sensor is responsible for detecting the cube with reference color 1 in the map
    - Methods:
        - object_was_found: verify if exist reference color in the image generated by sensor vision
        - make_trajectory: causes the quadcopter to rotate the map until it finds the cube
        - foward_more: Set foward movement to point value
        - side: Check the current orientation and return the radian side

## Documentation
This project was based on
- [VREP](http://www.coppeliarobotics.com/helpFiles/)
- [Forum](http://www.forum.coppeliarobotics.com)

## Project decisions
    To perform the search of the black cube on the map, the technique called right-hand rule was used.
    This was made possible because the map structure looks like a maze without loops. This left hand 
    on wall algorithm can be simplified into these simple conditions:
-  If you can turn right then go ahead and turn right:

- else if you can continue driving straight then drive straight:

- else if you can turn left then turn left:

- If you are at a dead end then turn around:


    For this project defined the following settings for the vision sensor robotnik:

![Optional Text](https://github.com/pcsfilho/lazaro/blob/master/img/pespective.png)

    Just like a color selection filter for cube visualization in simulation:

![Optional Text](https://github.com/pcsfilho/lazaro/blob/master/img/filter.png)

## Improvements
In the future it is expected to improve the search algorithm in order to find the object more accurately.
In addition to improving collision correction.