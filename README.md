# Continual Learning for Affective Robotics: A Proof of Concept for Wellbeing

This repository holds the source code for the paper Continual Learning for Affective Robotics: A Proof of Concept for Wellbeing published at the AR4W: Affective Robotics for Well-being Workshop at the 10th International Conference on Affective Computing and Intelligent Interaction, 2022.

<br />
<p align="center">
    <img src="dialogue_manager/img/Experiment.png" alt="Experiment Design" width="680" height="300">
  </a>
</p>

<p>&nbsp;</p>


## Pepper Robot

<img src="dialogue_manager/img/PepperRobot.png" align="right" width="250px" />

SoftBank's *Pepper Robot* is used for testing and experiments.

For controlling the robot, **Python 2.7** is used with the official library of the<br>SoftBank, **NAOqi 2.5.7**.

Robot can be used either with the official simulation software of the SoftBank,
 **Choregraphe**, or with the physical robot itself.

<p>&nbsp;</p>
<p>&nbsp;</p>
<p>&nbsp;</p>

## FaceChannel

Project uses the **FaceChannel** neural network that is a light-weight neural network to recognize emotion from facial expressions with much fewer parameters than common neural networks.


![FaceChannel](dialogue_manager/img/FaceChannel.png)

<br>

### Related Publication

```sh
P. Barros, N. Churamani and A. Sciutti,  "The FaceChannel: A Light-Weight Deep Neural Network for Facial Expression Recognition.," in 2020 15th IEEE International Conference on Automatic Face and Gesture Recognition (FG 2020) (FG), Buenos Aires, undefined, AR, 2020 pp. 449-453.
doi: 10.1109/FG47880.2020.00070
keywords: {emotion recognition;deep learning}
url: https://doi.ieeecomputersociety.org/10.1109/FG47880.2020.00070
```

### [GitHub repo of the FaceChannel](https://github.com/pablovin/FaceChannel)

<br>

## Smach

Project utilizes a Python library named **Smach**. This allows us to develop a state machine structure to implement the script easily. Below is the structure of the demo state machine:

![State Machine](dialogue_manager/img/Smach.png)

A custom state machine can be created and used for different purposes.

<br>

## Requirements

Essentials:

* Install all the libraries in the *requirements.txt* file.

* Install CUDA 9 ([Instructions](https://github.com/akirademoss/cuda-9.0-installation-on-ubuntu-18.04))

* Install **ROS (Robot Operating System)**
  
  >*ROS Melodic* with *Ubuntu 18.04* preferably

* Install **NAOqi** Python SDK ([Download link](https://developer.softbankrobotics.com/pepper-naoqi-25-downloads-linux))

  >*NAOqi 2.5.7* preferably
  
* Install *checkpoint.zip* file ([Download link](https://drive.google.com/file/d/1I7vI4omJtjVA2205UMaRSk5J1FajlK_o/view?usp=sharing))

  >It is necessary for using the CL model

To use Choregraph simulation and load the behaviors to Pepper Robot:

* Install **Choregraphe** (Choregraphe is a must for both the physical and the virtual Pepper Robot)

To display view Smach states live:

* Install **smach_viewer** ([Documentation Link](http://wiki.ros.org/smach_viewer)) ([Tutorial](http://wiki.ros.org/smach/Tutorials/Smach%20Viewer))

  To use smach_viewer, uncomment lines that contain **sis** from *state_manager.py*

<br>

## Instructions

<br />
  <a href="https://drive.google.com/file/d/1NOUQQdYElSrAGtRTWOEKKw_PDEQ6Fkng/view?usp=sharing">
    <img src="dialogue_manager/img/VideoBanner.png" alt="Video Banner" width="640" height="360">
  </a>

 >🎥 To watch the video about the technical details of the project, [click](https://drive.google.com/file/d/1NOUQQdYElSrAGtRTWOEKKw_PDEQ6Fkng/view?usp=sharing) the image above.

After installing ROS, a workspace and then a package should be installed. For advanced guide, see:

:warning: ROS package name should be *'dialogue_manager'*. Otherwise, change the name from the project files.

* [Creating a ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

* [Creating a ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

After creating the package, put the project files inside the *dialogue_manager* folder into the ROS package's folder.

Extract *checkpoint.zip* file and put the folder named "checkpoint" inside the "CLModel" folder in the ROS package. The path should be 

```sh
dialogue manager > src > CLModel > checkpoint
```

Then, go to the *catkin_ws* folder and run:

```sh
$ catkin_make
```

Upload the Pepper robot behaviors to physical or virtual robot via Choregraphe software. Behavior files are located in 

```sh
dialogue_manager > src > Behaviors
```

Custom behaviors can be created and implemented to the system using the Choregraphe software.

<br>

Demo state machine is created to test three different conditions, where the first condition implements only the script for interaction, the second condition uses FaceChannel for emotion detection and third one uses the Continual Learning model for personalization.

After running the program, it will create a folder named "SessionReports" **automatically** which will be holding the log files and image outputs.

The condition is selected randomly in the *state_manager.py*

To define the **IP** and **PORT** values of the robot, use *config.txt*. (The file can be modified with different values, but the order should be covered to run the demo.)

For most of the computers, setting **MICROPHONE_NAME** to 'default' in *config.txt* would work. To find all microphones' names, run this:

```python
import speech_recognition as sr
print sr.Microphone.list_microphone_names()
```

<br>

### Launch files

There are two launch files in the project:

1. **ComputerDialogueManager.launch**: To run the project using computer's own camera and speakers, use this file.

2. **PepperDialogueManager.launch**: To run the project using the Pepper Robot itself, or with the Choregraphe's simulation, use this file.

To run the demo, just type

```sh
$ roslaunch dialogue_manager PepperDialogueManager.launch session:=experiment_1 name:=atahan save_frames:=1
```

Launch files' arguments:

* *session* is the name of the session that will be used in the name of the log files and frames' main folder. Same session name **can** be used for different sessions. Default is 'Experiment'

* *name* is the user's name who will interact with the robot. Default is 'User'.

* *save_frames* determines whether the camera output frames will be saved or not. If the given value is **1**, it saves all the frames in the session folder. Default is 0.

The log files and frames can be found in the session folder that is located in *dialogue_manager/src/SessionReports*. The folder will be created after first run.

<br>

### Log Files

After each session, three logs files are created:

* *arousal_valence.log* is the file that holds the arousal-valence values of each frame with timestamps. The calculations are made with the FaceChannel framework.

* *flow.log* holds a table that shows all the dialogues of the robot and the user with timestamps. The table can be easily converted to other formats.

* *main.log* holds all events from all files with timestamps


<br>

## Acknowledgement
For the purpose of open access, the authors have applied a Creative Commons Attribution (CC BY) license to any Accepted Manuscript version arising. 

**Funding:** N.Churamani is funded by the EPSRC under grant EP/R1513180/1 (ref.2107412). M.Axelsson is funded by the Osk. Huttunen foundation and the EPSRC under grant EP/T517847/1. H.Gunes' work is supported by the EPSRC under grant ref. EP/R030782/1. A.Çaldır contributed to this work while undertaking a summer research study at the Department of Computer Science and Technology, University of Cambridge.

**Data Access Statement:** Overall statistical analysis of research data underpinning this publication is contained in the manuscript. Additional raw data related to this publication cannot be openly released as the raw data contains videos and transcripts of the participants' interaction with the robot, which were impossible to anonymise.
```
@INPROCEEDINGS{churamani2022CL4HRI,  
  author = {Churamani, Nikhil and Axelsson, Minja and Caldir, Atahan and Gunes, Hatice},
  booktitle   = {Proceedings of the 10th International Conference on Affective Computing and Intelligent Interaction Workshops and Demos (ACIIW)},   
  title = {{Continual Learning for Affective Robotics: A Proof of Concept for Wellbeing}},
  year      = {2022},  
  publisher   = {IEEE},
}
```

## Contact

[![Nikhil](https://img.shields.io/badge/nikhil_churamani-nikhil.churamani@cl.cam.ac.uk-yellow?style=for-the-badge&logo=mail)](mailto:nikhil.churamani@cl.cam.ac.uk)

[![Minja](https://img.shields.io/badge/minja_axelsson-minja.axelsson@cl.cam.ac.uk-red?style=for-the-badge&logo=mail)](mailto:minja.axelsson@cl.cam.ac.uk)

[![Atahan](https://img.shields.io/badge/atahan_caldir-atahan.caldir@ozu.edu.tr-blue?style=for-the-badge&logo=mail)](mailto:atahan.caldir@ozu.edu.tr)

[![Hatice](https://img.shields.io/badge/prof_hatice_gunes-hatice.gunes@cl.cam.ac.uk-green?style=for-the-badge&logo=mail)](mailto:hatice.gunes@cl.cam.ac.uk)
