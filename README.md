# Mental Well-being Robot Coach with Pepper Robot

This repository holds the code for the Mental Well-being Robot Coach project that is created in the Affective Intelligence and Robotics Lab (AFAR Lab) in the University of Cambridge. The main idea is to test the effects of Continual Learning in social robots' interaction.

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://www.cl.cam.ac.uk/~hg410/people.html">
    <img src="dialogue_manager/img/Cambridge.png" alt="Logo" width="300" height="100">
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

### [Source code for the FaceChannel framework](https://github.com/pablovin/FaceChannel)

<br>

## Requirements

Install all the libraries on the requirements.txt file.

Install **ROS (Robot Operating System)** (*ROS Melodic* with *Ubuntu 18.04* preferably)

(*Optional*) Install **Choregraphe**

<br>

## Instructions

Current files is created to test three different conditions, where the first condition is implements only the script for interaction, the second condition uses FaceChannel for emotion detection and third one use the Continual Learning model for personalization.

The condition is selected randomly in the state_manager.py

The script can be found ....

A custom log manager is created for the logging (log_manager.py). The log files can be found in the "logs" directory. Every session needs to be specified with a different log file name which can be passing to the launch file as argument.

The 'config.txt' file can be modified with different values but the order should be covered to run the demo.

To run the demo, just type

```sh
$ roslaunch dialogue_manager pepperRunAll.launch logger:=experiment_1 name:=atahan
```

where *logger* is the name of the session that will be used in the name of the log file and *name* is the user's name who will interact with the robot.

The current codes are using the Pepper Robot for interaction. If Choregraphe simulation aimed to be used, in the launch file, pepper_camera_publisher.py and pepper_tts_service.py should be changed with camera_publisher.py and text_to_speech_service.py, respectively.

<br>

## Contact

[![Hatice](https://img.shields.io/badge/dr_hatice_gunes-hg410@cam.ac.uk-green?style=for-the-badge&logo=mail)](mailto:hg410@cam.ac.uk)

[![Nikhil](https://img.shields.io/badge/nikhil_churamani-nc528@cam.ac.uk-yellow?style=for-the-badge&logo=mail)](mailto:nc528@cam.ac.uk)

[![Minja](https://img.shields.io/badge/minja_axelsson-mwa29@cam.ac.uk-red?style=for-the-badge&logo=mail)](mailto:mwa29@cam.ac.uk)

[![Atahan](https://img.shields.io/badge/atahan_caldir-athncldr@ozu.edu.tr-blue?style=for-the-badge&logo=mail)](mailto:atahan.caldir@ozu.edu.tr)
