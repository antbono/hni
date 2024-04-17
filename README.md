[![Build (humble)](https://github.com/antbono/hni/actions/workflows/build_and_test_humble.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_humble.yaml)
[![Build (iron)](https://github.com/antbono/hni/actions/workflows/build_and_test_iron.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_iron.yaml) 
[![Build (rolling)](https://github.com/antbono/hni/actions/workflows/build_and_test_rolling.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_rolling.yaml)

# Human NAO Interaction (HNI)

This package provides the user with some very useful capabilities for HRI experimentation with the NAO v6 robot.

It is part of the [Open Access NAO](https://github.com/antbono/OAN) framework.


## Main features

At the moment, you can exploit ithe package for the following activities:

- *Teach by demonstration*: The  `joints_record` node allows to record the movements of the joints you are interested in. In this way,
you need only move the limbs of interest in the desired manner and the robot will learn the movement. The smooth reproduction of the movement is then handled by the `joints_play_action_server` node.

- *Face/object detection and tracking*: Thanks to the Yolov8 model, the robot can detect faces and objects in a real time video stream. This info is then exploited to move the head in order to mantain eye contact with the face/object.

- *Speech recognition and synthesis*: Using the APIs offered by the Google Cloud Platform, we implemented Speech-To-Text and Text-To-Speech Services.

- *Conversational ability*: We leverage on the ChatGPT model to implement NLP on the NAO. It is possible, indeed, to modify the personality of the assistant or provide specific instructions about how it should behave throughout the conversation.


For more info, please refer to the related [paper](https://arxiv.org/abs/2403.13960).


## Usage

For a quick test, you can reproduce the HRI experiment described in the aformentioned paper and showed on [YouTube](https://youtu.be/LxboNtHfDJg?si=1951kaU84Miw7Ubb).


Open a terminal on the nao

	ssh nao@<nao_ip_address>

On that terminal run

	cd <your_oan_ws>
	source install/local_setup.bash
	ros2 launch hni_cpp experiment_nao_launch.py

Now on your machine install xterm if you don't have it

	sudo apt install -y xterm

and then run

	cd <your_oan_ws>
	source install/local_setup.bash
	ros2 launch hni_cpp experiment_pc_launch.py

You will see an xterm terminal popping up. Use it to start and run the experiment.

Enjoy it! :D

