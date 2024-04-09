[![Build (humble)](https://github.com/antbono/hni/actions/workflows/build_and_test_humble.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_humble.yaml)
[![Build (iron)](https://github.com/antbono/hni/actions/workflows/build_and_test_iron.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_iron.yaml) 
[![Build (rolling)](https://github.com/antbono/hni/actions/workflows/build_and_test_rolling.yaml/badge.svg)](https://github.com/antbono/hni/actions/workflows/build_and_test_rolling.yaml)

# Human NAO Interaction (HNI)

This package is part of the [Open Access NAO](https://github.com/antbono/OAN) framework.

It provides the user with some very useful capabilities for HRI experimentation.


## Main features

At the moment, you can exploit ithe package for the following activities:

- *Teach by demonstration*: The  `joints_record` node allows to record the movements of the joints you are interested in. In this way,
you need only move the limbs of interest in the desired manner and the robot will learn the movement. The smooth reproduction of the movement is then handled by the `joints_play_action_server` node.

- *Face/object detection and tracking*: Thanks to the Yolov8 model, the robot can detect faces and objects in a real time video stream. This info is then exploited to move the head in order to mantain eye contact with the face/object.

- *Speech recognition and synthesis*: Using the APIs offered by the Google Cloud Platform, we implemented Speech-To-Text and Text-To-Speech Services.

- *Conversational ability*: We leverage on the ChatGPT model to implement NLP on the NAO. It is possible, indeed, to modify the personality of the assistant or provide specific instructions about how it should behave throughout the conversation.


For more info, please refer to the related [paper](https://arxiv.org/abs/2403.13960).


### Note on CI

The failing status is due to the colcon `--symlink-install` option that [action-ros-ci](https://github.com/ros-tooling/action-ros-ci) uses. At the moment the symlinking cannot be disabled but we are actively working with action-ros-ci maintainers (see issue [815](https://github.com/ros-tooling/action-ros-ci/issues/815)). 