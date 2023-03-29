# gamepad_snakebot_control
Code for teleoperation of a concentric tube robot through a gamepad

This builds on from the paper in 2017: "Teleoperation of a concentric tube robot through Hand Gesture tracking" https://www.semanticscholar.org/paper/Teleoperation-of-a-concentric-tube-robot-through-Razjigaev-Crawford/3ce78fca22d2970e5e97562a4e1ec980df1869c2#citing-papers

This repo has two Microsoft Visual Studio Projects compiled in Debug x86.

- AndrewsLeapAttempt - has gamepad control for the snakebot [AndrewLeapAttempt.cpp](https://github.com/Andrew-Raz-ACRV/gamepad_snakebot_control/blob/main/AndrewsLeapAttempt/AndrewsLeapAttempt/AndrewsLeapAttempt.cpp)

- XInput_demo - example of gamepad interfacing [main code](https://github.com/Andrew-Raz-ACRV/gamepad_snakebot_control/blob/main/XInput_demo/code/main.cpp)

Note: AndrewsLeapAttempt requires the Nuget package of Eigen to compile
