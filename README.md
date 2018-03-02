****************************************************************************
Title: Applying Deep Q Learning for Multi-Agent Robotic Soccer
Developer: Minuk Ma, Hyunjun Jang, Minjeong Ju, AIM lab, KAIST
Contact: akalsdnr@kaist.ac.kr, wiseholi@kaist.ac.kr, jujeong94@kaist.ac.kr
****************************************************************************

(0) Intro
2018 AI WorldCup is a 5 vs 5 robotic soccer simulation competition where the participants have to write AI codes.
Official host: http://mir.kaist.ac.kr/anouncement/view/id/316
Official github: https://github.com/aiwc
You need a good GPU for this competition. The simulation runs with webots(https://www.cyberbotics.com/overview). 
The recommended environment is python3, webots 2018a, tensorflow 1.4.1, etc. 

This code is for developing deep q learning based agent for 2018 AI WorldCup, KAIST.
We also wrote some rule-based codes. We use that to pre-train neural network for deep q learning.
The state is either image-based or coordinate-based.
The action space is continuous in principle, i.e. the left and right wheel speeds of the robots. 

(1) Directories
minuk/       => The root directory of our team's code. It should be located under AIWorldCup/examples/
minuk/data   => We gathered some image, coordinates, label data here.
minuk/freeze => We put some files for safety.
minuk/save   => We saved neural networks' weights here. 

(2) Files
calculator.py     => From AIM lab's 2017 code.
data_processor.py => From AIM lab's 2017 code.
motor.py          => From AIM lab's 2017 code.
pid_ctl.py        => From AIM lab's 2017 code.
strategy.py       => Our rule-based code. This file changes according to your policy. 
main.py           => From AIM lab's 2017 code. But this file changes a lot according to what you're trying to do. 