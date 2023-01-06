# Visual Studio Code Tips & Tricks

In this section we will describe how you can use the ROS, Remote and Docker extensions together to make the Visual Studio Code environment even more powerfull. Here we will also describe further Tips and Tricks that are related to Visual Studio Code.

## Contents:
* [1. Powerfull Combination](#1-powerfull-combination)
* [2. Tips & Tricks](#2-tips-and-tricks)


## 1) Powerfull Combination

Instead of using the extension sections seperately, we can also combine them. For example, we have a drone that needs to be controlled from an external source (laptop) and a few containers need to be started on the drone. Through the remote extension, we can connect the Visual Studio Code on the laptop to the Drone. When the connection is set up, the docker extension can be used to start the containers that are neccesary. Now we can also attach Visual Studio Code on the laptop to the container that is running drone. This makes it really easy to develop remotely in the containers on a specific robot. Furthermore, you could also debug ROS nodes inside these containers. 


## 2) Tips And Tricks

TODO!


# Checkout one of the other Modules

[1. Working with Robotic Operating System ](vscode_ros.md)  
[2. Remote development using SSH](vscode_remote.md)  
[3. Development inside Docker containers](vscode_docker.md) 
[4. Tips to increase development](vscode_tips.md)  