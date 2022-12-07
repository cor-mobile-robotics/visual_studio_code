# Automation In Visual Studio Code With ROS

This repository contains two important python scripts that automate your ROS workflow in the Visual Studio Code IDE. Below is a step by step guide on how to setup the VS environment and work with this environment. 

Note: The assumption is made that you know how ROS works and that it is installed. Also Visual Studio Code should be installed.


## Contents:
* [1. Visual Studio Code Extensions](#1-visual-studio-code-extensions)
* [2. Setup of the VS workspace](#2-setup-of-the-vs-workspace)
* [3. Setting up automated formatting using ROS standarts](#3-setting-up-automated-formatting-using-ros-standards)
* [4. Automation of the VS workspace](#4-automation-of-the-vs-workspace)
* [5. Debugging your code in VS with ROS](#5-debugging-your-code-in-vs-with-ros)
* [6. Hints to increase development speed](#6-hints-to-increase-development-speed)
* [7. Explanation of the .json files](#7-explanation-of-the-.json-files)

## 1) Visual Studio Code Extensions

The following extensions are recommended:
- C/C++ (C/C++ IntelliSense, debugging, and code browsing.)
- Python (IntelliSense (Pylance), Linting, Debugging (multi-threaded, remote), Jupyter Notebooks, code formatting, refactoring, unit tests, and more.)
- ROS (Develop Robot Operating System (ROS) with Visual Studio Code.)
- CMake (CMake langage support for Visual Studio Code)
- CMake Tools (Extended CMake support in Visual Studio Code)
- Black Formatter (Formatter extension for Visual Studio Code using black)
- Clang-Format (Use Clang-Format in Visual Studio Code)
- isort (Import organization support for python)

## 2) Setup of the VS Workspace

We will setup the workspace automatically by running one of the python scripts. The script will ask you two specific questions. The first one will specify the catkin builder and the second if you want to use the example packages. Setup the workspace by using,
```bash
 mkdir ~/catkin_ws
 cd ~/catkin_ws
 git clone https://github.com/R2CLab/visual_studio_code.git
 mv -v visual_studio_code/code/.vstools .
 python3 .vstools/initialise_VSDebug.py
```
When the script is done, you need to save the workspace. Go to File -> Save Workspace As and press save. The final structure of the ROS and VS workspace will look like:
```
~/catkin_ws/
    .catkin_tools
    .vscode/
        c_cpp_properties.json
        extensions.json
        settings.json
        tasks.json
    .vstools/
        initialise_VSDebug.py
        update_VSDebug_launch_pkg.py
        update_VSDebug_launch.py
        update_VSDebug.py
    build/
    devel/
    logs/
    src/
        beginner_tutorials/
        hello_vs_code/
    .clang-format
```

| WARNING: Make sure that the python file worked and the above files system looks identical to yours. |

Below an image of how it will look like in Visual studio code.

![Alt text](images/image_setup.png?raw=true "Initial setup")



Note: Make sure that the ROS environment is sourced in .bashrc and that ROS1 is mentioned with its distribution in the bar located at the bottom of VS code. If it is not the case, close Visual Studio Code and add the source of the ROS environment to your .bashrc and restart Visual Studio Code. ROS1 should now be visible.

## 3) Setting up Automated formatting using ROS standards

This Visual Code environment has a autoformatter for both python and c++ when files are saved to the ROS standards. An extra feature is that python imports are sorted to alphabetical order. To use the c++ formatter, some extra steps are required. First install clang-tools-12 using a terminal:
```
sudo apt-get install clang-tools-12
```
And now we need to set the main c++ formatter to Clang-Format in Visual Studio Code using the command palette,
```
press Ctrl+Shift+P -->> Format Document With... -->> Configure Default Formatter... -->> Clang-Format
```
The format for clang tools is defined in the .clang-format file and is automatically used by the settings.json file created in previous step.

Note: The formatter is recommended for a repository that maintains a certain standart between contributors. 

| WARNING: If the formatter is not used from the beginning of a repository, it could change files in a way that pull requests can become huge. Make sure that you know this beforehand. |


## 4) Automation of the VS workspace

Now everything is ready and we can start using the settings we made. From the command palette, a few tasks can be used that where previously made by the python file. To use these tasks:
```
press Ctrl+Shift+P -->> Tasks: Run Task 
```
And then specify one of the created tasks:
```
ROS: catkin build
ROS: catkin clean
ROS: catkin build debug
ROS: catkin build debug ~ Debug rosrun
ROS: catkin build debug ~ Debug roslaunch
ROS: catkin build debug ~ Debug roslaunch from specific Pkg
```

![Alt-text-1](images/tasks.png?raw=true "Tasks drop down menu") ![Alt-text-2](images/ros_build.png?raw=true "ROS build drop down menu")

or the faster way:
```
press Ctrl+Shift+B
```
which will automatically go to the section: `Tasks: Run Task`.

The last 3 tasks,
```
ROS: catkin build debug ~ Debug rosrun
ROS: catkin build debug ~ Debug roslaunch
ROS: catkin build debug ~ Debug roslaunch from specific Pkg
```
 will first run catkin with debug mode and then use one of the extra python files in the .vstools to create a launch.json file in the .vscode folder. This will display the nodes or launch files in the debug section of VS code. This file will be updated automatically when one of these three tasks is run.

For example puposes, we will run the rosrun debug on the example packages:
```
press Ctrl+Shift+B -->> ROS: catkin build debug ~ Debug rosrun
```

We can now go to the Run and Debug section (Ctrl+Shift+D) and all the ros nodes are added to the drop-down menu visualized in the image below. 

![Alt-text-1](images/dropdown_menu.png?raw=true "Debug drop down menu")

The nodes are first described by the package name in alphabetical order, followed by the node name and then if its a c++ or python file.
Lets start two nodes that talk to eachother, one publishes a counter and the other subscribes to the same counter. 

Roscore has to be started for connecting the nodes. This is done by following the commands:
```
Press Ctrl+Shift+P -->> ROS: Start
```
Now start c++: cppsub_Node - PKG:beginner_tutorials by selecting this node from the drop down menu and then press the green play button to the left. This starts the subscriber node. Now do the same for the c++: cpptalker_Node - PKG:beginner_tutorials. Now the two nodes are running and talking to eachother. The terminal of the talker is visualized but you can switch to the subscriber node terminal using the bottom-left pannel (Marked with the green circle) and press on the wanted terminal as shown below.


![Alt-text-1](images/two_nodes.png?raw=true "Two nodes running")
RED: The Debug player  
GREEN: The debug terminals, one for each node  
ORANGE: Checkmark when Roscore is running or cross if not  

To stop the nodes, the terminals can be closed or the stop button can be pressed for the node. This is visualized in the image below.

Now you know how to start the nodes and stop them. The next section shows how to debug the code

## 5) Debugging your code in VS with ROS

The next step to debugging is very easy. Let's take the two nodes from previous section and open the scripts next to eachother and in both scripts set a Breakpoint as visualized in the image below.


Now run both scripts as done in previous section and see how the code stops at the break point set in the publisher (left figure below). The debug player should be put on the talker node, as shown in the figure. Now press a few times F5 or the continue button in the debug player to see what happens. After a few times pressing, a yellow bar should appear in the subscriber file and the breakpoint is hit (see the right figure below). Now in the dropdown menu in the debug player, select the subscriber node and the corresponding variables are loaded. Again press F5 to let the code continue. 

![Alt-text-1](images/talk_breakpoint.png?raw=true "Breakpoint hit in talker") ![Alt-text-2](images/sub_breakpoint.png?raw=true "Breakpoint hit in subscriber")


   

| Important! |
| --- |
| - You will see that the subscriber doesnt receive the first or first few messages. This is due to the setup of the architecture of ROS. The publisher is already publishing messages when the connections are not yet initialized, and thus are lost. To have zero loss, set a ros rate sleeper (of around 5 seconds) after setting up the publisher note and always first start the subscriber nodes. This will make sure that all messages are received by the subscribers. |
| - Another thing, you will notice with the tutorial nodes that sometimes the breakpoint of the publisher is hit twice before the breakpoint in the subscriber is hit. This is due to the speed until the next breakpoint, the publisher was faster in hitting the next breakpoint than the subscriber was in receiving the message and hitting it's breakpoint. |

## 6) Hints to increase development speed

### Keybindings
We can assign a keybinding to start ROS: update Build & Debug by doing:
```
Press Ctrl+Shift+P -->> Preferences: Open Keyboard Shortcuts (JSON)
```
The keybindings.json file will open. The keybindings in here overwrite the default keybinding so make sure that the keybinding is not something you reguraly use. Add the following to the file,
```
[
    { 
        "key": "ctrl+F5",         
        "command": "workbench.action.tasks.runTask",
        "args": "ROS: catkin build debug"
    }
]
```

And it should look like the image below. Save the file to make the keybinding work. Now you only have to use the keybinding to build/update the ros workspace and add new nodes to the launch file for VS code debugger

![Alt-text-1](images/keybinding.png?raw=true "Keybinding for build and debug")


### Multiple windows

Instead of using one VS code window, one could use multiple VS code windows to debug more easily. This is espacially usefull with a multiple monitor setup. This needs to be done in a specific way, we need to clone the workspace. To do this follow,
```
Go to File --> Duplicate Workspace
```
Now you will have to VS code windows, both with the same VS code workspace. Now you can do the same test as presented in the previous chapter, only now you can run the publisher in one window and the subscriber in the other window. Below an image of how it looks like.

![Alt-text-1](images/two_screens.png?raw=true "Use of two VS code windows")


## 7) Explanation of the .json files

Below an explanation of each of the files.

### <ins> c_cpp_properties.json </ins>

- `"name"`         
Specifies the operating system.

- `"includePath"`       
Specifies the directories that intellisense searches for headers and c++ files.

- `"intellisenseMode"`          
Default for Linux is gcc-x64, msvc-x64 for Windows and clang-x64 for Mac.

- `"CompilerPath"`      
The path to the compiler.

- `"cStandard"`     
Specify the C standard.

- `"cppStandard"`       
Specify the c++ standard.



### <ins> extensions.json </ins>

This file specifies the recommended extensions.

### <ins> settings.json </ins>

- `"python.autoComplete.extraPaths"`           
Specifies the ros packages directory.

- `"[python]":"editor.defaultFormatter"`     
Sets the default python formatter.

- `"[python]":"editor.formatOnSave"`      
If you want the python files to be formatted automatically on save.

- `"cmake.sourceDirectory"`
Directory where all the cmake files are located. The standard directory for is "${workspaceFolder}/src".

- `"cmake.configureOnOpen"`
With ROS workspace this should be set to `false`, since cmake environment is only a subsection of ROS workspace.

- `"terminal.integrated.scrollback"`    
The number of lines in your terminal before lines are deleted.

- `"editor.codeActionsOnSave":"source.fixAll"`    
Removes imports that are not used in python on save.

- `"editor.codeActionsOnSave":"source.organizeImports"`   
Organizes the python imports on save.

- `"editor.formatOnSave"`    
Format c++ files on save.

- `"clang-format.executable"`    
Path to the clang format program.

- `"clang-format.style"`    
Specifies which style to use. In the case of this repository, a clang-format file specifies the style.

- `"clang-format.language.c.enable"`    
Enable or disable all formatting of different C styles.

- `"[c]":"editor.defaultFormatter"`    
The extension that is used for C styles formatting.   

- `"python.analysis.extraPaths"`   
Adding extra path to ROS packages.



### <ins> tasks.json </ins>

Here we can specify tasks that can be easily run using the tasks command palette (Ctrl+Shift+B). 

- `"label"`    
The command name displayed in the command palette

- `"type"`   
Specifies the tasks type. For a custom task set it to `shell`.

- `"command"`   
Specify the command for the custom task that is entered in the `shell`.

- `"args"`   
The arguments need to be specified in this field.

- `"problemMatcher"`   
For catkin specify `catkin-gcc`. The task system will know when the task is finished if you want to set `"isBackground": true`. 

- `"presentation"`   
Here we can specify different settings how the task is presented. In our case we want to `"reveal":"always"`, since we want to see what catkin is doing. Futhermore, we want the `"panel":"shared"` between different tasks that are dependend on eachother. The `"focus":true` let's the terminal get active, so we don't have to click on it to make the terminal active. Finally, `"clear":true` specifies that we want to clear the terminal before the command is run, so we only get details of the current command. 


### <ins> launch.json </ins>

This file is created when using the debug option. The file specifies the option for the Visual Studio Code Debugger. The launch configurations can be run from the dropdown menu in the debugger section.


# Checkout one of the other Modules

[1. Tips to increase development](docs/vs_code_tips.md)  
[2. Working with Robotic Operating System ](docs/vscode_ros.md)  
[3. Remote development using SSH](docs/vscode_remote.md)  
[4. Development inside Docker containers](docs/vscode_docker.md) 