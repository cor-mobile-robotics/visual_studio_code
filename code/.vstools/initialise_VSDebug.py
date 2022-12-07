#!/usr/bin/env python3

import json
import os
import shutil
from pathlib import Path

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# Define if the user wants to use "catkin build" or "catkin_make"
print(
    "\n Specify the catkin workspace that you want to use: \n \
    [1] catkin build \n \
    [2] catkin_make \n \
It is highly recommended to use [1] catkin build, since the tools only provide a basic setup for [2] catkin_make. \n"
)
loop = True
while loop:
    value_input = int(input("Specify a correct number:"))
    if value_input == 1:
        catkin_make = False
        loop = False
        print("\n *Catkin Build workspace specified")
    elif value_input == 2:
        catkin_make = True
        loop = False
        print("\n *Catkin_make workspace specified")

# Create src folder for packages if not existing
try:
    os.mkdir(workspace_folder + "/src")
    print("\n *Created folder src.")
except Exception:
    print("\n !Don't have to create folder src, already existing.")

# Move all the needed files to the workspace from the cloned repository and remove clone
try:
    shutil.move(
        workspace_folder + "/visual_studio_code/.clang-format", workspace_folder
    )
    print("\n *Moving .clang-format file to the workspace.")
except Exception:
    print(
        " \n !Couldn't move the .clang-format file, check if it is already in your workspace."
    )

# Specify if you want to use the example packages
loop = True
while loop:
    str_input = input("\n Do you want to use the example packages (y/n):")
    if str_input == "y":
        try:
            shutil.move(
                workspace_folder + "/code/beginner_tutorials", workspace_folder + "/src"
            )
            shutil.move(workspace_folder + "/code/hello_vs_code", workspace_folder + "/src")
            print("\n *Moved both packages to the src folder.")
        except Exception:
            print(
                "\n !Couldn't move the example packages to the src folder! check your folders."
            )
        loop = False
    elif str_input == "n":
        print("\n !Not using the test packages.")
        loop = False

# Remove the repository folder
try:
    shutil.rmtree(workspace_folder + "/visual_studio_code")
    print("\n *Removing the github repository, not needed anymore.")
except Exception:
    print(
        "\n !Couldn't remove the folder visual_studio_code! check your workspace (Already removed?)."
    )

# Create folder for VS code json files
try:
    os.mkdir(workspace_folder + "/.vscode")
except Exception:
    None

########################################################################################
## Creation of the tasks.json file
########################################################################################
if catkin_make:
    tasks_data = {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "ROS: catkin_make",
                "type": "catkin_make",
                "args": [
                    "--directory",
                    workspace_folder,
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {"kind": "build", "isDefault": True},
            },
            {
                "label": "ROS: catkin_make ~ Debug rosrun",
                "command": "python3 "
                + str(workspace_folder)
                + "/.vstools/update_VSDebug.py",
                "type": "shell",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
                "dependsOn": ["ROS: catkin_make"],
            },
            {
                "label": "ROS: catkin_make ~ Debug roslaunch",
                "command": "python3 "
                + str(workspace_folder)
                + "/.vstools/update_VSDebug_launch.py",
                "type": "shell",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
                "dependsOn": ["ROS: catkin_make"],
            },
        ],
    }
else:
    tasks_data = {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "ROS: catkin build",
                "type": "catkin",
                "args": [
                    "build",
                    "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {
                    "reveal": "always",
                    "panel": "shared",
                    "focus": True,
                    "clear": True,
                },
            },
            {
                "label": "ROS: catkin build debug",
                "type": "catkin",
                "args": [
                    "build",
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {
                    "reveal": "always",
                    "panel": "shared",
                    "focus": True,
                    "clear": True,
                },
            },
            {
                "label": "ROS: catkin clean",
                "type": "catkin",
                "args": ["clean"],
                "problemMatcher": "$catkin-gcc",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
            },
            {
                "label": "ROS: catkin build debug ~ Debug rosrun",
                "command": "python3 "
                + str(workspace_folder)
                + "/.vstools/update_VSDebug.py",
                "type": "shell",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
                "dependsOn": ["ROS: catkin build debug"],
            },
            {
                "label": "ROS: catkin build debug ~ Debug roslaunch",
                "command": "python3 "
                + str(workspace_folder)
                + "/.vstools/update_VSDebug_launch.py",
                "type": "shell",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
                "dependsOn": ["ROS: catkin build debug"],
            },
            {
                "label": "ROS: catkin build debug ~ Debug roslaunch from specific Pkg",
                "command": "python3 "
                + str(workspace_folder)
                + "/.vstools/update_VSDebug_launch_pkg.py",
                "type": "shell",
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "shared", "focus": True},
                "dependsOn": ["ROS: catkin build debug"],
            },
        ],
    }

# Store the data in the json file
with open(".vscode/tasks.json", "w") as jsonFile_tasks:
    json.dump(tasks_data, jsonFile_tasks, indent=4)
    jsonFile_tasks.close()

########################################################################################
## Creation of the c_cpp_properties.json file
########################################################################################
# create the full properties file
# So intellisense can find all headers from ROS, installed packages and headers in the workspace
c_cpp_properties_data = {
    "configurations": [
        {
            "name": "Ubuntu",
            "includePath": [
                "/usr/include/**",
                "/opt/ros/noetic/include/**",
                "${workspaceFolder}/**",
            ],
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/g++",
            "cStandard": "c11",
            "cppStandard": "c++17",
        }
    ],
    "version": 4,
}

# Store the data in the json file
with open(".vscode/c_cpp_properties.json", "w") as jsonFile_c_cpp_properties:
    json.dump(c_cpp_properties_data, jsonFile_c_cpp_properties, indent=4)
    jsonFile_c_cpp_properties.close()

########################################################################################
## Creation of the extensions.json file
########################################################################################
extensions_data = {
    "recommendations": [
        "ms-iot.vscode-ros",
        "ms-python.python",
        "ms-vscode.cpptools",
        "twxs.cmake",
        "ms-python.black-formatter",
        "xaver.clang-format",
        "ms-vscode.cmake-tools",
        "ms-python.isort",
    ]
}
# Store the data in the json file
with open(".vscode/extensions.json", "w") as jsonFile_extensions:
    json.dump(extensions_data, jsonFile_extensions, indent=4)
    jsonFile_extensions.close()

########################################################################################
## Creation of the settings.json file
########################################################################################
settings_data = {
    "python.autoComplete.extraPaths": ["/opt/ros/noetic/lib/python3/dist-packages"],
    "[python]": {
        "editor.defaultFormatter": "ms-python.black-formatter",
        "editor.formatOnSave": True,
    },
    "cmake.sourceDirectory": "${workspaceFolder}/src",
    "cmake.configureOnOpen": False,
    "terminal.integrated.scrollback": 1000000,
    "editor.codeActionsOnSave": {"source.fixAll": True, "source.organizeImports": True},
    "editor.formatOnSave": True,
    "clang-format.executable": "/usr/bin/clang-format-12",
    "clang-format.style": "file",
    "clang-format.language.c.enable": True,
    "[c]": {"editor.defaultFormatter": "xaver.clang-format"},
    "python.analysis.extraPaths": ["/opt/ros/noetic/lib/python3/dist-packages"],
}
# Store the data in the json file
with open(".vscode/settings.json", "w") as jsonFile_settings:
    json.dump(settings_data, jsonFile_settings, indent=4)
    jsonFile_settings.close()

print("\n\n --- All done ---")
