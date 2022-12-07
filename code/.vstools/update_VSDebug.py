#!/usr/bin/env python3

import glob
import json
import os
import subprocess
from pathlib import Path

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# Retrieve all package names
process = subprocess.Popen(
    ["catkin", "list", "--quiet"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
)

# retrieve data from catkin list as a string array
stdout, stderr = process.communicate()
string = str(stdout)
string = string.replace("\\n-", "")
string = string.replace("\\n'", "")
array = string.split()
array = array[1:]
array.sort()

# check if there even exist one package
if not array:
    print("**No packages found!")
    exit()


###################################################################
# Creation of the launch.json file
###################################################################
launch_data_array = []
for pkg in array:
    # Find all executables inside package
    executable_names = [
        f
        for f in os.listdir(workspace_folder + "/devel/lib/" + pkg)
        if os.path.isfile(os.path.join(workspace_folder + "/devel/lib/" + pkg, f))
    ]

    # Remove name cmake.lock
    if "cmake.lock" in executable_names:
        executable_names.remove("cmake.lock")

    # CPP and Python file name in their own array
    python_executable_names = [
        name for name in executable_names if name.endswith(".py")
    ]
    cpp_executable_names = [
        name for name in executable_names if not name.endswith(".py")
    ]

    # Add the CPP files to the launch file
    for j in cpp_executable_names:
        launch_data_array.append(
            {
                "name": "(" + pkg + ")  " + str(j) + " Node ",
                "type": "cppdbg",
                "request": "launch",
                "program": "${workspaceFolder}/devel/lib/" + pkg + "/" + str(j),
                "args": [],
                "stopAtEntry": False,
                "cwd": "${workspaceFolder}/../../",
                "environment": [],
                "externalConsole": False,
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": True,
                    }
                ],
            }
        )

    # Find the python files and also add them to the launch file
    for j in python_executable_names:
        path_package = "./src/" + pkg
        path_file = glob.glob(path_package + "/**/" + str(j), recursive=True)

        path = path_file[0]

        launch_data_array.append(
            {
                "name": "(" + pkg + ")  " + str(j) + " Node ",
                "type": "python",
                "request": "launch",
                "program": "${workspaceFolder}" + path[1:],
            }
        )

# Give the correct form to the launch file
launch_data = {}
launch_data["configurations"] = launch_data_array

# Store the data in the json file
with open(".vscode/launch.json", "w") as jsonFile_launch:
    json.dump(launch_data, jsonFile_launch, indent=4)
    jsonFile_launch.close()
