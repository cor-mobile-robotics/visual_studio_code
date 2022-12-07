#!/usr/bin/env python3

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


# Ask which Package to use
print("The packages that are available in this workspace:")
j = 1
launch_data_array = []
for pkg in array:
    # sometimes folders below eachother are called the same for the same package.
    # like catkin_ws/src/test_pkg/test_pkg/include/test_pkg.
    # to still find the launch in these circumstances we append all the paths,
    # and see if one of the paths include launch files
    pkg_paths = []
    # search for the path of the package
    for path, subdirs, files in os.walk(workspace_folder + "/src"):
        for name in subdirs:
            if name == pkg:
                pkg_paths.append(os.path.join(path, name))

    # print name of the pkg
    print("***" + pkg + " launch file added:")

    # search for launch files in that package and store json data
    launch_exists = False
    for pkg_path in pkg_paths:
        for path, subdirs, files in os.walk(pkg_path):
            for name in files:
                if name.endswith(".launch"):
                    # check package name to name launch configuration
                    launch_data_array.append(
                        {
                            "name": "(" + pkg + ")  " + name + " ",
                            "type": "ros",
                            "request": "launch",
                            "target": "" + os.path.join(path, name) + "",
                        }
                    )
                    print("    -" + name)
                    launch_exists = True

    if not launch_exists:
        print("    (no launch files exist in this package)")


if not launch_data_array:
    print("\n No launch files found in this package!! \n")

# Give the correct form to the launch file
launch_data = {}
launch_data["configurations"] = launch_data_array

# Store the data in the json file
with open(".vscode/launch.json", "w") as jsonFile_launch:
    json.dump(launch_data, jsonFile_launch, indent=4)
    jsonFile_launch.close()
