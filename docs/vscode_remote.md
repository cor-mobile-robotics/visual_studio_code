# Remote Development Using Visual Studio Code

This tutorial shows how you can connect Visual Studio Code to a remote machine and be able to develop remotely using SSH. The use case for this implementation is diverse and powerfull. You could for instance develop code in a container on a remote server, but more importantly you could easily connect to robots when they are connected to a network. More advantages are:
 - Setting up ROS and quickly see all Topics over the network.
 - Debugging nodes remotely.
 - Interface to the folder setup.
 - Visualize all available containers and start them on the remote machine.
 - Connect to these containers, which pops-up a new window.

## Contents:
* [1. Visual Studio Code Extensions](#1-visual-studio-code-extensions)
* [2. Using SSH](#2-using-ssh)
* [3. Creating A Connection To A Remote Machine](#3-creating-a-connection-to-a-remote-machine)



## 1) Visual Studio Code Extensions

The following extensions are used in this module:
- Remote Development (An extension pack that lets you open any folder remotely or in a container)
- Remote Explorer (View remote machines for Remote - SSH and Remote Server)
- Remote - SSH: Editing Configuration Files
- Remote - SSH (Open any folder on a remote machine using SSH and take advantage of VS Code's full feature set.)

## 2) Using SSH

| :exclamation: If you already know how to work with SSH and you have a public key typically located at `~/.ssh/id_{encryption name}.pub`, you can skip this section. |
|------|

Install a supported SSH client if you do not get information how to use ssh if you type in the terminal,
```
ssh
```
then install,
```
sudo apt-get install openssh-client
```

Now we need to create a public and private key 
```
ssh-keygen -t rsa -b 4096
```
This will typically be stored in the location `~/.ssh/id_rsa.pub`.

| :warning: WARNING: Never send the private key  `~/.ssh/id_rsa` |
|---------|



## 3) Creating A Connection To A Remote Machine

### Remote machine
First we need to know the IP address of the remote machine on the local network. This is visible by checking the internet settings or by using the terminal,
```
ifconfig
```
And if you are connected through cable, the ip address can be found under `enp9s0` and with wifi under `wlp2s0` behind the word `inet`. 


### Host machine

After the extensions are installed on your machine, you should see a green icon in the lower left corner of VS Code. You need to click this icon. A command palette will open where you can select `Connect to Host...`. 

![Alt text](images/connecttohost.png?raw=true "SSH Icon")

We can now add a new SSH host by selecting `+ Add New SSH Host...`. You now need to enter the SSH command with the username of the remote machine and it's IP address. An example will be,
```
ssh username@192.100.100.10
```
Note: The above is an example and you should use your own IP address of the remote machine and it's username.

Now we select a configuration file to add the above connection to. Typically you will need to use `/home/username/.ssh/config`. If we now open the config file, using:
```
Press green icon -->> Open SSH Configuration File... -->> /home/username/.ssh/config
```
You will get something similar as the image below only with your username and IP address.

![Alt text](images/sshconfig.png?raw=true "SSH config")

Now we can change the `Host` setting name to something easier to remember.

![Alt text](images/sshconfigname.png?raw=true "SSH config")

The `Host` (In the example image: Specific Name) should now be available in the command palette,
```
Press green icon -->> Connect to Host... 
```
Select the name from the command palette and a connection will be made to the remote machine. You will need to supply the password of the remote machine to get into the machine. Visual Studio Code will download automatically SSH server on the remote machine if it is not installed so VS code can connect to it. When a connection is established, you will see the `Host` name depicted besides the green icon. You can now open any folder or terminal on the remote machine.

### Anoying password authentication

A lot of times you have to supply the password of the remote machine. Now to avoid this, we are going to paste our public key on the remote machine. Now we only have to supply the password one time and next time a new connection is made it wont ask for the password again. To do this, we give an example using the examples from above, you have to change it to your own settings,
```
ssh-copy-id -i ~/.ssh/id_rsa.pub username@192.100.100.10
```

| :warning: Note: You can always check which keys are stored on the machine by going to `~/.ssh/authorized_keys`, if there is a key that you don't know then remove it. Since this will give acces to the device without password.|
|--------|






# Checkout one of the other Modules

[1. Tips to increase development](vscode_tips.md)  
[2. Working with Robotic Operating System ](vscode_ros.md)  
[3. Remote development using SSH](vscode_remote.md)  
[4. Development inside Docker containers](vscode_docker.md) 