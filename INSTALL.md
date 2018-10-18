# Gazebo setup instructions 

## Linux 

Gazebo can be most easily ran on Linux (Debian/Ubuntu is the preferred distribution).
The plugin used was compiled on Ubuntu 18.04 LTS. If you have difficulty running the plugin,
please let us know or use the Docker container.

 - Please follow the setup instructions on the Gazebo website
    - http://gazebosim.org/tutorials?tut=install_ubuntu 
 - Clone this repository 
 - Change to the `world` directory and run gazebo 
 ```
    cd /path/to/this/dir/world
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:`pwd`
    gazebo --verbose interview_city.world
``` 
 - Optionally compile and run the sample client in the `build` directory
```
    mkdir /path/to/this/dir/build
    cd build
    cmake ..
    make 
    ./client_controller
```

## Windows/Mac/other Linux distributions

Gazebo server can be ran on any platform supporting Docker. In this case you will not see the visualisation
provided by Gazebo's client interface, but the simulation itself is fully functional.

 - Install Docker on your operating system
    - https://www.docker.com/get-started
 - Change to the directory of this repository
 - Build the container
    - `docker build -t gazebo_assignment_app .`
 - Run the container
    - `docker run -it --name gazebo_assignment_container -p 11345:11345 -v "/PATH/TO/A/LOCAL/DIRECTORY:/root/.gazebo/" gazebo_assignment_app`
    - It is recommended to mount a local directory into the container to store temporary GAzebo files, as this way
    the model files used by the simulation are only downloaded once. Change `/PATH/TO/A/LOCAL/DIRECTORY` to a 
    valid path on your file system in the command above.
    - Starting the simulator for the first time takes a few minutes, as it is downloading model files. Please be patient.
 - You can now connect to the simulator on localhost:11345
 - Optionally compile and run the sample client (see the code above)
 - Optionally install Gazebo client on your local machine and run `gzclient` to watch the simulation visualisation
 
Please check out the Gazebo Docker tutorial for more information on using this setup: https://hub.docker.com/_/gazebo/ 
