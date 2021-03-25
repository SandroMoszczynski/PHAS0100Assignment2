Pedestrian Simulator
-------------------

[![Build Status](https://travis-ci.com/MattClarkson/PHAS0100Assignment2.svg?branch=master)](https://travis-ci.com/sukrire/PHAS0100Assignment2)
[![Build Status](https://ci.appveyor.com/api/projects/status/5pm89ej732c1ekf0/branch/master)](https://ci.appveyor.com/project/MattClarkson/cmakecatch2)


Photos and Benchmarks
---------------------

All questions that asked for photographic evidence can be found in the proof of results folder. There are also some benchmarks there. 


Usage Instructions
------------------

This project requires an install of VTK-7.1.1, instructions on this are detailed below in build and install instructions. In this section I will go through the various methods and classes in this project. 


Class Calls:

Pedestrians:-
#include "sfmPedestrian.h"

Initialise with; 
 - sfm::Pedestrian <Pedestrian>(pos2d origin,pos2d destination, dir2d velocity,pos2d current_position,double speed,double rest_time);

Can return each value with;
 - sfm::<unittype> <unit_name> = <unit_name>.Return_<Quantity>()

Can update velocity and current position with;
 - sfm::dir2d <unit_name> = value
 - <unit_name>.Update_Velocity(<unitname>)
 - sfm::pos2d <unit_name> = value
 - <unit_name>.Update_Current_Position(<unitname>)


Forces:-
#include "sfmForces.h"

Initialise with;
 - sfm::Forces <Pedname>(pos2d origin,pos2d destination, dir2d velocity,pos2d current_position,double speed,double rest_time);
 
This will automatically have all the methods of the Pedestrian class, and will have a few new functions;

 - sfm::dir2d <Empty_value> = <Pedname>.desired_direction(<Empty_value>);
 - sfm::dir2d <Empty_value> = <Pedname>.attractive_force(<Empty_value>);

For this you will need to create a pointer to a Force by;
 - std::vector<std::shared_ptr<sfm::Forces> > <pointer_vector_name>;
 - std::shared_ptr<sfm::Forces> <pointer>(new sfm::Forces(<Pedname>));
 - <pointer_vector_name>.emplace_back(<pointer>);

This can then be input into the following Forces;
 - sfm::double <Empty_value> = <Pedname>.elipse(<pointer_vector_name>[index], <Empty_value>, double delta_t = 2); =
 - sfm::dir2d <Empty_value> = <Pedname>.repulsive_force(<pointer_vector_name>[index], <Empty_value>, double delta_t = 2);

This one takes the input of the first two functions in Forces aswell;
 - sfm::double <Empty_value> = <Pedname>.fov(<ata_force>,<des_dir>, <Empty_value>, double phi = 3.49066,double c = 0.5);    
 - sfm::dir2d <Empty_value> = <Pedname>.border_repulsive(<Empty_value>);

This last one takes all the pedestrians made through the forces pointer;
 - sfm::dir2d <Empty_value> = <Pedname>.Resultant_force(<pointer_vector_name>, <Empty_value>, double delta_t = 2);


Factory:-
This class requires no initialisations as all the subsets of this are static methods. This class creates pesdestrians of a specified type.

#include "sfmPedestrianSpawner.h"

There are three classes that create a random type of sfm vector and a random valued double. They are;

 - <dir2d> = sfm::Factory::R_Dir2d(<dir2d>);
 - <dir2d> = sfm::Factory::R_Pos2d(<dir2d>);
 - <double> = sfm::Factory::R_Doub(<double>);

These each take a value of an input of their chosen type (dir2d,pos2d,double respectively) and inputs of the x_min,x_max,y_min,y_max. To see the exact placement refer to the header or cpp files.

Next are the Spawner class and its "simplifications". Each of the subclasses can be created using inputs directly to the spawner, but they help simplify some variables that might not be needed. The Spawner class requires an empty vector of pointers as seen in the Forces section and the number of pedestrians to create as default. There are a lot of default values, please check the header or cpp file for details;

 - <pointer_vector_name> = sfm::Factory::Spawner(<pointer_vector_name>,<no_pedestrians>,...);

The next classes take a few more inputs ontop of the same pointer vector and number of pedestrians. The first is the Targeted creator, which asks for a destination to point the pedestrians to, this can be a box of a range of values or a single point. The single point can be created by setting the same values for x or y min and max. There are also default values for where to start the pedestrians and what speed and rest time they should have;

 - <pointer_vector_name> = sfm::Factory::Targeted(  <pointer_vector_name>,
                                                    <no_pedestrians>,
                                                    <x_min_dest,x_max_dest>,
                                                    <y_min_dest,y_max_dest>,
                                                    ...);

The next one is the Directional spawner. This has the same default inputs as above except it only takes a single vectory for the direction of the pedestrian. These created pedestrians will move in that vector direction, unless acted on by other pedestrians or the borders. The mechanics of how this works is details in the cpp file;

 - <pointer_vector_name> = sfm::Factory::Targeted(  <pointer_vector_name>,
                                                    <no_pedestrians>,
                                                    <x_direction,y_direction>,
                                                    ...);

The next two types will be detailed together, they are the Uniform or Distributed spawners. The uniform method creates a pedestrian randomly somewhere in the box, the exact dimensions are mentioned in the .h file. The Distributed lets you set where to spawn them;

 - <pointer_vector_name> = sfm::Factory::Uniform(   <pointer_vector_name>,
                                                    <Pedestrian_Type>,
                                                    <no_pedestrians>,
                                                    <x_min_dest,x_max_dest>,
                                                    <y_min_dest,y_max_dest>)
 - <pointer_vector_name> = sfm::Factory::Distributed(   <pointer_vector_name>,
                                                        <Pedestrian_Type>,
                                                        <no_pedestrians>,
                                                        <x_min_dest,x_max_dest>,
                                                        <y_min_dest,y_max_dest>,
                                                        <x_min_start,x_max_start>,
                                                        <y_min_start,y_min_start>)

The Pedestrian_Type here is a string of either "Targeted" or "Directional". It uses this to choose which type of pedestrians to spawn. If one chooses to use the directional type then the second destiantion (y_miny_min_dest,y_max_dest) isnt used, but a value still needs to be input. 
For the Distributed method, a second set of parameters are needed, namely the area that the pedestrian should be spawned.

                                                
Visualiser
----------
This currently acts as a demo of three sets of spawners, Run the sfmVisualiser app. When this starts up you have a choice of entering 1 2 or 3;

 - 1. This Creates 2 sets of 50 directional pedestrians at either end of the corridor along a range 1-2 and 48-49. It then sets their velocities to be along the x axis towards each other. I have kept the value of the border repulsive force quite high as this simulation would have left all the pedestrians just overlapping the y border, which doesnt look realistic. The x borders are unbound as per request. 
 - 2. This also creats 2 sets of 50 targeted pedestrians. These are created in the same place as the above ones but instead have a destination set to a random point in the opposite spawn box.
 - 3. This is a set of 100 completely random pedestrians. set anywhere within the box. 

The seed for each is randomised every time, so there wont be the exact same results each time. Note that the box spawning before the input is requested is a choice, so that you can reposition the box to where you like it each time. This was important when taking screenshots. 


Benchmarking
------------

For this run the sfmOpen_MP.x file. This will then request if default is requested, if "Yes", it will then run with the specified parameters. Note this is quite high, and will take about 5-10 minutes to run on a good computer. If default is not chosen, then there will be requested inputs for the number of pedestrians and the legnth of the run in seconds/0.1s. 
This takes the spawner of the first demo of the visualiser and changes the number of points to the ammount requested. This demo was chosen as the pedestrians are moving constantly, so it should yield better results. 
In either case a file will be created in the directory the file was run in that will be named after the parameters, and have a small preamble followed by the data in csv format. 


Purpose
-------

Extended version of CMakeCatch2 for PHAS0100 Assignment 2 that includes vec2d, pos2d and dir2d classes with wrapping functionality for periodic boundary conditions. As with CMakeCatch2 this can be used a starting point for a reasonable folder structure for [CMake](https://cmake.org/) based projects,
that use [CTest](https://cmake.org/) to run unit tests via [Catch](https://github.com/catchorg/Catch2).

Now includes a Visualisation Toolkit (VTK) based visualiser class `Code/Lib/sfmVisualiser` that can be used to plot pedestrian positions and velocities as they are simulated. This requires VTK 7 to be installed, see below. An example of its use in application given in `Code/CommandLineApps/sfmVisualiserDemo.cpp`. 

Credits
-------

This project was developed as a teaching aid for UCL's ["Research Computing with C++"](http://rits.github-pages.ucl.ac.uk/research-computing-with-cpp/)
course developed by [Dr. James Hetherington](http://www.ucl.ac.uk/research-it-services/people/james)
and [Dr. Matt Clarkson](https://iris.ucl.ac.uk/iris/browse/profile?upi=MJCLA42) and then extended by Dr. Jim Dobson based on code by Dr Tim Spain.

Build Instructions
------------------

If you want to use the VTK visualiser you need the VTK development libraries. On Ubuntu these can be installed with:
``` shell
apt-get install libvtk7-dev
```

This project itself can be built if you just want to test it. In Linux terms that
would be:
``` cmake
git clone https://github.com/MattClarkson/PHAS0100Assignment2
mkdir PHAS0100Assignment2-Build
cd PHAS0100Assignment2-Build
cmake ../PHAS0100Assignment2
make
```
You can either use this project with the current naming convention or you can use it as a 
template to create your own project with a different naming convention. To do so,
please refer to the [CMakeTemplateRenamer](https://github.com/MattClarkson/CMakeTemplateRenamer)
which will show you how to clone this repository, and rename all the variables to names of your choice.
Then you would simply build your new project, using cmake, as shown above.
