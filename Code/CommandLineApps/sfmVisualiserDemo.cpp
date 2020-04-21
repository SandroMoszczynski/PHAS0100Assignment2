#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <random>
#include "sfmVisualiser.h"

int main(int argc, char** argv){

  // Needed these to define viewer world	
  double world_width_x = 50.0;
  double world_height_y = 10.0;
  const unsigned int n_pedestrians = 10;

  // Create viewer and initialise with required number of pedestrians
  sfm::Visualiser viewer(n_pedestrians, world_width_x, world_height_y);

  // Or create with non-default window size, set to 20 pixels/meter
//  sfm::Visualiser viewer(n_pedestrians, world_width_x, world_height_y, 20.0);

  // Example if you want to change the base marker width (in pixels) 
//  viewer.SetMarkerSize(8.0);

  // Minimal structure to store dummy x,y,vx,vy just for the purposes of 
  // the visualiser demo 
  std::array<std::array<double, 4>, n_pedestrians> pedestrians;
 
  std::default_random_engine generator;
  std::uniform_real_distribution<double> ranx(0.0, world_width_x);
  std::uniform_real_distribution<double> rany(0.0, world_height_y);
  std::uniform_real_distribution<double> ranvx(-0.2, 0.2);
  std::uniform_real_distribution<double> ranvy(-1.3, 1.3);

  // Dummy loop over time
  int n_steps = 1000;
  for(int i = 0; i < n_steps; i++){ 

    // Simulate some pedestrian movement 
    for(auto& pedestrian : pedestrians){
      pedestrian[0] = ranx(generator);
      pedestrian[1] = rany(generator);
      pedestrian[2] = ranvx(generator);
      pedestrian[3] = ranvy(generator);
    }

    // Send updated pedestrian positions to viewer 
    for(unsigned int i = 0; i < n_pedestrians; i++){
      viewer.SetPedestrian(i, pedestrians[i][0],  pedestrians[i][1], 
		              pedestrians[i][2],  pedestrians[i][3]);
    }

    // Tell viewer to redraw scene
    viewer.UpdateScene();

    // Sleep for a bit so can see visualiser updating 
    std::this_thread::sleep_for (std::chrono::milliseconds(200));

  } 

  return 0;
}
