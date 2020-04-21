/*=============================================================================
  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model
  Copyright (c) University College London (UCL). All rights reserved.
  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.
  See LICENSE.txt in the top level directory for details.
  VTK based viewer for Social Force Model simulation: 12 Apr 2020
      Author: Jim Dobson
  See Code/CommandLineApps/sfmVisualiserDemo.cpp for example usage.
=============================================================================*/

#ifndef sfmVisualiser_h
#define sfmVisualiser_h

#include <vector>

#include "vtk-7.1/vtkSmartPointer.h"
#include "sfmBasicTypes.h"

class vtkContextView;
class vtkChartXY;
class vtkTable;
class vtkPlot;

//! Single namespace for all code in this package
namespace sfm {

/**
* \class Visualiser 
* \brief VTK based visualiser class 
* \ingroup utilities
*/
class Visualiser {

  public:
    Visualiser(unsigned int n_pedestrians = 0,
                  double world_x = POS2D_XWRAP, double world_y = POS2D_YWRAP, 
		  double window_scale = 15.0);
    ~Visualiser();
    void CreateWorld(unsigned int n_pedestrians, 
		    double width_x, double width_y);
    bool SetPedestrian(unsigned int i,  
		       double xpos, double ypos,
		       double xspeed, double yspeed);
    void UpdateScene();
    void SetWindowSize(double scale); // Adjust window size (pixels/meter), requires CreateWorld to take effect
    void SetMarkerSize(double scale); // Adjust base marker size in pixels 

  private:
    void CreateTables();
    vtkSmartPointer<vtkContextView> m_view;
    vtkSmartPointer<vtkChartXY> m_chart;
    std::vector<vtkSmartPointer<vtkTable> > m_tables;
    std::vector<vtkSmartPointer<vtkPlot> > m_points;
    int m_n_pedestrians;
    double m_world_x, m_world_y;
    double m_window_scale; // Scale size of window, arbitrary units
    double m_marker_scale; // Scale marker size, arbitrary units

}; // end class Visualiser

} // end namespace

#endif