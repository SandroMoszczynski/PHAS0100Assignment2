/*=============================================================================
  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model
  Copyright (c) University College London (UCL). All rights reserved.
  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.
  See LICENSE.txt in the top level directory for details.
  VTK based viewer for Social Force Model simulation: 12 Apr 2020
      Author: Jim Dobson
=============================================================================*/

#include "sfmVisualiser.h"

#include <iostream>
#include <cmath>

#include <vtk-7.1/vtkChartXY.h>
#include <vtk-7.1/vtkContextScene.h>
#include <vtk-7.1/vtkContextView.h>
#include <vtk-7.1/vtkFloatArray.h>
#include <vtk-7.1/vtkPlotPoints.h>
#include <vtk-7.1/vtkRenderWindow.h>
#include <vtk-7.1/vtkRenderWindowInteractor.h>
#include <vtk-7.1/vtkVariantArray.h>
#include <vtk-7.1/vtkRenderer.h>
#include <vtk-7.1/vtkTable.h>
#include <vtk-7.1/vtkAxis.h>
#include <vtk-7.1/vtkBrush.h>
#include <vtk-7.1/vtkPen.h>


namespace sfm
{

//=============================================================================*/
// Visualiser definitions
//-----------------------------------------------------------------------------
Visualiser::Visualiser(unsigned int n_pedestrians, 
		       double world_x, double world_y, 
		       double window_scale):
	m_window_scale(window_scale), m_marker_scale(5.0) {

  CreateWorld(n_pedestrians, world_x, world_y);

}

//-----------------------------------------------------------------------------
Visualiser::~Visualiser(){
  // cleanup handled by smart pointers
}

//-----------------------------------------------------------------------------
void Visualiser::CreateWorld(unsigned int n_pedestrians, double world_x, double world_y){

  m_world_x = world_x;
  m_world_y = world_y;
  m_n_pedestrians = n_pedestrians;

  // Create view
  m_view = vtkSmartPointer<vtkContextView>::New();
  m_view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
  m_view->GetRenderWindow()->SetSize( m_world_x*m_window_scale+35, 
		                      m_world_y*m_window_scale+35);     

  // Add 2D chart
  m_chart = vtkSmartPointer<vtkChartXY>::New();
  m_view->GetScene()->AddItem(m_chart);

  m_chart->GetAxis(0)->SetBehavior(vtkAxis::FIXED);
  m_chart->GetAxis(1)->SetBehavior(vtkAxis::FIXED);
  m_chart->GetAxis(0)->SetRange(0.0, world_y);
  m_chart->GetAxis(1)->SetRange(0.0, world_x);

  // Initialise view in non-blocking mode
  m_view->GetRenderWindow()->SetMultiSamples(0);
  m_view->GetInteractor()->Initialize();

  // Create tables that store pedestrian data
  CreateTables();  
}

//-----------------------------------------------------------------------------
void Visualiser::CreateTables(){

  m_tables.clear();

  // To set different marker sizes and color for each pedestrian then 
  // need a vtkTable with one row for each vtkPlot
  for(int i = 0; i < m_n_pedestrians; i++){

    m_tables.push_back(vtkSmartPointer<vtkTable>::New());

    vtkSmartPointer<vtkFloatArray> arrX =
      vtkSmartPointer<vtkFloatArray>::New();
    arrX->SetName("X Axis [m]");
    m_tables[i]->AddColumn(arrX);
  
    vtkSmartPointer<vtkFloatArray> arrY =
      vtkSmartPointer<vtkFloatArray>::New();
    arrY->SetName("Y Axis [m]");
    m_tables[i]->AddColumn(arrY);
  
    vtkSmartPointer<vtkFloatArray> arrVX =
      vtkSmartPointer<vtkFloatArray>::New();
    arrVX->SetName("Velocity X [m/s]");
    m_tables[i]->AddColumn(arrVX);
   
    vtkSmartPointer<vtkFloatArray> arrVY =
      vtkSmartPointer<vtkFloatArray>::New();
    arrVY->SetName("Velocity Y [m/s]");
    m_tables[i]->AddColumn(arrVY);
  
    m_tables[i]->SetNumberOfRows(1);
  }

}

//-----------------------------------------------------------------------------
bool Visualiser::SetPedestrian(unsigned int i, 
		                  double xpos, double ypos,
				  double xspeed, double yspeed){

  if(i < m_n_pedestrians)
  {
    if(xpos >= 0.0 && xpos < m_world_x &&
       ypos >= 0.0 && ypos < m_world_y)
    {
      m_tables[i]->SetValue(0, 0, xpos);
      m_tables[i]->SetValue(0, 1, ypos);
      m_tables[i]->SetValue(0, 2, xspeed);
      m_tables[i]->SetValue(0, 3, yspeed);

      return true;
    }
    else
    {
      std::cerr << "Pedestrian position x = "<< xpos 
	        << ", y = "<< ypos << " as outside of world\n";
    }
  }
  else
  {
    std::cerr << "Unable to update pedestrian "<< i+1 
	      << "as there are only " << m_n_pedestrians 
	      << " pedestrians in world\n";
  }

  return false;
}

//-----------------------------------------------------------------------------
void Visualiser::UpdateScene(){

  // Brute force just replace all plots
  m_chart->ClearPlots();
  m_points.clear();

  float VX, VY, speed;
  for(int i = 0; i < m_n_pedestrians; i++){

    // vtkPlot for each pedestrian to control individual marker properties
    m_points.push_back(m_chart->AddPlot(vtkChart::POINTS));
    m_points[i]->SetInputData(m_tables[i], 0, 1); 

    // Set marker volume proportional to speed
    VX = m_tables[i]->GetRow(0)->GetValue(2).ToFloat();
    VY = m_tables[i]->GetRow(0)->GetValue(3).ToFloat();
    speed = std::pow(std::pow(VX, 2.0) + std::pow(VY, 2.0), 0.5);
    m_points[i]->SetWidth(m_marker_scale*std::pow(speed, 0.5));

    if(VX >= 0) // blue if moving to the right
    {
      m_points[i]->GetPen()->SetColor(0,0,255);
    }
    else // black if moving to the left
    {
      m_points[i]->GetPen()->SetColor(0,0,0);
    }	 

    // Hollow markers not possible as no distinction made between marker 
    // fill area and border: vtkPlot::GetBrush()->SetOpacityF(0.0) does nothing 
    // and vtkPlot::GetPen()->SetOpacityF(0.0) makes the whole marker invisible.
    // Instead set opacity to 50% for all:
    m_points[i]->GetPen()->SetOpacityF(0.5);

    m_points[i]->Modified();
  }

  m_view->GetRenderWindow()->Render();

}

//-----------------------------------------------------------------------------
void Visualiser::SetWindowSize(double scale){
  m_window_scale = scale;
}

//-----------------------------------------------------------------------------
void Visualiser::SetMarkerSize(double scale){
  m_marker_scale = scale;
}


} // end namespace