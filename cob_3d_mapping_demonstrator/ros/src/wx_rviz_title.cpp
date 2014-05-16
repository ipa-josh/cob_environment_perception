/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_3d_environment_perception_intern
 * ROS package name: cob_3d_mapping_demonstrator
 * Description: Feature Map for storing and handling geometric features
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 04/2012
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "cob_3d_mapping_demonstrator/wx_rviz_title.h"
#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"

using namespace std;


namespace rviz
{

  RvizTitle::~RvizTitle() {

  }

  /**
 Constructor
   */
  RvizTitle::RvizTitle(const std::string& name, VisualizationManager* manager/*wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi */)
  : Display( "title", manager ),
    frame_(0)
  //: wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
  //, m_wmi( wmi )
  {
    // Create controls
    //m_button = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"));
    wxWindow* parent = 0;

    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    if (wm)
    {
      parent = wm->getParentWindow();
    }
    else
    {
      frame_ = new wxFrame(0, wxID_ANY, wxString::FromAscii(""), wxDefaultPosition, wxDefaultSize, wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxRESIZE_BORDER | wxCAPTION | wxCLIP_CHILDREN);
      parent = frame_;
    }

    panel_ = new wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, wxString::FromAscii(""));
    title_ = new wxStaticText(panel_, wxID_ANY, wxString::FromAscii("3-D Umgebungserfassung"), wxDefaultPosition, wxDefaultSize);
    title_->SetFont(wxFont(48, wxSWISS, wxNORMAL, wxBOLD));
    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL);
    vsizer->Add(title_, 1, wxALIGN_CENTER);

    vsizer->SetSizeHints(panel_);
    panel_->SetSizerAndFit(vsizer);

    if (wm)
    {
      wm->addPane(name, panel_);
    }
  }

  void RvizTitle::onEnable()
  {
    if (frame_)
    {
      frame_->Show(true);
    }
    else
    {
      WindowManagerInterface* wm = vis_manager_->getWindowManager();
      wm->showPane(panel_);
    }
  }

  void RvizTitle::onDisable()
  {
    if (frame_)
    {
      frame_->Show(false);
    }
    else
    {
      WindowManagerInterface* wm = vis_manager_->getWindowManager();
      wm->closePane(panel_);
    }
  }

}
///////////////////////////////////////////////////////////////////////////////

