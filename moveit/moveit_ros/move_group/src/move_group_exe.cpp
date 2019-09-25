/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/move_group/move_group_exe.h>

move_group::MoveGroupExe::MoveGroupExe(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug) : node_handle_("~")
{
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution;
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);

    context_.reset(new MoveGroupContext(psm, allow_trajectory_execution, debug));

    // start the capabilities
    configureCapabilities();
}

move_group::MoveGroupExe::~MoveGroupExe()
{
    capabilities_.clear();
    context_.reset();
    capability_plugin_loader_.reset();
}

void move_group::MoveGroupExe::status()
{
    if (context_)
    {
        if (context_->status())
        {
        if (capabilities_.empty())
            printf(MOVEIT_CONSOLE_COLOR_BLUE "\nmove_group is running but no capabilities are "
                                            "loaded.\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        else
            printf(MOVEIT_CONSOLE_COLOR_GREEN "\nYou can start planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        fflush(stdout);
        }
    }
    else
        ROS_ERROR("No MoveGroup context created. Nothing will work.");
}


void move_group::MoveGroupExe::configureCapabilities()
{
    try
    {
        capability_plugin_loader_.reset(
            new pluginlib::ClassLoader<MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities: " << ex.what());
        return;
    }

    std::set<std::string> capabilities;

    // add default capabilities
    for (size_t i = 0; i < sizeof(DEFAULT_CAPABILITIES) / sizeof(DEFAULT_CAPABILITIES[0]); ++i)
        capabilities.insert(DEFAULT_CAPABILITIES[i]);

    // add capabilities listed in ROS parameter
    std::string capability_plugins;
    if (node_handle_.getParam("capabilities", capability_plugins))
    {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
        capabilities.insert(tok.begin(), tok.end());
    }

    // drop capabilities that have been explicitly disabled
    if (node_handle_.getParam("disable_capabilities", capability_plugins))
    {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
        for (boost::tokenizer<boost::char_separator<char> >::iterator cap_name = tok.begin(); cap_name != tok.end();
            ++cap_name)
        capabilities.erase(*cap_name);
    }

    for (std::set<std::string>::iterator plugin = capabilities.cbegin(); plugin != capabilities.cend(); ++plugin)
    {
        try
        {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, plugin->c_str());
        MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(*plugin);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(cap);
        }
        catch (pluginlib::PluginlibException& ex)
        {
        ROS_ERROR_STREAM("Exception while loading move_group capability '" << *plugin << "': " << ex.what());
        }
    }

    std::stringstream ss;
    ss << std::endl;
    ss << std::endl;
    ss << "********************************************************" << std::endl;
    ss << "* MoveGroup using: " << std::endl;
    for (std::size_t i = 0; i < capabilities_.size(); ++i)
        ss << "*     - " << capabilities_[i]->getName() << std::endl;
    ss << "********************************************************" << std::endl;
    ROS_INFO_STREAM(ss.str());
}