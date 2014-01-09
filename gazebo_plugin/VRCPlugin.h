/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_VRC_PLUGIN_HH
#define GAZEBO_VRC_PLUGIN_HH

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <atlas_msgs/AtlasSimInterfaceCommand.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class VRCPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: VRCPlugin();

    /// \brief Destructor
    public: virtual ~VRCPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to parent world.
    /// \param[in] _sdf Pointer to sdf element.
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller on every World::Update
    private: void UpdateStates();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Generic tools for manipulating models                                //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////

    /// \brief add a constraint between 2 links
    /// \param[in] _world a pointer to the current World
    /// \param[in] _model a pointer to the Model the new Joint will be under
    /// \param[in] _link1 parent link in the new Joint
    /// \param[in] _link2 child link in the new Joint
    /// \param[in] _type string specifying joint type
    /// \param[in] _anchor a Vector3 anchor offset of the new joint
    /// \param[in] _axis Vector3 containing xyz axis of the new joint
    /// \param[in] _upper upper linit of the new joint
    /// \param[in] _lower lower linit of the new joint
    /// \return Joint created between _link1 and _link2 under _model.
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint.
    /// \param[in] _joint Joint to remove.
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Vehicle properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Vehicle
    {
      /// \brief Load the drc vehicle portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr model;
      private: math::Pose initialPose;
      private: physics::LinkPtr seatLink;

      /// \brief flag for successful initialization of vehicle
      private: bool isInitialized;

      friend class VRCPlugin;
    } drcVehicle;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Multisense properties and states                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Robot
    {
      public:
        Robot();
        ~Robot();

      /// \brief Load the atlas portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void InsertModel(physics::WorldPtr _parent,
        sdf::ElementPtr _sdf);


      /// \brief Spawns a gazebo robot model from string.
      /// \param[in] _robotStr string containing model sdf or urdf.
      /// \param[in] _modelName name of newly spawned model in gazebo.
      /// \param[in] _spawnPose spawn location of model in world frame.
      /// \return pointer to the newly spawned model.
      private: bool CheckGetModel(physics::WorldPtr _world);
      private: math::Pose spawnPose;
      private: physics::ModelPtr model;
      private: physics::LinkPtr pinLink;
      private: physics::JointPtr pinJoint;

      private: std::string modelName;
      private: std::string pinLinkName;

      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
      private: math::Pose initialPose;

      /// \brief flag for successful initialization of atlas
      private: enum StartupSequence {
        NONE = 0,
        SPAWN_QUEUED = 1,
        SPAWN_SUCCESS = 2,
        INITIALIZED = 3
      };

      private: int startupSequence;


      /// \brief Pose of robot relative to vehicle.
      private: math::Pose vehicleRelPose;

      /// \brief Robot configuration when inside of vehicle.
      private: std::map<std::string, double> inVehicleConfiguration;

      friend class VRCPlugin;
    } atlas;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: bool warpRobotWithCmdVel;
    private: double lastUpdateTime;
    private: geometry_msgs::Twist robotCmdVel;

    /// \brief Pointer to parent world.
    private: physics::WorldPtr world;

    /// \brief fix multisense to vehicle for efficiency
    // public: std::pair<physics::LinkPtr, physics::LinkPtr> vehicleRobot;
    public: physics::JointPtr vehicleRobotJoint;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // default ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueueThread;

    // items below are used for deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    /// \brief Are cheats enabled?
    private: bool cheatsEnabled;
  };
/** \} */
/// @}
}
#endif
