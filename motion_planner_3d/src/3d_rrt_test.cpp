#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/geometry/bvh/BVH_model.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;


template <typename S>
class Planner {
public:
	void setStart(double x, double y, double yaw)
	{	
		ob::ScopedState<ob::CompoundStateSpace> start(space);
		// base
		ob::StateSpacePtr startSE2 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
		ob::ScopedState<ob::SE2StateSpace> startBase(startSE2);
		startBase->setXY(x,y);
		startBase->setYaw(yaw);
		start << startBase;
		// arm
		ob::StateSpacePtr startR2 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(1));		
		ob::ScopedState<ob::RealVectorStateSpace> startArm(startR2);
		startArm[0] = 0.3;
		startArm[1] = 0.5;
		start << startArm;
		pdef->clearStartStates();
		pdef->addStartState(start);
		ROS_INFO("start set to: x: %f y: %f yaw: %f h: %f l: %f", x, y, yaw, 0.3, 0.5);
	}
	void setGoal(double x, double y, double yaw)
	{
		ob::ScopedState<ob::CompoundStateSpace> goal(space);

		// base
		ob::StateSpacePtr goalSE2 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
		ob::ScopedState<ob::SE2StateSpace> goalBase(goalSE2);
		goalBase->setXY(x,y);
		goalBase->setYaw(yaw);
		goal << goalBase;
		// arm
		ob::StateSpacePtr goalR2 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(1));		
		ob::ScopedState<ob::RealVectorStateSpace> goalArm(goalR2);
		goalArm[0] = 0.3;
		goalArm[1] = 0.5;
		goal << goalArm;
		pdef->clearGoal();
		pdef->setGoalState(goal);
		ROS_INFO("goal set to: x: %f y: %f yaw: %f h: %f l: %f", x, y, yaw, 0.3, 0.5);
	}

	void updateMap(std::shared_ptr<fcl::CollisionGeometry<S>> map)
	{
		tree_obj = map;
	}
	// Constructor
	Planner(void)
	{	
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry<S>>(new fcl::Box<S>(0.1, 0.1, 0.1));
		fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry<S>>(tree);
				
		ob::StateSpacePtr baseSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
		ob::StateSpacePtr armSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

		ob::RealVectorBounds baseBounds(2);
		baseBounds.setLow(-2);
		baseBounds.setHigh(2);

		ob::RealVectorBounds armBounds(2);
		armBounds.setLow(0,0);
		armBounds.setHigh(0,0.7); // x
		armBounds.setLow(1,0);
		armBounds.setHigh(1,1.8); // y
		
		baseSpace->as<ob::SE2StateSpace>()->setBounds(baseBounds);
		armSpace->as<ob::RealVectorStateSpace>()->setBounds(armBounds);

		// compound two spaces
		ob::CompoundStateSpace *cs = new ompl::base::CompoundStateSpace();
		cs->addSubspace(baseSpace, 1);
		cs->addSubspace(armSpace, 1);

		space = ob::StateSpacePtr(cs);

		// full state
		// ob::ScopedState<ob::CompoundStateSpace> fullState(space);
		ob::ScopedState<ob::CompoundStateSpace> start(space);
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		// start start and goal
		start.random();
		goal.random();

		// solver parameters
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
		si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
		pdef->setStartAndGoalStates(start, goal);
		pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));
		ROS_INFO("OMPL Solver Initialized");

	}
	// Destructor
	~Planner()
	{
	}
	void replan(void)
	{

		std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
		if(path_smooth->getStateCount () <= 2)
			plan();
		else
		{
			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
				if(!replan_flag)
					replan_flag = !isStateValid(path_smooth->getState(idx));
				else
					break;

			}
			if(replan_flag)
				plan();
			else
				std::cout << "Replanning not required" << std::endl;
		}
		
	}
	void plan(void)
	{

	    // create a planner for the defined space
		og::RRTstar* rrt = new og::RRTstar(si);

		//设置rrt的参数range
		rrt->setRange(0.25);
		rrt->setNumSamplingAttempts(10);

		ob::PlannerPtr plan(rrt);

	    // set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

	    // perform setup steps for the planner
		plan->setup();

	    // print the settings for this space
		si->printSettings(std::cout);

		std::cout << "problem setting\n";
	    // print the problem settings
		pdef->print(std::cout);

	    // attempt to solve the problem within one second of planning time
		ob::PlannerStatus solved = plan->solve(2);

		if (solved)
		{
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
			ROS_INFO("Found solution:");
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);
	        // print the path to screen
	        // path->print(std::cout);

			nav_msgs::Path msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "map";
			traj_pub.publish(msg);
	        //Path smoothing using bspline
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			ROS_INFO("Smoothed Path");
			path_smooth->printAsMatrix(std::cout);

			//Publish path as markers
			nav_msgs::Path smooth_msg;
			smooth_msg.header.stamp = ros::Time::now();
			smooth_msg.header.frame_id = "map";

			vis_pub.publish(smooth_msg);
			ros::Duration(0.1).sleep();
			
			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
			std::cout << "No solution found" << std::endl;
	}
private:

	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	og::PathGeometric* path_smooth;

	bool replan_flag = false;
	
	std::shared_ptr<fcl::CollisionGeometry<S>> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry<S>> tree_obj;

	template<typename BV>
	void octomapCOllisionTestBVH(const ob::State *state)
	{
		using S1 = typename BV::S1;

		std::vector<fcl::Vector3<S1>> armModel;
		std::vector<fcl::Vector3<S1>> baseModel;

		fcl::BVHModel<BV>* robotModel = new fcl::BVHModel<BV>();
		robotModel->beginModel();
		robotModel->addSubModel(armModel, baseModel);
		robotModel->endModel();

		std::shared_ptr<fcl::CollisionGeometry<S>> robotModelPtr(robotModel);
		fcl::CollisionObject<S> treeObj((tree_obj));
		fcl::CollisionObject<S> robotObj(robotModel);

		fcl::DefaultCollisionData<S> cdata;
		fcl::DefaultCollisionFunction(&treeObj, &robotObj, &cdata);
		return cdata.result.numContacts>10;
	}


	bool isStateValid(const ob::State *state)
	{	

		return false;
	}


	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		// obj->setCostThreshold(ob::Cost(1.51));
		return obj;
	}

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		return obj;
	}

};

template <typename S>
void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, Planner<S>* planner_ptr)
{
    // loading octree from binary
	const std::string filename = "/HELLO_FLEET_PATH/maps/geb079.bt";
	octomap::OcTree temp_tree(0.1);
	temp_tree.readBinary(filename);

	fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	// convert octree to collision object
	// octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	// fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(tree_oct));
	
	// Update the octree used for collision checking
	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry<S>>(tree));
	//planner_ptr->replan();
}

template <typename S>
void odomCb(const nav_msgs::Odometry::ConstPtr &msg, Planner<S>* planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

template <typename S>
void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, Planner<S>* planner_ptr)
{
	planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
}

template <typename S>
void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, Planner<S>* planner_ptr)
{
	planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
	planner_ptr->plan();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	Planner<double> planner_object;
	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback<double>, _1, &planner_object));
	// ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/rovio/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
	ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/goal", 1, boost::bind(&goalCb<double>, _1, &planner_object));
	ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start", 1, boost::bind(&startCb<double>, _1, &planner_object));
	// vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	vis_pub = n.advertise<nav_msgs::Path>("visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints_marker",1);
	traj_pub = n.advertise<nav_msgs::Path>("waypoints",1);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	ros::spin();
	return 0;
}