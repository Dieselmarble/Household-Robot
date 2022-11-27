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

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
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


namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

template <typename S>
class Planner {
public:
	void setStart(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> start(space);
		start->setXYZ(x,y,z);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearStartStates();
		pdef->addStartState(start);
	}
	void setGoal(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> goal(space);
		goal->setXYZ(x,y,z);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearGoal();
		pdef->setGoalState(goal);
		std::cout << "goal set to: " << x << " " << y << " " << z << std::endl;
	}

	void updateMap(std::shared_ptr<fcl::CollisionGeometry<S>> map)
	{
		tree_obj = map;
	}
	// Constructor
	Planner(void)
	{
		//四旋翼的障碍物几何形状
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry<S>>(new fcl::Box<S>(0.1, 0.1, 0.1));
		//分辨率参数设置
		fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry<S>>(tree);
		
		//解的状态空间
		space = ob::StateSpacePtr(new ob::SE3StateSpace());

		// create a start state
		ob::ScopedState<ob::SE3StateSpace> start(space);
		
		// create a goal state
		ob::ScopedState<ob::SE3StateSpace> goal(space);

		// set the bounds for the R^3 part of SE(3)
		// 搜索的三维范围设置
		ob::RealVectorBounds bounds(3);

		bounds.setLow(0,-5);
		bounds.setHigh(0,5); // x
		bounds.setLow(1,-5);
		bounds.setHigh(1,5); // y
		bounds.setLow(2,0);
		bounds.setHigh(2,5); // z

		space->as<ob::SE3StateSpace>()->setBounds(bounds);

		// construct an instance of  space information from this state space
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

		start->setXYZ(0,0,0);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		//start.random();

		goal->setXYZ(0,0,0);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// goal.random();

	    // set state validity checking for this space
		si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

	    // set Optimizattion objective
		pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

		std::cout << "Initialized: " << std::endl;
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
		og::InformedRRTstar* rrt = new og::InformedRRTstar(si);

		//设置rrt的参数range
		rrt->setRange(0.5);

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
		ob::PlannerStatus solved = plan->solve(1);

		if (solved)
		{
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);
	        // print the path to screen
	        path->print(std::cout);


			nav_msgs::Path msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "map";

			for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
			{
				const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				geometry_msgs::PoseStamped pose;

				pose.header.frame_id = "/world";

				pose.pose.position.x = pos->values[0];
				pose.pose.position.y = pos->values[1];
				pose.pose.position.z = pos->values[2];

				pose.pose.orientation.x = rot->x;
				pose.pose.orientation.y = rot->y;
				pose.pose.orientation.z = rot->z;
				pose.pose.orientation.w = rot->w;

				msg.poses.push_back(pose);

			}
			traj_pub.publish(msg);

	        //Path smoothing using bspline
			//B样条曲线优化
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);

			//Publish path as markers
			nav_msgs::Path smooth_msg;
			smooth_msg.header.stamp = ros::Time::now();
			smooth_msg.header.frame_id = "map";

			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
	            // cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
				
				geometry_msgs::PoseStamped point;

				point.header.frame_id = "/world";

				point.pose.position.x = pos->values[0];
				point.pose.position.y = pos->values[1];
				point.pose.position.z = pos->values[2];

				point.pose.orientation.x = rot->x;
				point.pose.orientation.y = rot->y;
				point.pose.orientation.z = rot->z;
				point.pose.orientation.w = rot->w;

				smooth_msg.poses.push_back(point);

				std::cout << "Published marker: " << idx << std::endl;
			}

			vis_pub.publish(smooth_msg);
			// ros::Duration(0.1).sleep();
			
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

	bool isStateValid(const ob::State *state)
	{
	    // cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	    // extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	    // extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		fcl::CollisionObject<S> treeObj((tree_obj));
		fcl::CollisionObject<S> aircraftObject(Quadcopter);

	    // check validity of state defined by pos & rot
		fcl::Vector3<S> translation(pos->values[0],pos->values[1],pos->values[2]);
		fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
		aircraftObject.setTransform(rotation, translation);
		fcl::CollisionRequest<S> requestType(1, false, 1, false);
		fcl::CollisionResult<S> collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
		return(!collisionResult.isCollision());
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
	// const std::string filename = "/HELLO_FLEET_PATH/maps/geb079.bt";
	// octomap::OcTree temp_tree(0.1);
	// temp_tree.readBinary(filename);

	// fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree<S>* tree = new fcl::OcTree<S>(std::shared_ptr<const octomap::OcTree>(tree_oct));
	
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

	//n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	vis_pub = n.advertise<nav_msgs::Path>("visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints_marker",1);
	traj_pub = n.advertise<nav_msgs::Path>("waypoints",1);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	ros::spin();
	return 0;
}