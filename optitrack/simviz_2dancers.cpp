/**
 * @file simviz.cpp
 * @brief Simulation and visualization of dancing toro 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
#include <chrono>

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}
double DELAY = 2000; // simulation frequency in Hz
std::shared_ptr<Sai2Model::Sai2Model> hannah;
std::shared_ptr<Sai2Model::Sai2Model> tracy;
std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics;


#include "redis_keys.h"

using namespace Eigen;
using namespace std;


// mutex and globals
VectorXd hannah_ui_torques;
VectorXd tracy_ui_torques;

mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string hannah_name = "HRP4C0";
static const string tracy_name = "HRP4C1";
static const string camera_name = "camera_fixed";

const std::vector<std::string> background_paths = {
    "../../optitrack/assets/space.jpg",
    "../../optitrack/assets/sea.jpg",
    "../../optitrack/assets/stanford.jpg",
    "../../optitrack/assets/trees.jpg",
    "../../optitrack/assets/wood.jpg"
};

void setBackgroundImage(std::shared_ptr<Sai2Graphics::Sai2Graphics>& graphics, const std::string& imagePath) {
    chai3d::cBackground* background = new chai3d::cBackground();
    bool fileload = background->loadFromFile(imagePath);
    if (!fileload) {
        std::cerr << "Image file loading failed: " << imagePath << std::endl;
        return;
    }
    graphics->getCamera(camera_name)->m_backLayer->addChild(background);
}

chai3d::cColorf lagrangianToColor(double lagrangian, double min_lagrangian, double max_lagrangian) {
    double normalized = (lagrangian - min_lagrangian) / (max_lagrangian - min_lagrangian);
    normalized = std::max(0.0, std::min(1.0, normalized)); // Clamp to [0, 1]

    // Blue to Red gradient
    double red = normalized;
    double blue = 1.0 - normalized;
    double green = 0.0;

    return chai3d::cColorf(red, green, blue);
}


// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);



int main() {
	static const string toro_file = "./resources/model/HRP4c.urdf";
    //static const string world_file = "./resources/world/world_basic_10.urdf";
	static const string world_file = "./resources/world/world_basic_1.urdf";
    std::cout << "Loading URDF world model file: " << world_file << endl;

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load graphics scene
    graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
    //setBackgroundImage(graphics, "../../optitrack/assets/space.jpg"); // Set background to space
    
    graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 

    // load robots
    hannah = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);
	tracy = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);
    hannah->updateModel();
	tracy->updateModel();
    hannah_ui_torques = VectorXd::Zero(hannah->dof());
	tracy_ui_torques = VectorXd::Zero(tracy->dof());
	// SET the initial q -- define manual here to stand
	auto hannah_joint_task = std::make_shared<Sai2Primitives::JointTask>(hannah);
	auto tracy_joint_task = std::make_shared<Sai2Primitives::JointTask>(tracy);
	hannah_joint_task->disableInternalOtg();
	tracy_joint_task->disableInternalOtg();
	VectorXd hannah_q_desired(hannah->dof());
	VectorXd tracy_q_desired(tracy->dof());

	hannah_q_desired << 0, 0, 0, 0, 0, 0, 0, -0.1, -0.25, 0.5, -0.25, 0.1, 0, 0.1, -0.25, 0.5, -0.25, -0.1, 0, 0, -0.1, -0.2, 0.3, -1.3, 0.2, 0.7, -0.7, -0.1, 0.2, -0.3, -1.3, 0.7, 0.7, -0.7, 0, 0;
    tracy_q_desired << 0, 0, 0, 0, 0, 0, 0, -0.1, -0.25, 0.5, -0.25, 0.1, 0, 0.1, -0.25, 0.5, -0.25, -0.1, 0, 0, -0.1, -0.2, 0.3, -1.3, 0.2, 0.7, -0.7, -0.1, 0.2, -0.3, -1.3, 0.7, 0.7, -0.7, 0, 0;
    
	hannah_joint_task->setGoalPosition(hannah_q_desired);
	tracy_joint_task->setGoalPosition(tracy_q_desired);
	//graphics->addUIForceInteraction(toro_name);

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(hannah_name, hannah->q());
	sim->setJointVelocities(hannah_name, hannah->dq());
	sim->setJointPositions(hannah_name, hannah_q_desired);

	sim->setJointPositions(tracy_name, tracy->q());
	sim->setJointVelocities(tracy_name, tracy->dq());
	sim->setJointPositions(tracy_name, tracy_q_desired);
	
	for (int robot_index = 0; robot_index < 10; ++robot_index) {
		if (robot_index % 2 == 0) {  // Even index robots (Hannah)
			sim->setJointPositions("HRP4C" + std::to_string(robot_index), hannah_q_desired);
		} else {  // Odd index robots (Tracy)
			sim->setJointPositions("HRP4C" + std::to_string(robot_index), tracy_q_desired);
		}
	}



    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 

	redis_client.setEigen(HANNAH_TORO_JOINT_ANGLES_KEY, hannah->q()); 
	redis_client.setEigen(HANNAH_TORO_JOINT_VELOCITIES_KEY, hannah->dq()); 
	redis_client.setEigen(HANNAH_TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * hannah->q());

	redis_client.setEigen(TRACY_TORO_JOINT_ANGLES_KEY, tracy->q()); 
	redis_client.setEigen(TRACY_TORO_JOINT_VELOCITIES_KEY, tracy->dq()); 
	redis_client.setEigen(TRACY_TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * tracy->q());


	string link_name = "neck_link2"; // head link
	Eigen::Affine3d transform = hannah->transformInWorld(link_name); // p_base = T * p_link
	MatrixXd rot = transform.rotation();
	VectorXd pos = transform.translation();
	VectorXd vert_axis = rot.col(2); // straight up from head (pos z)
	VectorXd lookat = rot.col(0); // straight ahead of head (pos x)

	VectorXd offset(3);
	offset << -2.8, 0.0, -1.1; // x = 1.6
	pos += offset;

	redis_client.setEigen(HEAD_POS, pos);
	redis_client.setEigen(HEAD_VERT_AXIS, vert_axis);
	redis_client.setEigen(HEAD_LOOK_AT, lookat + pos);

	bool conmove = true;
	bool LAGRANGIAN_BACKGROUND_MODE = true;
	bool IMAGE_BACKGROUND_MODE = false;
	// start simulation thread
	thread sim_thread(simulation, sim);
	
	int robot_index = 0; // index to track which robot to update next
    int total_robots = 10; // total number of robots to update

    Sai2Common::LoopTimer timer(DELAY);
    timer.reinitializeTimer(1e9);

	bool changed_recently = false;
	double last_background_change_time = 0.0; 

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
		timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

		// Get the Lagrangian value from Redis

		if (LAGRANGIAN_BACKGROUND_MODE) {
		double lagrangian = stod(redis_client.get(LAGRANGIAN));
		chai3d::cColorf backgroundColor = lagrangianToColor(lagrangian, -50.0, 200.0);
		double red = backgroundColor.getR();
    	double green = backgroundColor.getG();
   		double blue = backgroundColor.getB();
		graphics->setBackgroundColor(red, green, blue);
		}
        
        // Update primary robot graphics
        //graphics->updateRobotGraphics(toro_name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
        
        // Update one robot graphics per iteration
        // std::string name = "HRP4C" + std::to_string(robot_index);
        // graphics->updateRobotGraphics(name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
        int index = (index + 1) % total_robots;
		for (int robot_index = 0; robot_index < 10; ++robot_index) {
			graphics->updateRobotGraphics("HRP4C" + std::to_string(robot_index), redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
		}

		if (index == 8) {
			changed_recently = false; // better logic for this
		}
		
		Eigen::Vector3d ra_end_effector_pos = redis_client.getEigen("1::11::pos");
		Eigen::Vector3d la_end_effector_pos = redis_client.getEigen("1::23::pos");

		// Calculate the L2 norm of the difference between the end effectors
		double l2_norm = (ra_end_effector_pos - la_end_effector_pos).norm();

		double last_background_change_time = 0.0; // Initialize the last background change time

		Eigen::Vector3d ground_threshold = redis_client.getEigen(GROUND);
    	double ground_z = ground_threshold.z();  // Extract the z component

		Eigen::Vector3d current_rleg_pos = redis_client.getEigen("1::47::pos");
		Eigen::Vector3d current_lleg_pos = redis_client.getEigen("1::50::pos");
		//std::cout << "right leg:" << current_rleg_pos.z() << std::endl;

		// // Check if both legs are off the ground relative to their initial positions
		bool isJumping = (current_rleg_pos.z() - ground_z > -0.08) && (current_lleg_pos.z() - ground_z > -0.08);
		double rleg_ground_difference = current_rleg_pos.z() - ground_z;
		std::cout << "foot " << rleg_ground_difference << std::endl;


		if (isJumping) {
			graphics->setBackgroundColor(1.0, 1.0, 1.0);  // White RGB
		}

		// if (l2_norm < CLAP_THRESHOLD) {
		// 	if (!changed_recently) {
		// 		int current_clap_count = redis_client.getInt(CLAP_KEY);
		// 		redis_client.setInt(CLAP_KEY, current_clap_count + 1);

		// 		int background_index = (current_clap_count + 1) % background_paths.size();
		// 		if (IMAGE_BACKGROUND_MODE) {
		// 			setBackgroundImage(graphics, background_paths[background_index]);
		// 		}
		// 		// Update the last background change time
		// 		last_background_change_time = time;
		// 		changed_recently = true;
		// 	}
		// }

		if (l2_norm < CLAP_THRESHOLD) {
			//graphics->setBackgroundColor(1.0, 1.0, 1.0);  // White RGB
		}

		// Retrieve the head position
		Eigen::Vector3d head_pos = redis_client.getEigen("1::5::pos");

		// Check if both hands are above the head
		bool hands_above_head = (head_pos.z() - ra_end_effector_pos.z() < HANDS_ABOVE_HEAD_THRESHOLD ||
								head_pos.z() - la_end_effector_pos.z() < HANDS_ABOVE_HEAD_THRESHOLD);
		redis_client.setBool(HANDS_ABOVE_HEAD_KEY, hands_above_head);

		if (hands_above_head) {
			//graphics->setBackgroundColor(0.0, 0.0, 0.0);  // Black RGB
		}

		double hands_above = head_pos.z() - ra_end_effector_pos.z();
		//std::cout << hands_above << std::endl;
		
		//std::cout << head_pos.z() - ra_end_effector_pos.z() << std::endl;
		// Set transparency for odd-numbered robots if hands are above the head
		//if (hands_above < 0.1) {
		// if (true) {
		// 	graphics->showTransparency(true, "toro0", 0.0);
		// 	graphics->showTransparency(true, "toro1", 0.0);
		// 	graphics->showTransparency(true, "toro2", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro3", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro4", 0.0);
		// 	graphics->showTransparency(true, "toro5", 1.0);
		// 	graphics->showTransparency(true, "toro6", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro7", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro8", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro9", 0.0); // Fully transparent
		// } 
		// else if (0.3 > hands_above > 0.1) {
		// 	graphics->showTransparency(true, "toro0", 0.0);
		// 	graphics->showTransparency(true, "toro1", 0.0);
		// 	graphics->showTransparency(true, "toro2", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro3", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro4", 1.0);
		// 	graphics->showTransparency(true, "toro4", 1.0);
		// 	graphics->showTransparency(true, "toro5", 1.0);
		// 	graphics->showTransparency(true, "toro6", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro7", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro8", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro9", 0.0); // Fully transparent
		// } 
		// else if (0.6 > hands_above > 0.3) {
		// 	graphics->showTransparency(true, "toro0", 1.0);
		// 	graphics->showTransparency(true, "toro1", 0.0);
		// 	graphics->showTransparency(true, "toro2", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro3", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro4", 1.0);
		// 	graphics->showTransparency(true, "toro5", 0.0);
		// 	graphics->showTransparency(true, "toro6", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro7", 0.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro8", 1.0); // Fully transparent
		// 	graphics->showTransparency(true, "toro9", 0.0); 
		// }
		// else {
		// 	graphics->showTransparency(true, "toro0", 1.0);
		// 	graphics->showTransparency(true, "toro1", 1.0);
		// 	graphics->showTransparency(true, "toro2", 1.0); 
		// 	graphics->showTransparency(true, "toro3", 1.0); 
		// 	graphics->showTransparency(true, "toro4", 1.0);
		// 	graphics->showTransparency(true, "toro5", 1.0);
		// 	graphics->showTransparency(true, "toro6", 1.0); 
		// 	graphics->showTransparency(true, "toro7", 1.0);
		// 	graphics->showTransparency(true, "toro8", 1.0); 
		// 	graphics->showTransparency(true, "toro9", 1.0); 
		// }
    
        {
			lock_guard<mutex> lock(mutex_update);
			
		}

		if (redis_client.getBool(GLOBAL_CAM) && conmove) {
			Eigen::Vector3d head_pos;
			head_pos << 2.0, -0.8, 6.0;
			Eigen::Vector3d head_vert_axis;
			head_vert_axis << 0.0, 0.0, 1.0;
			Eigen::Vector3d head_look_at;
			head_look_at << 0.0, 0.0, 0.0;

			// Create the rotation matrix
			Eigen::Matrix3d rotation;
			rotation.col(2) = head_vert_axis.normalized(); // z-axis
			rotation.col(0) = head_look_at.normalized(); // x-axis
			rotation.col(1) = rotation.col(2).cross(rotation.col(0)).normalized(); // y-axis

			// Create the Affine3d object
			Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();
			camera_pose.translation() = head_pos;
			camera_pose.linear() = rotation;

			// Call setCameraPose with the correct arguments
			graphics->setCameraPose(camera_name, camera_pose);
			conmove = false;
		} else if (redis_client.getBool(GLOBAL_CAM) && !conmove) {

		} else {
			// Retrieve the head position, vertical axis, and look-at direction
			Eigen::Vector3d head_pos = redis_client.getEigen(HEAD_POS);
			Eigen::Vector3d head_vert_axis = redis_client.getEigen(HEAD_VERT_AXIS);
			Eigen::Vector3d head_look_at = redis_client.getEigen(HEAD_LOOK_AT);

			// Create the rotation matrix
			Eigen::Matrix3d rotation;
			rotation.col(2) = head_vert_axis.normalized(); // z-axis
			rotation.col(0) = head_look_at.normalized(); // x-axis
			rotation.col(1) = rotation.col(2).cross(rotation.col(0)).normalized(); // y-axis

			// Create the Affine3d object
			Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();
			camera_pose.translation() = head_pos;
			camera_pose.linear() = rotation;

			// Call setCameraPose with the correct arguments
			graphics->setCameraPose(camera_name, camera_pose);
			conmove = true;
		}

		
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			
			hannah_ui_torques = graphics->getUITorques(hannah_name);
			tracy_ui_torques = graphics->getUITorques(tracy_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
    // fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // create a timer
    double sim_freq = 2000;
    Sai2Common::LoopTimer timer(sim_freq);

    sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

    sim->enableJointLimits(hannah_name);
	sim->enableJointLimits(tracy_name);

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        VectorXd hannah_control_torques = redis_client.getEigen(HANNAH_TORO_JOINT_TORQUES_COMMANDED_KEY);
		VectorXd tracy_control_torques = redis_client.getEigen(TRACY_TORO_JOINT_TORQUES_COMMANDED_KEY);
        {
            lock_guard<mutex> lock(mutex_torques);
            sim->setJointTorques(hannah_name, hannah_control_torques + hannah_ui_torques);
			sim->setJointTorques(tracy_name, tracy_control_torques + tracy_ui_torques);
        }
        sim->integrate();

		VectorXd hannah_robot_q = sim->getJointPositions(hannah_name);
    	VectorXd hannah_robot_dq = sim->getJointVelocities(hannah_name);
		VectorXd tracy_robot_q = sim->getJointPositions(tracy_name);
    	VectorXd tracy_robot_dq = sim->getJointVelocities(tracy_name);

		VectorXd robot_q = (hannah_robot_q + tracy_robot_q)/2;
		VectorXd robot_dq = (hannah_robot_dq + tracy_robot_dq)/2;

        //robot_dq = 0.1 * VectorXd::Ones(robot_dq.size()) * sin(time);

        // Get the mass matrix
        // MatrixXd hannah_robot_M = hannah->M();
        // VectorXd hannah_g = hannah->jointGravityVector();
        // double hannah_kinetic_energy = 0.5 * hannah_robot_dq.transpose() * hannah_robot_M * hannah_robot_dq;
		// Eigen::VectorXd hanah_comPosition = hannah->comPosition();
		// double hannah_potential_energy = 90 * hannah_comPosition(2) * -9.8;
        // double hannah_lagrangian = hannah_kinetic_energy - hannah_potential_energy;
        
		// redis_client.set(HANNAH_KINETIC, std::to_string(hannah_kinetic_energy));
		// redis_client.set(HANNAH_POTENTIAL, std::to_string(hannah_potential_energy));
		// redis_client.set(HANNAH_LAGRANGIAN, std::to_string(hannah_lagrangian));


		// MatrixXd tracy_robot_M = tracy->M();
        // VectorXd tracy_g = tracy->jointGravityVector();
        // double tracy_kinetic_energy = 0.5 * tracy_robot_dq.transpose() * tracy_robot_M * tracy_robot_dq;
		// //Eigen::VectorXd tracy_comPosition = tracy->comPosition();
		// //double tracy_potential_energy = 90 * tracy_comPosition(2) * -9.8;
        // double tracy_lagrangian = tracy_kinetic_energy - tracy_potential_energy;
        
		// redis_client.set(TRACY_KINETIC, std::to_string(tracy_kinetic_energy));
		// redis_client.set(TRACY_POTENTIAL, std::to_string(tracy_potential_energy));
		// redis_client.set(TRACY_LAGRANGIAN, std::to_string(tracy_lagrangian));

        // redis_client.setEigen(HANNAH_TORO_JOINT_ANGLES_KEY, hannah_robot_q);
        // redis_client.setEigen(HANNAH_TORO_JOINT_VELOCITIES_KEY, hannah_robot_dq);
		// redis_client.setEigen(TRACY_TORO_JOINT_ANGLES_KEY, tracy_robot_q);
        // redis_client.setEigen(TRACY_TORO_JOINT_VELOCITIES_KEY, tracy_robot_dq);

        {
            lock_guard<mutex> lock(mutex_update);
        }
    }
    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
}

