#include "boss.h"

//////////////////// AGENT MANAGER //////////////////////
AgentManager::AgentManager(ros::NodeHandle nh)
{
	nh_ = nh;
}
/////////////////////////////////////////////////////////

///////////////////////// BOSS //////////////////////////
Boss::Boss()
{
	// init goals
	GoalArea area;
	area.goal.type="Position";

	//0// to have easier index
	list_goals_.push_back(area);

	//1//
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//2//
	area.goal.x=3.15; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//3//
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//4//
	area.goal.x=7.90; 	area.goal.y=5.1; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//5//
	area.goal.x=7.8; 	area.goal.y=9.98; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//6//
	area.goal.x=3.42; 	area.goal.y=9.48; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//7//
	area.goal.x=4.72; 	area.goal.y=17.68; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//8//
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//9//
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//10//
	area.goal.x=0.65; 	area.goal.y=8.50; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);

	// Scenarios //
	// Wide Area
	//11 // init wide area H
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//12 // goal wide are H
	area.goal.x=9.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//13 // init wide area R
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//14 // goal wide area R
	area.goal.x=2.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);

	// Narrow Passage //
	//15 // init narrow passage H
	area.goal.x=1.0; 	area.goal.y=1.9; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	/////// goal narrow passage H => 4
	//16 // init narrow passage R
	area.goal.x=7.50; 	area.goal.y=7.32; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	/////// goal narrow passage R => 1

	// Corridor //
	//17 // init corridor H
	area.goal.x=10.35; 	area.goal.y=2.60; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//18 // goal corridor H
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//19 // init corridor R
	area.goal.x=10.35; 	area.goal.y=14.3; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//20 // goal corridor R
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);

	// Narrow corridor //
	//21 // init narrow corridor H
	area.goal.x=0.65; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//22 // goal narrow corridor H
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//23 // init narrow corridor R
	area.goal.x=0.65; 	area.goal.y=14.1; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//24 // goal narrow corridor R
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);

}

void Boss::showState()
{
}

void Boss::askChoice()
{
	// Ask Main choice_
	while(ros::ok() && (cout	<< "1- Send goal" << endl
					<< "2- Scenario" << endl
					<< "3- Set Attitude" << endl
					<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=1 && choice_<=3)))
		cleanInput();
	cout << endl;

	switch(choice_)
	{
		// Send goal
		case 1:
			while(ros::ok() && (cout	<< "1- From List" << endl
							<< "2- Enter coordinates" << endl
							<< "0- Back" << endl
							<< "Choice ? ")
			&& (!(cin >> choice_) || !(choice_>=0 && choice_<=2)))
				cleanInput();
			cout << endl;

			switch(choice_)
			{
				// From list
				case 1:
					while(ros::ok() && (cout << "Select a goal [1-10] " << endl
								 << "0- Back" << endl
								 << "Choice ? ")
					&& (!(cin >> choice_) || !(choice_>=0 && choice_<=10)))
						cleanInput();
					cout << endl;

					// Select agent
					// show list agents
					//	for agent_managers_.size()
					//		cout << i << "- " << agent_managers_[i].showName();
					// 	input
					// if human goals[i].goal (human_sim::Goal)
					// if robot getPose(goals[i].goal) (geometry_msgs::PoseStamped)

					break;
				// Enter coordinates
				case 2:
					break;
				// Back
				case 0:
					break;

				default:
					break;
			}
			break;

		// Scenario
		case 2:
			break;

		// Set attitude
		case 3:
			break;


		default:
			break;
	}
}

void Boss::cleanInput()
{
	cout << "Wrong input..." << endl << endl;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	choice_ = 0;
}
/////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boss");

	Boss boss;

	while(ros::ok())
	{
		boss.showState();
		boss.askChoice();
	}
}
