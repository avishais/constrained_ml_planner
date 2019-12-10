#include "kdl_class_comau3.h"

// Constructor for the robots
kdl::kdl() {
	b = 344; // Height of base
	l1x = 524.5; 
	l1z = 199;
	l2 = 1250; 
	l3x = 186.5; 
	l3z = 210; 
	l4 = 1250; 
	l5 = 116.5; 
	lee = 22;

	initMatrix(Q12, 4, 4);
	initMatrix(Q13, 4, 4);
	Q12 = {{cos(PI/3),-sin(PI/3),0,RD*cos(PI/3)+RD},{sin(PI/3),cos(PI/3),0,RD*sin(PI/3)},{0,0,1,0},{0,0,0,1}};
	Q13 = {{cos(-PI/3),-sin(-PI/3),0,RD*cos(-PI/3)+RD},{sin(-PI/3),cos(-PI/3),0,RD*sin(-PI/3)},{0,0,1,0},{0,0,0,1}};

	// Joint limits
	q1minmax = deg2rad(180);
	q2max = deg2rad(125);
	q2min = deg2rad(-60);
	q3max = deg2rad(75);
	q3min = deg2rad(-90);
	q4minmax = deg2rad(2700);
	q5minmax = deg2rad(120);
	q6minmax = deg2rad(400);

	initMatrix(T_fk, 4, 4);
	initMatrix(T_fk13, 4, 4);
	initMatrix(T_mult_temp,4, 4);	

	initMatrix(T_pose1, 4, 4);
	T_pose1 = {{cos(ROBOT1_ROT), -sin(ROBOT1_ROT), 0, ROBOT1_X}, {sin(ROBOT1_ROT), cos(ROBOT1_ROT), 0, ROBOT1_Y}, {0, 0, 1, ROBOT1_Z}, {0, 0, 0, 1}};
	initMatrix(T_pose12, 4, 4);
	T_pose12 = {{cos(PI/3), -sin(PI/3), 0, ROBOT2_X-ROBOT1_X}, {sin(PI/3), cos(PI/3), 0, ROBOT2_Y-ROBOT1_Y}, {0, 0, 1, ROBOT2_Z-ROBOT1_Z}, {0, 0, 0, 1}};
	initMatrix(T_pose3, 4, 4);
	T_pose3 = {{cos(ROBOT3_ROT), -sin(ROBOT3_ROT), 0, ROBOT3_X}, {sin(ROBOT3_ROT), cos(ROBOT3_ROT), 0, ROBOT3_Y}, {0, 0, 1, ROBOT3_Z}, {0, 0, 0, 1}};

	//Definition of a kinematic chain & add segments to the chain
	// Robot 1
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base link
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z)))); // Link 1
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2)))); // Link 2
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z)))); // Link 3
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0)))); // EE
	// Wheel
	Rotation r( Q12[0][0],Q12[0][1],Q12[0][2], Q12[1][0],Q12[1][1],Q12[1][2], Q12[2][0],Q12[2][1],Q12[2][2] );
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(Q12[0][3],Q12[1][3],Q12[2][3])))); // Box translation
	chain.addSegment(Segment(Joint(Joint::None),Frame(r))); // Box rotation
	// Robot 2 - backwards
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(lee,0.0,0.0)))); // Box translation	
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l3x,0.0,-l3z)))); // Link 3 "-"
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,-l2)))); // Link 2
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l1x,0.0,-l1z)))); // Link 1
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-b)))); // Base link

	// Robot 1
	Rotation r1( T_pose1[0][0],T_pose1[0][1],T_pose1[0][2], T_pose1[1][0],T_pose1[1][1],T_pose1[1][2], T_pose1[2][0],T_pose1[2][1],T_pose1[2][2] );
	rob1.addSegment(Segment(Joint(Joint::None),Frame(Vector(T_pose1[0][3],T_pose1[1][3],T_pose1[2][3])))); // Robot translation
	rob1.addSegment(Segment(Joint(Joint::None),Frame(r1))); // Robot rotation
	rob1.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base link
	rob1.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z)))); // Link 1
	rob1.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2)))); // Link 2
	rob1.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z)))); // Link 3
	rob1.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0)))); // Link 4
	rob1.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0)))); // Link 5
	rob1.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0)))); // EE
	// Wheel
	Rotation r13( Q13[0][0],Q13[0][1],Q13[0][2], Q13[1][0],Q13[1][1],Q13[1][2], Q13[2][0],Q13[2][1],Q13[2][2] );
	rob1.addSegment(Segment(Joint(Joint::None),Frame(Vector(Q13[0][3],Q13[1][3],Q13[2][3])))); // Wheel translation
	rob1.addSegment(Segment(Joint(Joint::None),Frame(r13))); // Wheel rotation

	// Robot 3
	Rotation r3( T_pose3[0][0],T_pose3[0][1],T_pose3[0][2], T_pose3[1][0],T_pose3[1][1],T_pose3[1][2], T_pose3[2][0],T_pose3[2][1],T_pose3[2][2] );
	rob3.addSegment(Segment(Joint(Joint::None),Frame(Vector(T_pose3[0][3],T_pose3[1][3],T_pose3[2][3])))); // Robot translation
	rob3.addSegment(Segment(Joint(Joint::None),Frame(r3))); // Robot rotation
	rob3.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base link
	rob3.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z)))); // Link 1
	rob3.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2)))); // Link 2
	rob3.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z)))); // Link 3
	rob3.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0)))); // Link 4
	rob3.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0)))); // Link 5
	rob3.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0)))); // EE


	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	jointpositions = JntArray(nj);
	unsigned int nj1 = rob1.getNrOfJoints();
	jointpositions1 = JntArray(nj1);

	initVector(q_solution12, nj);
	initVector(q_solution3, 6);
	initVector(q_solution, 18);

	cout << "Initiated chain with " << nj+nj1 << " joints.\n";
}

// ----- Project -------

bool kdl::GD(State q) {

	State q12(12);
	for (int i = 0; i < 12; i++)
		q12[i] = q[i];
	
	if (!GD12(q12))
		return false;
	q12 = get_GD12_result();

	State q1(6);
	for (int i = 0; i < 6; i++) 
		q1[i] = q12[i];
	FK13(q1, 1);
	Matrix T1 = get_FK13_solution();

	State q3(6);
	for (int i = 0; i < 6; i++) 
		q3[i] = q[i+12];
	Matrix T3 = MatricesMult(T1, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 1
	if (!IK3(q3, T3))
		return false;
	q3 = get_IK3_solution();

	for (int i = 0; i < 12; i++)
		q_solution[i] = q12[i];
	for (int i = 0; i < 6; i++)
		q_solution[i+12] = q3[i];

	return true;
}	

State kdl::get_GD_result() {
	return q_solution;
}

// ----- Descend -------

bool kdl::GD12(State q_init_flip) {

	bool valid = true;

	// Flip robot two vector
	State q_init(q_init_flip.size());
	for (int i = 0; i < 6; i++)
		q_init[i] = q_init_flip[i];
	for (int i = 11, j = 6; i >= 6; i--,j++)
		q_init[j] = q_init_flip[i];
	q_init[11] = -q_init[11];

	State q(12);

	IK_counter++;
	clock_t begin = clock();

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T_pose12[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T_pose12[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,10000,1e-5);//Maximum 10000 iterations, stop at accuracy 1e-5

	//Creation of jntarrays:
	JntArray qKDL(chain.getNrOfJoints());
	JntArray qInit(chain.getNrOfJoints());

	for (int i = 0; i < chain.getNrOfJoints(); i++)
		qInit(i) = q_init[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	bool result = false;
	if (ret >= 0) {

		for (int i = 0; i < 12; i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI)
				q[i] -= 2*PI;
			if (q[i]<-PI)
				q[i] += 2*PI;
		}

		for (int i = 0; i < 6; i++)
			q_solution12[i] = q[i];
		for (int i = 11, j = 6; i >= 6; i--,j++)
			q_solution12[j] = q[i];
		q_solution12[6] = -q_solution12[6];

		if (include_joint_limits && !check_angle_limits(q_solution12))
			result = false;
		else
			result = true;
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

bool kdl::check_angle_limits(State q) {

	if (fabs(q[0]) > q1minmax)
		return false;
	if (q[1] < q2min || q[1] > q2max)
		return false;
	if (q[2] < q3min || q[2] > q3max)
		return false;
	if (fabs(q[3]) > q4minmax)
		return false;
	if (fabs(q[4]) > q5minmax)
		return false;
	if (fabs(q[5]) > q6minmax)
		return false;

	if (q.size()==6)
		return true;

	if (fabs(q[6]) > q1minmax)
		return false;
	if (q[7] < q2min || q[7] > q2max)
		return false;
	if (q[8] < q3min || q[8] > q3max)
		return false;
	if (fabs(q[9]) > q4minmax)
		return false;
	if (fabs(q[10]) > q5minmax)
		return false;
	if (fabs(q[11]) > q6minmax)
		return false;

	return true;
}

State kdl::get_GD12_result() {
	return q_solution12;
}

// -----FK-------

// This is only for validation. There is no use for this function in terms of closed chain kinematics
void kdl::FK(State q) {

	// Flip robot two vector
	State q_flip(q.size());
	for (int i = 0; i < 6; i++)
		q_flip[i] = q[i];
	for (int i = 11, j = 6; i >= 6; i--,j++)
		q_flip[j] = q[i];
	q_flip[11] = -q_flip[11];

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	for (int i = 0; i < q_flip.size(); i++)
		jointpositions(i) = q_flip[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk[i][3] = cartposFK.p(i);
		for (int j = 0; j < 3; j++) {
			T_fk[i][j] = cartposFK.M(i,j);
			T_fk[i][j] = fabs(T_fk[i][j]) < 1e-4 ? 0 : T_fk[i][j];
		}
	}
	T_fk[3][0] = T_fk[3][1] = T_fk[3][2] = 0;
	T_fk[3][3] = 1;
}

// FK for robot 1 (with wheel) or 3
void kdl::FK13(State q, int robot_num) {
	//cout << "qfk: "; printVector(q);

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = robot_num == 1 ? ChainFkSolverPos_recursive(rob1) : ChainFkSolverPos_recursive(rob3);

	for (int i = 0; i < q.size(); i++)
		jointpositions1(i) = q[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions1, cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk13[i][3] = cartposFK.p(i);
		for (int j = 0; j < 3; j++) {
			T_fk13[i][j] = cartposFK.M(i,j);
			T_fk13[i][j] = fabs(T_fk13[i][j]) < 1e-4 ? 0 : T_fk13[i][j];
		}
	}
	T_fk13[3][0] = T_fk13[3][1] = T_fk13[3][2] = 0;
	T_fk13[3][3] = 1;
}

Matrix kdl::get_FK_solution() {

	return T_fk;
}

Matrix kdl::get_FK13_solution() {

	return T_fk13;
}

// ---------- IK of robot 3 -------------------

bool kdl::IK3(State q3, Matrix T3) {

	bool valid = true;
	State q(q3.size());

	IK_counter++;
	clock_t begin = clock();

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T3[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T3[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(rob3); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(rob3);//Inverse velocity solver
	ChainIkSolverPos_NR iksolver(rob3,fksolver,iksolverv,10000,1e-5);//Maximum 10000 iterations, stop at accuracy 1e-5

	//Creation of jntarrays:
	JntArray qKDL(rob3.getNrOfJoints());
	JntArray qInit(rob3.getNrOfJoints());

	for (int i = 0; i < rob3.getNrOfJoints(); i++)
		qInit(i) = q3[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	bool result = false;
	if (ret >= 0) {

		for (int i = 0; i < q3.size(); i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI)
				q[i] -= 2*PI;
			if (q[i]<-PI)
				q[i] += 2*PI;
		}

		for (int i = 0; i < 6; i++)
			q_solution3[i] = q[i];

		if (include_joint_limits && !check_angle_limits(q_solution3))
			result = false;
		else
			result = true;
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

State kdl::get_IK3_solution() {
	return q_solution3;
}

State kdl::sample_q() {

	State q(18);

	while (1) {
		for (int i = 0; i < q.size(); i++)
			q[i] = -PI + (double)rand()/RAND_MAX * 2*PI;

		if (!GD(q)) {
			continue;
		}

		q = get_GD_result();

		return q;
	}
}


//------------------------

// Misc
void kdl::initMatrix(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void kdl::initVector(State &V, int n) {
	V.resize(n);
}

double kdl::deg2rad(double deg) {
	return deg * PI / 180.0;
}

void kdl::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void kdl::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}


void kdl::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

// Matrix multiplication
Matrix kdl::MatricesMult(Matrix M1, Matrix M2) {
	clearMatrix(T_mult_temp);
	for(unsigned i = 0; i < 4; ++i)
		for(unsigned j = 0; j < 4; ++j)
			for(int k = 0; k < 4; ++k)
			{
				T_mult_temp[i][j] += M1[i][k] * M2[k][j];
			}
	return T_mult_temp;
}

void kdl::log_q(State q) {
	std::ofstream myfile;
	myfile.open("../paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();
}
