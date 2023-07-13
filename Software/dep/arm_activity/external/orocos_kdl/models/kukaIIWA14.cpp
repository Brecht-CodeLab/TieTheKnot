// Dynamical parameters based on paper: Parameter identification of the
// KUKA LBR iiwa robot including
// constraints on physical feasibility

#include <chain.hpp>
#include "models.hpp"

namespace KDL{
    Chain KukaIIWA14(){
        Chain KukaIIWA14;

	//joint 0
	KukaIIWA14.addSegment(Segment(Joint(Joint::None),
				  Frame(Rotation::RPY(PI/4, 0.0, 0.0))*Frame::DH_Craig1989(0.0, 0.0, 0.36, 0.0)
				  ));
	//joint 1
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.2415, 0.0),
					Frame::DH_Craig1989(0.0, PI/2, 0.0, 0.0),
					Frame::DH_Craig1989(0.0, PI/2, 0.0, 0.0).Inverse()*RigidBodyInertia(3.94781,
									Vector(-0.00351, 0.00160, -0.03139),
									RotationalInertia(0.00455, 0.00454, 0.00029, 0.0, 0.0, 0.00001))
									));
				   
	//joint 2 
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.37328, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.42, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.42, 0.0).Inverse()*RigidBodyInertia(4.50275,
									Vector(-0.00767, 0.16669,-0.00355),
									RotationalInertia(0.00032, 0.00010, 0.00042, 0.0, 0.0, 0.0))
									));
				  
	//joint 3
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.11025, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.0, 0.0).Inverse()*RigidBodyInertia(2.45520,
									Vector(-0.00225, -0.03492, -0.02652),
									RotationalInertia(0.00223, 0.00219, 0.00073, -0.00005, 0.00007, 0.00007))
									));
				  
	//joint 4
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.10, 0.0),
				  Frame::DH_Craig1989(0.0, PI/2, 0.40, 0.0),
				  Frame::DH_Craig1989(0.0, PI/2, 0.40, 0.0).Inverse()*RigidBodyInertia(2.61155,
									Vector(0.00020, -0.05268, 0.03818),
									RotationalInertia(0.03844, 0.01144, 0.04988, 0.00088, -0.00112, -0.00111))
									));
				  
	//joint 5
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.10, 0.0),
				  Frame::DH_Craig1989(0.0, PI/2, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, PI/2, 0.0, 0.0).Inverse()*RigidBodyInertia(3.41,
									Vector(0.00005, -0.00237, -0.21134),
									RotationalInertia(0.00277, 0.00284 , 0.00012,  -0.00001,  0.00001, 0.0))
									));
				  
	//joint 6
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.12484, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, -PI/2, 0.0, 0.0).Inverse()*RigidBodyInertia(3.38795,
									Vector(0.00049, 0.02019, -0.02750),
									RotationalInertia(0.00050, 0.00281, 0.00232, -0.00005, -0.00003, -0.00004))
									));
	//joint 7
	KukaIIWA14.addSegment(Segment(Joint(Joint::RotZ, 1.0, 0.0, 0.0, 0.10, 0.0),
				   Frame::Identity(),
				   RigidBodyInertia(0.35432,
									Vector(-0.03466, -0.02324, 0.07138),
									RotationalInertia(0.00795, 0.01089, 0.00294, 0.00022, -0.00029, -0.00029))));
    
		return KukaIIWA14;
    }
    
}
