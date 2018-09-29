#include <gtest/gtest.h>
#include "rebvio/rebvio.hpp"



TEST(EdgeTracker, estimateLs4Acceleration) {
	rebvio::EdgeTracker	edge_tracker(std::make_shared<rebvio::Camera>());
	rebvio::types::Vector3f Vgv = TooN::makeVector(-4.06833e-05, 9.40667e-05, 5.70767e-05);
	rebvio::types::Float dt = 0.05;
	rebvio::types::Vector3f Av = TooN::makeVector(0, 0, 0 );
	rebvio::types::Matrix3f R = TooN::Data(1, 8.83134e-05, -7.48149e-05,
																				 -8.831e-05, 1, 4.57494e-05,
																				 7.4819e-05, -4.57428e-05, 1);
	edge_tracker.estimateLs4Acceleration(-Vgv/dt,Av,R,dt);
	EXPECT_NEAR(0.0162734,Av[0],0.00001);
	EXPECT_NEAR(-0.0376267,Av[1],0.00001);
	EXPECT_NEAR(-0.0228307,Av[2],0.00001);
}


int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}
