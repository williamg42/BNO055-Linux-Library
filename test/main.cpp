#include <iostream>
#include "../src/BNO055.h"
#include <unistd.h>



using namespace std;




double * Acc_bframe_to_acc_NEDframe(double qW, double qX, double qY, double qZ, double aX, double aY, double aZ ) {

   static double  acc_NEDframe[3]; //X, Y, Z

acc_NEDframe[0] = (qW*qW+qX*qX-qY*qY-qZ*qZ)*aX+2*(qX*qY-qW*qZ)*aY+2*(qW*qY+qX*qZ)*aZ;
acc_NEDframe[1] = 2*(qX*qY+qW*qZ)*aX+(qW*qW-qX*qX+qY*qY-qZ*qZ)*aY+2*(qY*qZ-qW*qX)*aZ;
acc_NEDframe[2] = 2*(qX*qZ+qW*qY)*aX+2*(qY*qZ+qW*qX)*aY+(qW*qW-qX*qX-qY*qY+qZ*qZ)*aZ;



   return acc_NEDframe;
}



int main()
{

double * acc_NEDframe;

BNO055 bno = BNO055(-1, BNO055_ADDRESS_A, 1);

bno.begin(bno.OPERATION_MODE_NDOF_FMC_OFF);

usleep(1000000);

 int temp = bno.getTemp();
  std::cout << "Current Temperature: " << temp << " C" << std::endl;

  bno.setExtCrystalUse(true);

usleep(1000000);
imu::Quaternion quat = bno.getQuat();
imu::Vector<3> acc_bframe = bno.getVector(BNO055::VECTOR_ACCELEROMETER);
usleep(1000000);

std::cout << "qW: " << quat.w() << " qX: " << quat.x() << " qY: " << quat.y() << " qZ: " << quat.z() << std::endl;

acc_NEDframe = Acc_bframe_to_acc_NEDframe(quat.w(), quat.x(), quat.y(), quat.z(), acc_bframe.x(), acc_bframe.y(), acc_bframe.z() );

std::cout << "X: " << acc_NEDframe[0] << " Y: " << acc_NEDframe[1] << " Z: " << acc_NEDframe[2]  << std::endl;

while(1)
{

 
quat = bno.getQuat();
acc_bframe = bno.getVector(BNO055::VECTOR_LINEARACCEL);

acc_NEDframe = Acc_bframe_to_acc_NEDframe(quat.w(), quat.x(), quat.y(), quat.z(), acc_bframe.x(), acc_bframe.y(), acc_bframe.z() );

std::cout << "X: " << acc_NEDframe[0] << " Y: " << acc_NEDframe[1] << " Z: " << acc_NEDframe[2]  << std::endl;


usleep(100000);



 

}
    
    return 0;
}
