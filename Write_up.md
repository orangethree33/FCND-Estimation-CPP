# Estimation Project #

This README is broken down into the following sections:

- [Setup](#setup) - the environment and code setup required to get started and a brief overview of the project structure
- [Code](#Code) - the code of this project
- [The Tasks AND the Project Rubric](#the-tasks) - the tasks you will need to complete for the project
- [Submission](#submission) - overview of the requirements for your project submission


### Setup ###

This project will continue to use the C++ development environment you set up in the Controls C++ project.

 1. Clone the repository
 ```
 git clone https://github.com/udacity/FCND-Estimation-CPP.git
 ```
 
 2. Import the code into IDE like done in the [Controls C++ project](https://github.com/udacity/FCND-Controls-CPP#development-environment-setup)
 
 3. Compile and run the estimation simulator just as you did in the controls project

## Code ##
#### UpdateFromIMU() Function ####
![r](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/Mat3x3F_r.jpg)
```
void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{


  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  pitchEst = 0;
  rollEst = 0;
  float theta=pitchEst;
  float phi=rollEst;
  Mat3x3F r=Mat3x3F::Zeros();
  r(0,0)=1; r(0,1)=sin(phi)*tan(theta); r(0,2)=cos(phi)*tan(theta);
  r(1,0)=0; r(1.1)=cos(phi);            r(1,2)=-sin(phi);
  r(2,0)=0; r(2,1)=sin(phi)/cos(theta); r(2,2)=cos(phi)/cos(theta);

  V3F e_angle_dot=r*gyro;
	 float predictedPitch = pitchEst + dtIMU * e_angle_dot.y;
	 float predictedRoll = rollEst + dtIMU * e_angle_dot.x;
	 ekfState(6) = ekfState(6) + dtIMU * e_angle_dot.z;
 // normalize yaw to -pi .. pi
  if (ekfState(6) > F_PI) 
    ekfState(6) -= 2.f*F_PI;
  if (ekfState(6) < -F_PI) 
    ekfState(6) += 2.f*F_PI;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
#### FromEuler123_RPY Function() ####
```

Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
 Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
 attitude.IntegrateBodyRate(gyro,dtIMU);
 float predictedPitch = attitude.Pitch();
 float predictedRoll = attitude.Roll();
 ekfState(6) = attitude.Yaw();
 // normalize yaw to -pi .. pi
 if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
 if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

 
  /////////////////////////////// END STUDENT CODE ////////////////////////////`
  

 
 ```
  #### GetRbgPrime() Function ####
  ![Rbg](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/Rbg.png)
  ```
 MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
 
  MatrixXf RbgPrime(3, 3);
  RbgPrime.setZero();
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //Rpg rotation ,Rpg' rotation as partial derivation of psi
  float sinPsi = sin(yaw); float cosPsi = cos(yaw);
  float sinTheta = sin(pitch); float cosTheta = cos(pitch);
  float sinPhi = sin(roll); float cosPhi = cos(roll);

  RbgPrime(0, 0) = -cosTheta * sinPsi;
  RbgPrime(0, 1) = -sinPhi * sinTheta*sinPsi - cosTheta * cosPsi;
  RbgPrime(0, 2) = -cosPhi * sinTheta*sinPsi + sinPhi * cosPsi;
  RbgPrime(1, 0) = cosTheta * cosPsi;
  RbgPrime(1, 1) = sinPhi * sinTheta*cosPsi - cosPhi * sinPsi;
  RbgPrime(1, 2) = cosPhi * sinTheta*cosPsi + sinPhi * sinPsi;
  //
/////////////////////////////// END STUDENT CODE ////////////////////////////

  return RbgPrime;
  ```
  
 #### GetRbgPrime() Function  #### 
 ![G](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/G.png)
  ```
 MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

 MarixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
 gPrime.setIdentity();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  gPrime(0,3)=dt;
  gPrime(1,4)=dt;
  gPrime(2,5)=dt;
  //this part is from my mentor
  gPrime(3, 6) = (RbgPrime(0)*accel).sum()*dt;
  gPrime(4, 6) = (RbgPrime(1)*accel).sum()*dt;
  gPrime(5, 6) = (RbgPrime(2)*accel).sum()*dt;




  ekfCov=gPrime*ekfCov*gPrime.transposeInPlace+Q;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  ekfState = newState;
}
```
#### UpdateFromGPS() Function #### 
```
void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  VectorXf z(6), zFromX(6);
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;

  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
  hPrime.setZero();
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //zFromX(0)=z(0);
  zFromX(0)=ekfState(0);
  zFromX(1)=ekfState(1);
  zFromX(2)=ekfState(2);
  zFromX(3)=ekfState(3);
  zFromX(4)=ekfState(4);
  zFromX(5)=ekfState(5);
  zFromX(6)=ekfState(6);

  for (int i=0;i<6;i++)
    {
      hPrime(i,i)=1;

      }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_GPS, zFromX);
}
```
#### UpdateFromMag() Fuction
```
void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
  VectorXf z(1), zFromX(1);
  z(0) = magYaw;

  MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
  hPrime.setZero();
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
//normalize the difference between the measured and the estmated
        hPrime(0, 6) = 1;
	zFromX = hPrime * ekfState(6);

	VectorXf yaw_error = z - zFromX;
	
	if (yaw_error(0) > F_PI)z(0) -= 2.f*F_PI;
	if (yaw_error(0) < -F_PI) z(0) += 2.f*F_PI;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_Mag, zFromX);
}
```

## The Tasks AND the Project Rubric ##

###step 1:Implement Estimator


Once again, you will be building up your estimator in pieces.  At each step, there will be a set of success criteria that will be displayed both in the plots and in the terminal output to help you along the way.

Project outline:

 - [Step 1: Sensor Noise](#step-1-sensor-noise)
 - [Step 2: Attitude Estimation](#step-2-attitude-estimation)
 - [Step 3: Prediction Step](#step-3-prediction-step)
 - [Step 4: Magnetometer Update](#step-4-magnetometer-update)
 - [Step 5: Closed Loop + GPS Update](#step-5-closed-loop--gps-update)
 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)



### Step 2: Sensor Noise ###
![orig]()
it is the scenario 06_SensorNoise.when the simulator runs,we can see the two files which is in/config/Graph1 and /Graph2 then we use the math to calculate the value of parameters .
The video is following,when the scenario passed the test,I can see the lines outputs on the terminal 
1. Run the simulator in the same way
Choose scenario `06_NoisySensors`.,This is scenario6.gif running on my vs
change the parameters
2. Run the simulator. 
![ch6](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/06.png)
![ch6](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/06result.png)


### Step 3: Attitude Estimation ###
In this steps,we need to write the code of UpDateIMU,the only thing I need is to intergrate the parameter of gyroscope in the estimated roll and pitch.
```
Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
attitude.IntegrateBodyRate(gyro,dtIMU);
float predictedPitch = attitude.Pitch();
float predictedRoll = attitude.Roll();
ekfState(6) = attitude.Yaw();
// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

```
(during the studying time ,I try a lot of methods and I met a lot of problems,thanks to mentor and some teachers)
create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
	//    OR 
	//    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
	//       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)
I try the first one in the first time,first I think to find the roll,pitch,and also yaw derivates using the equations that I had showed in the Code part,and then when I the derivates,I think I supposed to multiply it through dt ,but somehow I failed ,still, I did not figute it out.so my teachers give me another to solve it.
well,it works well. 

1. Run scenario `07_AttitudeEstimation`.  
2. This is scenario7.gif running on vs
![ch7](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/NCH7.gif)
and this is the result
![result](https://github.com/orangethree33/FCND-Estimation-CPP/tree/master/images)



### Step 4: Prediction Step ###
This has been sperated in two parts.

1. Run scenario `08_PredictState`.  
2. In `QuadEstimatorEKF.cpp`, implement the state prediction step in the `PredictState()` functon. 
this is scenario.gif
![nch8](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/NCH8.gif)


3. Now let's introduce a realistic IMU, one with noise.  Run scenario `09_PredictionCov`. You will see a small fleet of quadcopter all using your prediction code to integrate forward. You will see two plots:
   - The top graph shows 10 (prediction-only) position X estimates
   - The bottom graph shows 10 (prediction-only) velocity estimates
You will notice however that the estimated covariance (white bounds) currently do not capture the growing errors.
this is scenario9.gif 
![nch9](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/NCH9.gif)












### Step 5: Magnetometer Update ###



1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift.
after getting the reviews I adjust my code
this is the scenario10.gif
![NCH10]()








### Step 6: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.
```
Quad.UseIdealEstimator = 0
```

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0

```
this is the scenario.11 gif
![nch11](https://github.com/orangethree33/FCND-Estimation-CPP/blob/master/images/NCH11.gif)






## Submission ##

For this project,the files that I would  submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`
   




