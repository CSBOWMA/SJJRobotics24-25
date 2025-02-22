## Libraries
holds all libraries created for the competition to allow ease of development

## AutoRobot

The primary library handling all code for the autonomous period. The library allows for a user to easily write a script for any given robot by just tweaking the required variables

This library was designed for reusability in future seasons allowing for those with less experience to have a jumping off point in development

The main focus of this library was handling the drive through the use of [mecanum wheels](https://en.wikipedia.org/wiki/Mecanum_wheel).

I wanted the members of the team to be able to simply call a drive function of the robot and have it reliably drive that distance in any cardinal direction. The main goal is reliabilty, the robot should move 12 inches every time if nothing changes in the code. As such by using data provided by the onboard imu, and odometry pods we were able to easily measure the distance the robot was going and adjust if there was any misdirection.

## movementCurves

This library allowed for nonlinear movement of the robot by allowing the power given to the motors to trace an equation similar to [animation curves](https://docs.unity3d.com/Manual/animeditor-AnimationCurves.html) found in animation. This allowed for smoothing of movement and decreased slipping caused by sudden changes in the angular velocity of the wheels. 

## vector

This holds a class vector class that allowed for ease of use in vector based math. One such application was allowing the robot to drive relative to the driver instead of the robot. By using the on board IMU we could get the current Yaw of the robot and then adjust the driving vector accordingly allowing for the user to drive forward without having to make the calculations in their head. 

## robotPeripherals

This folder contains the class for each of the components of the robot. Allowing for anyone to simply create an instance of a class and interact with the robot through said class.