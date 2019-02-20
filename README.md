# Robot Arm
This is my initial attempt at simulating a robot arm with 3 degrees of motion.

## Aim ##
During 2018, I enrolled on the course called Introduction to Vision and Robotics. The coursework required us to implement a robot arm in a Python environment. The coursework used a library that allowed limited visualisation of the environment. I decided to implement a similar robot arm, outlined in the coursework description in Unity 3D.

## Method ##
Currently it uses Jacobian Matrices to solve the Inverse Kinematics problem and it uses a simple PD control to create required torques.
In the future I want to redesign it, using Jacobian is considered difficult and has issues, the FABRIK method offers a simpler, vectorized approach to solving the IK problem. 

## Results ##
Unity doesn't provide Matrix algegra like Python does. Currently the code uses some hardcoded Matrix operations, that I wrote. I consider this solution suboptimal, and I am working on implementing these operations using a public library built for C#.

The Robot arm has a few issues:
- The Arm has difficulty reaching some positions that seem simple at first similar to the results we observed on the coursework.
- Joints in unity can bend. While this results in some additional features when colliding with other objects, it also results in a undesired simmulation.
- The Arm has an suprising workspace. Some areas are out of range of the robot when they should be inside. 
