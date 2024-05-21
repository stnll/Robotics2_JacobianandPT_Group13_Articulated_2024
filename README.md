# Robotics2_JacobianandPT_Group13_Articulated_2024
## <h1 align="center">Jacobian Matrix and Path & Trajectory Planning																
 
## <h2 align="center"> Abstract 
  In the realm of robotics and automated systems, the Jacobian matrix plays a pivotal role in the kinematic analysis and control of robotic manipulators. This repository will serve as Group 13-Articulated, final project in Robotics 2. This focuses on the Articulated Manipulator, providing an overview of the important characteristics of jacobian matrix and path & trajectory planning. This repository explores the theoretical foundation and practical applications of the Jacobian matrix in the context of path and trajectory planning. To further clarify the concepts of how an Articulated Manipulator functions, more images, and descriptions are also included.  

## <h2 align="center">TABLE OF CONTENTS:
<h4 align="center">  
 
  [I. Introduction](#i-introduction)
  
  [II. Jacobian Matrix](#ii-degrees-of-freedom)
  
  [III. Differential Equation	](#iii-kinematic-diagram-and-d-h-frame)
  
  [IV. Path and Trajectory Planning](iv-d-h-parametric-table)

  [V. References](viii-references)

  ## <h2 align="center">I. INTRODUCTION
<p align="center">
  <img alt = "Python" width="300" src="https://github.com/stnll/Robotics2_FKandIK_Group13_Articulated_2024/assets/157665975/befaf219-f7f4-46e6-8c83-3bb39b83a67e">
 <h5 align="justify"> The value of articulated robots is derived from their capacity to increase efficiency, precision, and safety in a variety of processes. These robots are meant to conduct repetitive and risky jobs that people typically find too difficult or unsafe to execute. 
</p>
     As the years passed, Articulated Manipulators undertook an innovation to take full advantage and unlock the full potential of their automation processes and make them more efficient. As a result, due to its ability to move quickly and precisely, it is used in manufacturing from assembly to material handling. In addition, Articulated Manipulators are now often used in Healthcare, Agriculture, Military Defense, and Education. With the help of advanced sensors, artificial intelligence, and adaptive algorithms, Articulated Manipulators continually improve their capabilities. For a variety of reasons, articulated robots are ideal for assembly applications. Their joints enable them to move in ways that other robots do not. Their payload capacity also allows them to transport items that other robot kinds cannot. They are accurate enough for even minor assembly operations. 

## <h2 align="center">II. JACOBIAN MATRIX
<img align="right" alt="Coding" width="200" src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/e7c6ff79-990f-4a5b-b54e-adf150ac52a3">
<h5 align="justify">   The Jacobian matrix is a mathematical tool used to evaluate the connection between numerous variables in calculus and linear algebra. Although the notion predates Carl Gustav Jacob Jacobi, it was named after him. The matrix evolved from the 18th-century studies of differential calculus and partial derivatives by mathematicians such as Euler and Lagrange. The Jacobian matrix is used to solve functions or equation systems with numerous variables. It is made up of the partial derivatives of the output variables with regard to the input variables. This matrix approximates the function linearly around a specified point. The Jacobian matrix, by arranging partial derivatives into matrices, enables fast computations in a variety of domains, including optimization, numerical analysis, differential equations, physics, and computer science. The Jacobian matrix is critical for understanding the behavior of functions and systems with numerous variables. It permits the investigation of rates of change, optimization, and stability analysis. Overall, the Jacobian matrix is a basic topic with numerous applications in mathematics and beyond.
</p>
<p align="center" width="200">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/2feefaf6-a93e-4a2c-a445-25242cac55eb">
<h5 align="justify"> As we can see in this matrix the one that contains the x prime, y prime, z prime, omega x, omega y, and omega z is called the end effector velocity. On the other side, the one that contains theta 1, theta 2, and theta 3 is the joint velocity vector. We would be using this matrix to help us convert the angular velocities of the joints into the velocity of the end effector of a robotic arm.
</p>
<h5 align="center"> This is the formula used in calculating the Jacobian Matrix. 
</p>
<img align="center" alt="Coding" width="300" src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/b76deb6c-92c8-42bb-9dc2-1137a3b2873d">
</p>
<h5 align="justify"> Wherein:
</p>
R = Rotational Matrix
</p>
d = Position Vector
</p>
i = Which joint is being solved
</p>
n = No. of joints
</p>
<h4 align="center">JACOBIAN MATRIX CALCULATION OF A ARTICULATED MANIPULATOR (RRR)

