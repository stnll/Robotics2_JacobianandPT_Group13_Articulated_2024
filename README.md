# Robotics2_JacobianandPT_Group13_Articulated_2024
## <h1 align="center">Jacobian Matrix and Path & Trajectory Planning																
 
## <h2 align="center"> Abstract 
  In the realm of robotics and automated systems, the Jacobian matrix plays a pivotal role in the kinematic analysis and control of robotic manipulators. This repository will serve as Group 13-Articulated, final project in Robotics 2. This focuses on the Articulated Manipulator, providing an overview of the important characteristics of jacobian matrix and path & trajectory planning. This repository explores the theoretical foundation and practical applications of the Jacobian matrix in the context of path and trajectory planning. To further clarify the concepts of how an Articulated Manipulator functions, more images, and descriptions are also included.  

## <h2 align="center">TABLE OF CONTENTS:
<h4 align="center">  
 
  [I. Introduction](#i-introduction)
  
  [II. Jacobian Matrix](#ii-jacobian-matrix)
  
  [III. Differential Equation	](#iii-differential-equation)
  
  [IV. Path and Trajectory Planning](#iv-path-and-trajectory-planning)

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
<h4 align="center">JACOBIAN MATRIX CALCULATION OF AN ARTICULATED MANIPULATOR (RRR)
<h5 align="center"> Substituting the Formulas for the Linear and Rotational Part 
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/553585eb-8822-4828-8d62-1d0b36e65c4e">
<h5 align="center"> Solving for the Column 1 of the Jacobian Matrix
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a62ba100-7649-49ac-9a09-1cae01bbed55">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/e2970469-be80-44f2-be36-ba967fd13540">
<h5 align="center"> Using Cross Product Method
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/258e8834-8573-4a74-a5c4-bc41023df753">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/d6f8cbe5-e314-46ba-9806-93ee17011047">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/61136c11-0376-45c6-89ee-eebf70da8822">
<h5 align="center"> Solving for the Column 2 of the Jacobian Matrix
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/0222b094-a8fd-4638-8ec6-1f4b2bd34e4b">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/56375dfd-5d0a-4ae9-82b5-76486e144adb">
<h5 align="center"> Using Cross Product Method
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/258e8834-8573-4a74-a5c4-bc41023df753">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/dcbc8e50-fb24-49d1-a1ef-ff27b6580d2a">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/8f1b9596-e4cf-42f8-82c0-0199685fb34e">
<h5 align="center">Solving for the Column 3 of the Jacobian Matrix
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/bbf9138f-6b68-4a7c-82d2-bfd49be73607)
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/4faab4dd-65cf-4fb0-9a23-cf7152ebf14e">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/c9a42e7f-3581-42ea-afca-f2f792ab7412">
<h5 align="center"> Using Cross Product Method
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/258e8834-8573-4a74-a5c4-bc41023df753">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/e6c3dafd-c4cf-49da-acb0-42c9b28f1a8d">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3c9d2d08-8609-4fd3-8c23-9710f07dce0b">
<h5 align="center"> Substituting the Matrices from Columns 1, 2, and 3 to the Jacobian Matrix
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/7553fe41-da0f-4816-950d-f1bbe64f1a20">
<h5 align="center">Assigning Values to the Link Lengths and Joint Variables
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/6dab705d-ac22-47ec-b408-31f310625d4d">
<h5 align="center">Substituting the Values to the Jacobian Matrix
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/1f4f25cc-cce5-49f7-8d14-67731e9cd721">
<h5 align="center">Solving the End Effector Velocity Vector
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/db26ddec-d219-4dc1-9096-d1e0ed307d68">
<h4 align="center">Supplementary Video about the Jacobian Matrix
<h5 align="center">To further understand how to get the Jacobian Matrix of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
 <a href="https://drive.google.com/file/d/10rq_Nah5IroM_ag2xdpRiHNySk9VqwC9/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/4a30b7b0-b572-4db2-814e-887b9e37b2be>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/4a30b7b0-b572-4db2-814e-887b9e37b2be" width = 600 title="Discussion of Jacobian Matrix for the Articulated Manipulator">
</a> 

## <h2 align="center">III. DIFFERENTIAL EQUATION
<h5 align="justify">The differential method in the context of the Jacobian matrix involves using the Jacobian to analyze and approximate the behavior of multivariable functions. This approach is particularly useful for linear approximations, solving systems of equations, and understanding the local behavior of transformations.
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/92c2cd74-96ff-4910-ad28-64a4c0c8587f">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/1eef8014-5d44-45ba-a65b-ab70031cf6cc">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/efbf3072-0b2e-4f9a-90a9-3ff6e75b2e29">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/1f4f25cc-cce5-49f7-8d14-67731e9cd721">
<p align="center">
  <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/db26ddec-d219-4dc1-9096-d1e0ed307d68">
<h4 align="center">Supplementary Video about the Differential Equation
<h5 align="center">To further understand how to get the Differential Equation of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
 <a href="https://drive.google.com/file/d/16PfOglQpvldTvxZH4LyJ3fdDTh_0XcKU/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3df9627f-f75b-41ab-886a-06e3b5ff1f1e>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a77dd704-065c-4754-aa49-12be80ee2054" alt="Discussion of Differential Equation for the Articulated Manipulator" width = 600 title="Discussion of Differential Equation for the Articulated Manipulator">
</a> 
<h4 align="center">Supplementary Video about the SINGULARITY
<h5 align="center">To understand how to get the SINGULARITY of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
 <a href="https://drive.google.com/file/d/1KtTT0IV5t4TWzQTjF9UpJw7MEQI5g80M/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3df9627f-f75b-41ab-886a-06e3b5ff1f1e>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a77dd704-065c-4754-aa49-12be80ee2054" alt="Discussion of Singularity for the Articulated Manipulator" width = 600 title="Discussion of Singularity for the Articulated Manipulator">
</a> 
 
## <h2 align="center">IV. PATH AND TRAJECTORY PLANNING
<h5 align="justify">Path and trajectory planning are critical aspects in fields such as robotics, autonomous vehicles, aerospace, and animation. These processes involve determining a sequence of positions and corresponding actions that a moving entity (like a robot or a drone) should follow to reach a destination efficiently and safely.
</p> 
 <h4 align="center">PICK AND PLACE
<p align="center">
  <img alt = "Python" width="500" src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/d11e69a4-1eb4-4034-b163-df9cc0d8b09a">
</p> 
 </p> 
 </p> 
 <h4 align="center">WELDING
<p align="center">
  <img alt = "Python" width="500" src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/2c003f71-6748-4615-8ea5-b5827e8d6f86">
 </p> 
</a> 
<h4 align="center">Supplementary Video about the PATH AND TRAJECTORY (MATLAB)
<h5 align="center">To understand PATH AND TRAJECTORY (MATLAB) of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
<a href="https://drive.google.com/file/d/1rkvcpdygYsD-PsXV5G63CRLuoiRLADk-/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3df9627f-f75b-41ab-886a-06e3b5ff1f1e>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a77dd704-065c-4754-aa49-12be80ee2054" alt="Discussion of Singularity for the Articulated Manipulator" width = 600 title="Discussion of PATH AND TRAJECTORY (MATLAB)">

</a> 
<h4 align="center">Supplementary Video about the PATH AND TRAJECTORY (PYTHON)
<h5 align="center">To understand PATH AND TRAJECTORY (PYTHON) of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
<a href="https://drive.google.com/file/d/15Ks-KWz9cz0KKY_rOD8W6e80AqeyajO1/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3df9627f-f75b-41ab-886a-06e3b5ff1f1e>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a77dd704-065c-4754-aa49-12be80ee2054" alt="Discussion of Singularity for the Articulated Manipulator" width = 600 title="Discussion of PATH AND TRAJECTORY (PYTHON)">

</a> 
<h4 align="center">Supplementary Video about the PATH AND TRAJECTORY (GUI CALCULATOR)
<h5 align="center">To understand PATH AND TRAJECTORY (GUI CALCULATOR) of an Articulated Manipulator, here is a supplementary video explaining how to get it.  
</p> 
<a href="https://drive.google.com/file/d/1lE7TUD0LTbFND6-xToHCPibqJkUJSEIE/view?usp=sharing">
    <source media="(prefers-color-scheme: dark)" srcset=https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/3df9627f-f75b-41ab-886a-06e3b5ff1f1e>
    <img src="https://github.com/stnll/Robotics2_JacobianandPT_Group13_Articulated_2024/assets/157665975/a77dd704-065c-4754-aa49-12be80ee2054" alt="Discussion of Singularity for the Articulated Manipulator" width = 600 title="Discussion of PATH AND TRAJECTORY (GUI CALCULATOR)">


    





 

  
  


 

 






