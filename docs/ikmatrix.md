# 📚 Inverse Kinematics and Jacobian

> In this assignment I solved the Inverse Kinematics and calculated the Jacobian matrix for a 3-DOF articulated robotic arm.

---

## 1) Summary

- **Homework Name:** Inverse Kinematics and Jacobian
- **Author:** Angel Ivan Dominguez Cruz 
- **Subject:** Applied Robotics
- **Date:** 10/03/2026 

---

## 2) Objectives

- **General:** Analyze a 3-DOF articulated robotic mechanism by applying geometric methods to determine the Inverse Kinematics and computing its Jacobian matrix to relate joint velocities to the end effector's linear velocities.

![Robot Diagram](recursos/imgs/ikmatrix/excercise.png)

---

## 3) 3-DOF Articulated Robot Excercise

- **Explanation:** This exercise focuses on a 3-Degree-of-Freedom (DOF) articulated robotic arm. First, the forward kinematics transformation matrix was established. Then, a geometric approach was used to find the inverse kinematics equations for $\theta_1, \theta_2$, and $\theta_3$. Finally, the Jacobian matrix was computed by differentiating the position vector equations with respect to each joint variable ($q_1, q_2, q_3$).

### Forward Kinematics Matrices
Before jumping into the inverse kinematics, the position equations from the transformation matrix are needed. First, the individual transformation matrices for each link ($A_1, A_2, A_3$) were defined based on the Denavit-Hartenberg parameters:

**Link 1 Transformation Matrix ($A_1$):**
![Matrix A1](recursos/imgs/ikmatrix/matrix_a1.png)

**Link 2 Transformation Matrix ($A_2$):**
![Matrix A2](recursos/imgs/ikmatrix/matrix_a2.png)

**Link 3 Transformation Matrix ($A_3$):**
![Matrix A3](recursos/imgs/ikmatrix/matrix_a3.png)

By multiplying these individual matrices ($T_0^3 = A_1 \cdot A_2 \cdot A_3$), the complete forward kinematics transformation matrix is obtained:

![FK Matrix](recursos/imgs/ikmatrix/fk_matrix.png)

### Inverse Kinematics Solution
Using geometric analysis (trigonometry and the law of cosines), the joint angles are solved based on a given end-effector position (x, y, z):
![Inverse Kinematics Calculation](recursos/imgs/ikmatrix/solve_ik.png)

### Jacobian Matrix
The analytical Jacobian matrix was obtained by calculating the partial derivatives of the X, Y, and Z position equations with respect to the joint angles $q_1, q_2$, and $q_3$:
![Jacobian Matrix Calculation](recursos/imgs/ikmatrix/solve_jacobian.png)

---