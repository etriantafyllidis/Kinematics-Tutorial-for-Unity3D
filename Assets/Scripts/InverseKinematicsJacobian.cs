/*
Author: Eleftherios Triantafyllidis (eleftherios.triantafyllidis@ed.ac.uk)

DISCLAIMER: This code was written to assist users/programmers and generally those
willing to learn about inverse kinematics. Please be aware that this is work in
progress and by extent may certainly have bugs and issues. You are welcome to 
report any issues and I will aim to update these regularly. Code has multiple
comments hoping to assist readers. You are more than welcome to copy any parts or 
use this code directly to your project. Finally, I would appreciate crediting the work.

====================================  License  ========================================
Copyright (c) <2019> <Eleftherios Triantafyllidis>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
====================================  Usage  ==========================================
Use of Jacobian Inverse Kinematics
Three methods are used:
1) Jacobian Transpose
2) Jacobian Pseudo Inverse
3) Jacobian Damped Least Squares
These can be selected in the dropdown menu of the script in the inspector

Additional Feature: 
Rotation constraint in both angle and axis. :)

Current Limitations: 
Globaly sets the degree of freedom i.e. the script applys a rotational
constraint in either the X, Y or/and Z axis not on the individual joint but globally
i.e. all the joints of the specified kinematic chain. 
In a "real" scenario, some joints have more/less degrees of freedom.
=======================================================================================
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InverseKinematicsJacobian : MonoBehaviour
{
    [Header("Kinematic Joints")]
    public Transform[] KinematicJoints; // The joints of the kinematic chain

    [Header("Target to Reach")]
    public Transform Target; // The target position

    public enum JacobianMethod { UseJacobianTranspose,
                                 UseJacobianPseudoInverse,
                                 UseJacobianDampedLeastSquares};
    [Header("Jacobian Method")]
    public JacobianMethod _JacobianMethod; // Enum Attribute to specify which method to choose

    [Header("Joint Angle Limits")]
    [Range(0.0f, 360.0f)]
    public float JointsAngleLimit; // The rotational limit of all joints

    [Header("Rotation Axis (DOF)")]
    public Vector3 GlobalRotationAxisConstraint; // The "allowable" axis the joints can rotate 
    // If this vector is equal to (0,1,1) then all joints can rotate in the Y and Z but NOT in X axis. 

    [Header("IK Settings")]
    public float EPS; // The "acceptable" distance between the end-effector and the target
    public float SimulationStep; // The "speed" of the simulation
    public float LDamping; // Damping Parameter

    private float[] Angles; // The current angle of the i-th joint
    private Vector4[] AnglesVector; // The vector containing the angles of all joints (array = 3 axes)
    private Vector4 TargetGoalVector; // Same as below only as a Vector4 to multiply with matrix
    private Vector3 TargetGoal; // The vector between the target position and the end-effector
    private Vector3[] VectorJoint; // The vector of each joint in the kinematic chain (array = 3 axes)
    private Matrix4x4 Jacobian; // The Jacobian Matrix

    void Start()
    {
        VectorJoint = new Vector3[KinematicJoints.Length];
        Angles = new float[KinematicJoints.Length*3]; //0:X i=0, 1:Y i=0, 2:Z i=0, 3:X i=1, 4:Y i=1 etc...
        AnglesVector = new Vector4[3]; // For the three axes
    }

    void Update()
    {
        if (Vector3.Distance(KinematicJoints[KinematicJoints.Length - 1].position, Target.position) > EPS)
        {  
            // X = 0, Y = 1, Z = 2
            if (GlobalRotationAxisConstraint.x != 0)
            {
                // Creates the Jacobian Matrix
                JacobianMatrix(0);

                // Selects the Jacobian Method to use
                JacobianOption(0);

                // Applys the IK i.e. the required Angles of all joints
                ApplyInverseKinematics(0);
            }
            
            if (GlobalRotationAxisConstraint.y != 0)
            {
                // Creates the Jacobian Matrix
                JacobianMatrix(1);

                // Selects the Jacobian Method to use
                JacobianOption(1);

                // Applys the IK i.e. the required Angles of all joints
                ApplyInverseKinematics(1);
            }

            if (GlobalRotationAxisConstraint.z != 0)
            {
                // Creates the Jacobian Matrix
                JacobianMatrix(2);

                // Selects the Jacobian Method to use
                JacobianOption(2);

                // Applys the IK i.e. the required Angles of all joints
                ApplyInverseKinematics(2);
            }
        }
    }

    Matrix4x4 JacobianTranspose()
    {
        // ==========================================
        // Based on the equation:
        // ==========================================
        // JT = J^T * I * S
        // JT: Jacobian Transpose
        // J^T: The transpose of the Jacobian Matrix
        // I: Identity Matrix
        // S: Simulation Steop
        // ==========================================

        Matrix4x4 JT = new Matrix4x4();
        Matrix4x4 I = Matrix4x4.identity; // Identity matrix * simulation step
        I.m00 *= SimulationStep;
        I.m11 *= SimulationStep;
        I.m22 *= SimulationStep;
        I.m33 *= SimulationStep;
        JT = I * Jacobian.transpose;
        return JT;
    }

    Matrix4x4 JacobianPseudoInverse()
    {
        // ==========================================
        // Based on the equation:
        // ==========================================
        // JPI = ((JT * J) + I * S) ^-1
        // JPI: Jacombian Pseudo Inverse
        // JT: Jacobian Transpose 
        // J: Jacobian Matrix
        // I: Identity Matrix
        // S: Simulation Step
        // ^-1: The inverse of that **
        // ==========================================

        // Jacobian Pseudo Inverse
        Matrix4x4 JPI = new Matrix4x4();

        // Identity matrix * simulation step
        Matrix4x4 I = Matrix4x4.identity;
        I.m00 *= SimulationStep;
        I.m11 *= SimulationStep;
        I.m22 *= SimulationStep;
        I.m33 *= SimulationStep;

        Matrix4x4 JT = Jacobian.transpose;
        Matrix4x4 JI = ((JT * Jacobian));

        // Matrix addition of the Jacobian Inverse with the identity matrix 
        JI.m00 += I.m00; JI.m10 += I.m10; JI.m20 += I.m20; JI.m30 += I.m30;
        JI.m01 += I.m01; JI.m11 += I.m11; JI.m21 += I.m21; JI.m31 += I.m31;
        JI.m02 += I.m02; JI.m12 += I.m12; JI.m22 += I.m22; JI.m32 += I.m32;
        JI.m03 += I.m03; JI.m13 += I.m13; JI.m23 += I.m23; JI.m33 += I.m33;

        JI = JI.inverse;
        JPI = JI * JT;
        return JPI;
    }

    Matrix4x4 JacobianDampedLeastSquares()
    {
        // ==========================================
        // Based on the equation:
        // ==========================================
        // JDLS = JT * (J * JT + L^2 * I * S) ^-1
        // JDLS: Jacombian Damped Least Squares
        // JT: Jacobian Transpose 
        // J: Jacobian Matrix
        // L^2: Damping Coefficient (squared)
        // I: Identity Matrix
        // S: Simulation Step
        // ^-1: The inverse of that **
        // ==========================================

        // Identity matrix * simulation step * squared damping coefficient
        Matrix4x4 I = Matrix4x4.identity;
        I.m00 *= SimulationStep * Mathf.Pow(LDamping, 2);
        I.m11 *= SimulationStep * Mathf.Pow(LDamping, 2);
        I.m22 *= SimulationStep * Mathf.Pow(LDamping, 2);
        I.m33 *= SimulationStep * Mathf.Pow(LDamping, 2);

        // Jacombian Damped Least Squares
        Matrix4x4 JDLS = new Matrix4x4();

        Matrix4x4 JT = JacobianTranspose();
        JDLS = Jacobian * JT;

        JDLS.m00 += I.m00; JDLS.m10 += I.m10; JDLS.m20 += I.m20; JDLS.m30 += I.m30;
        JDLS.m01 += I.m01; JDLS.m11 += I.m11; JDLS.m21 += I.m21; JDLS.m31 += I.m31;
        JDLS.m02 += I.m02; JDLS.m12 += I.m12; JDLS.m22 += I.m22; JDLS.m32 += I.m32;
        JDLS.m03 += I.m03; JDLS.m13 += I.m13; JDLS.m23 += I.m23; JDLS.m33 += I.m33;

        JDLS = JDLS.inverse;
        JDLS = JDLS * JT;
        return JDLS;
    }

    Matrix4x4 JacobianMatrix(int Axis)
    {
        // The target goal consisting between the vectors target and end-effector
        TargetGoal = Target.position - KinematicJoints[KinematicJoints.Length - 1].position;
        TargetGoalVector = new Vector4(TargetGoal.x, TargetGoal.y, TargetGoal.z, 1.0f);

        switch (Axis)
        {
            case 0: // X Axis
                for (int i = 0; i < KinematicJoints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(KinematicJoints[i].right,
                        (KinematicJoints[KinematicJoints.Length - 1].position - KinematicJoints[i].position));
                }
                break;
            case 1: // Y Axis
                for (int i = 0; i < KinematicJoints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(KinematicJoints[i].up,
                        (KinematicJoints[KinematicJoints.Length - 1].position - KinematicJoints[i].position));
                }
                break;
            case 2: // Z Axis
                for (int i = 0; i < KinematicJoints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(KinematicJoints[i].forward,
                        (KinematicJoints[KinematicJoints.Length - 1].position - KinematicJoints[i].position));
                }
                break;
        }

        for (int i = 0; i < KinematicJoints.Length - 1; i++)
        {
            Jacobian.SetColumn(i, new Vector4(VectorJoint[i].x, VectorJoint[i].y, VectorJoint[i].z, 0.0f));
        }
        Jacobian.SetColumn(KinematicJoints.Length - 1, new Vector4(0.0F, 0.0f, 0.0f, 1.0f));
        return Jacobian;
    }

    void JacobianOption (int Axis)
    {
        // The three methods of the Jacobian
        if (_JacobianMethod == JacobianMethod.UseJacobianTranspose)
        {
            AnglesVector[Axis] = JacobianTranspose() * TargetGoalVector;
        }
        else if (_JacobianMethod == JacobianMethod.UseJacobianPseudoInverse)
        {
            AnglesVector[Axis] = JacobianPseudoInverse() * TargetGoalVector;
        }
        else if (_JacobianMethod == JacobianMethod.UseJacobianDampedLeastSquares)
        {
            AnglesVector[Axis] = JacobianDampedLeastSquares() * TargetGoalVector;
        }
    }

    void ApplyInverseKinematics(int Axis)
    {
        for (int i=0; i < KinematicJoints.Length - 1; i ++)
        {
            Angles[i] += AnglesVector[Axis][i];
            Angles[i] %= 360;
            if (Mathf.Abs(Angles[i]) > 180.0f)
            {
                Angles[i] -= 360 * Mathf.Sign(Angles[i]);
            }

            // if angle required is larger than limit, apply maximum allowable limit
            if (Mathf.Abs(Angles[i]) > Mathf.Abs(JointsAngleLimit))
            {
                Angles[i] = 180.0f * Mathf.Sign(Angles[i]);
            }
            switch (Axis)
            {
                case 0:
                    KinematicJoints[i].localEulerAngles = new Vector3(Angles[i],
                                                                      KinematicJoints[i].localEulerAngles.y,
                                                                      KinematicJoints[i].localEulerAngles.z);
                    break;
                case 1:
                    KinematicJoints[i].localEulerAngles = new Vector3(KinematicJoints[i].localEulerAngles.x,
                                                                      Angles[i],
                                                                      KinematicJoints[i].localEulerAngles.z);
                    break;
                case 2:
                    KinematicJoints[i].localEulerAngles = new Vector3(KinematicJoints[i].localEulerAngles.x,
                                                                      KinematicJoints[i].localEulerAngles.y,
                                                                      Angles[i]);
                    break;
            }

        }

    }

}
