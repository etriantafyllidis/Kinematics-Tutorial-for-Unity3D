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
Simple Inverse Kinematics using CCD (Cyclic-Coordinate-Descent)
Does NOT take into account limits, 3-DOF on every joint/bone.
=======================================================================================
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InverseKinematicsCCD : MonoBehaviour
{
    [Header("Kinematic Joints")]
    public Transform[] KinematicJoints; // The kinematic chain containing all joints
    public Transform Target; // The target to reach

    void Update()
    {
        CCDIK();
    }

    // Cyclic-Coordinate-Descent 
    void CCDIK ()
    {
        // Iteration from end-effector to root in the kinematic chain
        for (int i = KinematicJoints.Length - 1; i >= 0; i--)
        {
            Matrix4x4 InverseTransformMatrix = KinematicJoints[i].localToWorldMatrix.inverse;            
            Vector3 EndEffectorDirection = Matrix4x4_Mult_Translation(KinematicJoints[KinematicJoints.Length - 1].position, InverseTransformMatrix).normalized;
            Vector3 TargetDirection = Matrix4x4_Mult_Translation(Target.position, InverseTransformMatrix).normalized;

            float DotProduct = Vector3.Dot(EndEffectorDirection, TargetDirection);
            if (DotProduct < 1.0f - 1.0e-6f)
            { 
                float RotationAngle = Mathf.Acos(DotProduct) * Mathf.Rad2Deg;
                Vector3 RotationAxis = Vector3.Cross(EndEffectorDirection, TargetDirection).normalized;
                KinematicJoints[i].Rotate(RotationAxis, RotationAngle);
            }
        }
    }

    // Multiply a 4x4Matrix with a Vector3 (position i.e. translation)
    Vector3 Matrix4x4_Mult_Translation(Vector3 Translation, Matrix4x4 TransformationMatrix)
    {
        Vector4 TempV4 = new Vector4(Translation.x, Translation.y, Translation.z, 1);
        TempV4 = TransformationMatrix * TempV4;
        Translation = new Vector3(TempV4.x, TempV4.y, TempV4.z);

        return Translation;
    }
}
