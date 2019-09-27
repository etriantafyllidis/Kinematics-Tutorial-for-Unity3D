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
Forward Kinematics with Rotation Axis constraint (can be configured to 3DOF)
=======================================================================================
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForwardKinematics : MonoBehaviour
{
    [Header("Kinematic Joints")]
    public Transform[] KinematicJoints; // The joints of the kinematic chain
    public float[] AngleX, AngleY, AngleZ; // The angle of the i-th joint
    public Vector3 RotationAxis; //  Controls the rotation axis i.e. DOF of all joints

    void Start()
    {
        AngleX = new float[KinematicJoints.Length];
        AngleY = new float[KinematicJoints.Length];
        AngleZ = new float[KinematicJoints.Length];

        StartOffset();
    }

    void Update()
    {
        ForwardKinematics_();
    }

    // The starting offset of rotation of the humanoid
    void StartOffset()
    {
        for (int i = 0; i < KinematicJoints.Length; i++)
        {
            AngleX[i] = KinematicJoints[i].localRotation.eulerAngles.x;
            AngleY[i] = KinematicJoints[i].localRotation.eulerAngles.y;
            AngleZ[i] = KinematicJoints[i].localRotation.eulerAngles.z;
        }
    }

    // FK with the option of Rotation Axis Constraints, if RotationAxis = 1.0, 1.0, 1.0 => 3DOF
    void ForwardKinematics_()
    {
        for (int i = 0; i < KinematicJoints.Length; i++)
        {   
            KinematicJoints[i].rotation = KinematicJoints[i].parent.rotation 
                * Quaternion.AngleAxis(AngleX[i], new Vector3(RotationAxis.x, 0.0f, 0.0f))
                * Quaternion.AngleAxis(AngleY[i], new Vector3(0.0f, RotationAxis.y, 0.0f))
                * Quaternion.AngleAxis(AngleZ[i], new Vector3(0.0f, 0.0f, RotationAxis.z));
        }
    }
}
