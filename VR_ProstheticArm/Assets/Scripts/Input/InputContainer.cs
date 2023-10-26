using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BearLab
{

    // Handles all input from various input devices (VR Controller or Delsys)
    public class InputContainer : MonoBehaviour
    {
        // Translation & Rotation
        public Vector3 armInputPosition;
        public Quaternion armInputOrientation;

        // Grasp Flags
        public bool openGrasp = false;
        public bool closeGrasp = false;

    }
}