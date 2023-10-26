using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BearLab
{

    public class ArmLocomotionManager : MonoBehaviour
    {
        // [SerializeField] private ArticulationBody armPivotPointAB;
        [SerializeField] private SPDController spdController;
        [SerializeField] private InputContainer inputContainer;

        // [SerializeField] private Rigidbody rigidBody;
        [SerializeField] private ArticulationBody rigidBody;

        // Offsets Logic
        public bool initialFrame = true;
        public Vector3 initInputPosOffset; // positional offset of input controller
        public Vector3 initArmPosOffset; // positional offset of model
        private Quaternion initInputOrientation; // rotational offset of input controller
        private Quaternion initArmOrientation; // rotational offset of model

        private void FixedUpdate()
        {
            if (initialFrame)
                SetInitialOffsets();

            Vector3 desiredPos = FetchDesiredPosition();
            Quaternion desiredOrientation = FetchDesiredOrientation();

            Vector3 forceToApply = spdController.ComputeSPDForce(rigidBody.transform.position, rigidBody.velocity, desiredPos);
            Vector3 torqueToApply = spdController.ComputeSPDTorque(rigidBody.transform.rotation, rigidBody.angularVelocity, desiredOrientation);

            rigidBody.AddForce(forceToApply);
            rigidBody.AddTorque(torqueToApply);

        }

        private Vector3 FetchDesiredPosition()
        {
            return initArmPosOffset + (inputContainer.armInputPosition - initInputPosOffset);
        }


        private Quaternion FetchDesiredOrientation()
        {

            return inputContainer.armInputOrientation * initArmOrientation;
        }


        // Offsets are read on first frame only
        // Read the initial offsets so we may account for them
        private void SetInitialOffsets()
        {
            // Position
            initInputPosOffset = inputContainer.armInputPosition;
            initArmPosOffset = this.transform.position;

            //Orientation
            initInputOrientation = inputContainer.armInputOrientation;
            initArmOrientation = this.transform.rotation;

            initialFrame = false;
            return;
        }
    }
}