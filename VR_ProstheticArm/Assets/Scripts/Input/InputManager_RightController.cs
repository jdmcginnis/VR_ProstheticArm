using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BearLab
{

    // Takes input from VR controller and stores it in InputContainer
    public class InputManager_RightController : MonoBehaviour
    {
        [SerializeField] private InputContainer inputContainer;

        private PlayerInput playerInput;

        private void OnEnable()
        {
            if (playerInput == null)
            {
                playerInput = new PlayerInput();
                playerInput.XRIRightHand.Position.performed += i => inputContainer.armInputPosition = i.ReadValue<Vector3>();
                playerInput.XRIRightHand.Rotation.performed += i => inputContainer.armInputOrientation = i.ReadValue<Quaternion>();
                playerInput.XRIRightHandInteraction.Select.performed += i => inputContainer.openGrasp = true;
                playerInput.XRIRightHandInteraction.Select.canceled += i => inputContainer.openGrasp = false;
                playerInput.XRIRightHandInteraction.Activate.performed += i => inputContainer.closeGrasp = true;
                playerInput.XRIRightHandInteraction.Activate.canceled += i => inputContainer.closeGrasp = false;
            }

            playerInput.Enable();
        }

        private void OnDisable()
        {
            playerInput.Disable();
        }
    }
}
