using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BearLab
{
    public class SPDController : MonoBehaviour
    {
        // Note: Controller is stable if gain.d >= gain.p * dt
        [System.Serializable]
        public class Gain
        {
            public float p = 0f;
            public float d = 0f;
        }

        public float mass;
        [SerializeField] private Gain posGain = new Gain();
        [SerializeField] private Gain rotGain = new Gain();

        // [SerializeField] Rigidbody objectRB;
        [SerializeField] ArticulationBody objectAB;
        private Quaternion inertiaTensorRotation;
        private Vector3 inertiaTensor;


        private void Start()
        {
            // mass = this.GetComponent<Rigidbody>().mass;
            // inertiaTensorRotation = objectRB.inertiaTensorRotation;
            // inertiaTensor = objectRB.inertiaTensor;

            mass = objectAB.mass;
            inertiaTensorRotation = objectAB.inertiaTensorRotation;
            inertiaTensor = objectAB.inertiaTensor;
        }

        // See page 23 of lab notebook for equation derivation
        public Vector3 ComputeSPDForce(Vector3 currentPos, Vector3 currentVel, Vector3 targetPos)
        {
            Vector3 acceleration = ComputeSPDLinearAcceleration(currentPos, currentVel, targetPos);
            Vector3 forceToApply = mass * acceleration;
            return forceToApply;
        }

        private Vector3 ComputeSPDLinearAcceleration(Vector3 currentPos, Vector3 currentVel, Vector3 targetPos)
        {

            float denominator = mass + (posGain.d * Time.fixedDeltaTime);
            Vector3 numerator = -posGain.p * (currentPos + (currentVel * Time.fixedDeltaTime) - targetPos) - (posGain.d * currentVel);

            Vector3 acceleration = numerator / denominator;
            return acceleration;

        }




        public Vector3 ComputeSPDTorque(Quaternion currentOrientation, Vector3 currentAngVel, Quaternion targetOrientation)
        {

            Vector3 angularAcc = ComputeSPDAngularAcceleration(currentOrientation, currentAngVel, targetOrientation);

            Quaternion rotInertia2World = objectAB.inertiaTensorRotation * currentOrientation;
            Vector3 torque = Quaternion.Inverse(rotInertia2World) * angularAcc;
            torque.Scale(objectAB.inertiaTensor);
            torque = rotInertia2World * torque;

            Debug.Log("Angular Acceleration: " + angularAcc);
            Debug.Log("Torque Being Applied: " + torque);

            return torque;
        }





        // currentAngVel is in rad/s
        // private Vector3 ComputeSPDAngularAcceleration(Quaternion currentOrientation, Vector3 currentAngVel, Quaternion targetOrientation)
        private Vector3 ComputeSPDAngularAcceleration (Quaternion wld_Rot_p0, Vector3 v0_wld, Quaternion wld_RotTarg_p0)
        {
            Vector3 vTarg0_wld = v0_wld;
            float dt = Time.fixedDeltaTime;
            float kp = rotGain.p;
            float kd = rotGain.d;

            Quaternion wld_RotTarg_p1;  // =  pTarg0 + dt * vTarg0   expected body rotation one frame in the future.
            {
                float omegaTarg_rad = vTarg0_wld.magnitude;
                if (omegaTarg_rad < 10e-8f)
                {
                    wld_RotTarg_p1 = wld_RotTarg_p0;
                } else
                {
                    Quaternion wld_delta_wld = Quaternion.AngleAxis(omegaTarg_rad * dt * Mathf.Rad2Deg,
                                                            vTarg0_wld / omegaTarg_rad);
                    wld_RotTarg_p1 = wld_delta_wld * wld_RotTarg_p0;
                }
            }

            Quaternion wld_Rot_p1; // = p0 + _dt * v0;   expected target rotation one frame in the future.
            {
                float omega = v0_wld.magnitude;
                if (omega < 10e-8f)
                {
                    wld_Rot_p1 = wld_Rot_p0;
                } else
                {
                    Quaternion wld_delta_wld = Quaternion.AngleAxis(omega * dt * Mathf.Rad2Deg,
                                                            v0_wld / omega);
                    wld_Rot_p1 = wld_delta_wld * wld_Rot_p0;
                }
            }

            float p_deltaMag; // p1 - pTarg1    difference between current angle and target angle
            Vector3 p_deltaDir_wld;
            {
                Quaternion wld_deltaP_wld = wld_Rot_p1 * Quaternion.Inverse(wld_RotTarg_p1);

                if (wld_deltaP_wld.w < 0f)
                {
                    // The rotation is greater than 180 degrees (long way around the sphere). Convert to shortest path.
                    wld_deltaP_wld.x = -wld_deltaP_wld.x;
                    wld_deltaP_wld.y = -wld_deltaP_wld.y;
                    wld_deltaP_wld.z = -wld_deltaP_wld.z;
                    wld_deltaP_wld.w = -wld_deltaP_wld.w;
                }

                wld_deltaP_wld.ToAngleAxis(out p_deltaMag, out p_deltaDir_wld);
                p_deltaDir_wld.Normalize();
                p_deltaMag *= Mathf.Deg2Rad;
            }




            Vector3 a = -kp * p_deltaDir_wld * p_deltaMag - kd * (v0_wld);
            return a;

        }




        // If rotation is greater than 180deg (long way round), convert to the shortest path
        private Quaternion OptimizeQuaternion(Quaternion orientation)
        {
            if (orientation.w < 0f)
            {
                orientation.x = -orientation.x;
                orientation.y = -orientation.y;
                orientation.z = -orientation.z;
                orientation.w = -orientation.w;
            }

            return orientation;
        }

    }
}
