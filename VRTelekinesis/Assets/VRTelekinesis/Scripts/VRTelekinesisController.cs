using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VRTelekinesisController : MonoBehaviour {

    [SerializeField]
    [Tooltip("Layers to exclude from the raycast.")]
    private LayerMask m_ExclusionLayers;
    [SerializeField]
    [Tooltip("Max Ray Distance.")]
    private float m_MaxDistance = 100;
    [SerializeField]
    [Tooltip("Reticle Transform.")]
    private Transform m_Reticle;
    [SerializeField]
    [Range(.05f, 15f)]
    [Tooltip("Amount of 'drag' the object has when moving it around")]
    private float m_FollowSpeed = 5;
    [SerializeField]
    [Range(.05f, .2f)]
    [Tooltip("How quickly the object pushes/pulls when using the depth axis")]
    private float m_DepthSpeed = .1f;
    [SerializeField]
    [Tooltip("Allow Rotation")]
    private bool m_AllowRotate = true;
    [Header("Magic Beam")]
    [SerializeField]
    [Tooltip("Particle system to use for the magic beam")]
    private GameObject m_PathParticle;
    [SerializeField]
    [Tooltip("Magic beam density")]
    private int m_PathParticleCount = 50;
    [SerializeField]
    [Range(0f, 1f)]
    [Tooltip("How much force is applied when the object is released. Used for throwing. A value of zero is 'natural.'")]
    private float m_ThrowForce = .15f;
    [SerializeField]
    [Tooltip("Will attempt to sibling each of these GameObjects in order, skipping them if disabled.")]
    private ControllerRef[] m_ControllerPreferences;


    public event Action<VRTelekinesisObject> OnAttach;             // Called when the object is grabbed 
    public event Action<VRTelekinesisObject> OnDetach;             // Called when the object is released

    private ControllerRef m_CurrentController;
    private bool m_IsTelekinesising;
    private VRTelekinesisObject m_ActiveObject;
    private float m_fDistance;
    private Vector3 m_DropVel;
    private bool m_OriginalGravity;
    private Vector3[] m_MagicBeamPoints;
    private GameObject m_ParticleHolder;
    private Queue<Vector3> lastPositionQueue = new Queue<Vector3>();
    private int maxQueueSize = 10;
    private float m_InitialDrag;
    private float m_InitialAngularDrag;
    private LineRenderer m_Line = null;
    private float m_LastControllerAngle;

    private void Start()
    {
        // Create magic beam point array for curve.
        m_MagicBeamPoints = new Vector3[3];

        // Make sure there's a place available for this to go if controllers aren't found. This is typically a child of the HMD/Camera.
        // If not, log an error.
        if (!m_ControllerPreferences[m_ControllerPreferences.Length - 1].siblingGO.activeSelf)
            Debug.LogError("Telekinesis: Last Controller Preference GameObject must always be enabled, as a viable fallback.");

        // Set a default controller.
        m_CurrentController = m_ControllerPreferences[0];

        // Assign the line renderer if we have one.
        if (GetComponent<LineRenderer>())
            m_Line = GetComponent<LineRenderer>();
    }
    void Update()
    {
        // Keep this raycaster positionally mapped to the active controller transfrom
        transform.position = m_CurrentController.siblingGO.transform.position;
        transform.rotation = m_CurrentController.siblingGO.transform.rotation;

        // If we have a line, set it's positions
        if (m_Line != null)
        {
            m_Line.positionCount = 2;
            m_Line.SetPosition(0, transform.position);
            m_Line.SetPosition(1, transform.position + transform.forward * 1f);
        }

        // If we have a reticle, position it forward from the controller.
        if (m_Reticle != null)
        {
            m_Reticle.position = transform.position + transform.forward * 5f;
            m_Reticle.rotation = Quaternion.FromToRotation(Vector3.forward, transform.position - (transform.position + transform.forward * 1f));
        }

        // See if we can get a preferred controller, or fall back if our current controller is disconnected.
        for (int i = 0; i < m_ControllerPreferences.Length; i++)
        {
            if (m_ControllerPreferences[i].siblingGO.activeSelf)
            {
                if (m_ControllerPreferences[i] != m_CurrentController)
                {
                    // Drop object if controller is lost.
                    Detach();
                    // Set new current controller
                    m_CurrentController = m_ControllerPreferences[i];
                }
                break;
            }
        }

        // If we're "telekinesising" do this stuff to move the object.
        if (m_IsTelekinesising)
        {
            // Check touchpad Y value to see if we should push or pull the object.
            if (m_CurrentController.GetY() > .5f)
            {
                m_fDistance += m_DepthSpeed;
            }
            else if (m_CurrentController.GetY() < -.5f)
            {
                m_fDistance -= m_DepthSpeed;
            }

            // Move in relation to the controller.
            // Get the rigid body.
            Rigidbody rigidBody = m_ActiveObject.gameObject.GetComponent<Rigidbody>();
            // Get the point directly in front of the controller, at the distance of the object.
            Vector3 targetPos = (transform.position + (transform.forward * m_fDistance));
            // Find out how far the object is from the target pos. This will contribute to how much force is applied.
            float travelDistance = Vector3.Distance(targetPos, m_ActiveObject.transform.position);
   
            // Adjust drag to aid in natural stopping/recoil, based on distance to travel.
            rigidBody.drag = Remap(Mathf.Min(travelDistance, .1f), m_InitialDrag, 5, 5, m_InitialDrag);
            // Add force in the direction we need to travel.
            rigidBody.AddForce((targetPos - m_ActiveObject.transform.position) * (travelDistance * m_FollowSpeed));

            // If we want to allow rotating the object...
            if (m_AllowRotate)
            {
                // Get the controller Z angle
                float RotateZAmount = Mathf.DeltaAngle(m_LastControllerAngle, transform.eulerAngles.z);
                // Rotate the object by the same amount
                m_ActiveObject.transform.Rotate(transform.forward, RotateZAmount, Space.World);
                // Record last applied controller angle
                m_LastControllerAngle = transform.eulerAngles.z;

                // If the user is touching the far left/right of the touchpad, rotate on the up axis in the appropriate direction
                if (m_CurrentController.GetX() > .5f || m_CurrentController.GetX() < -.5f)
                {
                    float RotateXAmount = m_CurrentController.GetX() * 90f;
                    m_ActiveObject.transform.Rotate(-transform.up, RotateXAmount * Time.deltaTime, Space.World);
                }
            }

            // Set up the origin, influence and destination for the Magic Beam curve.
            m_MagicBeamPoints[0] = transform.position;
            m_MagicBeamPoints[1] = transform.position + (transform.forward * Vector3.Distance(m_ActiveObject.transform.position, transform.position) / 2);
            m_MagicBeamPoints[2] = m_ActiveObject.transform.position;

            // Get the curve
            Vector3[] Curve = QuadraticBezierCurve(m_MagicBeamPoints[0], m_MagicBeamPoints[1], m_MagicBeamPoints[2], m_PathParticleCount);

            // Place a the magic beam particle on each point of the curve.
            int i = 0;
            foreach (Vector3 point in Curve)
            {
                Transform part = m_ParticleHolder.transform.GetChild(i);
                part.position = point;
                float scaleFactor = Remap(Mathf.Min(Vector3.Distance(m_ActiveObject.transform.position, transform.position), 10), 0, 10, .3f, .6f);
                part.localScale = ((Vector3.one / m_PathParticleCount) * i) * scaleFactor;
                part.LookAt(transform.transform);
                i++;
            }
            // If we detect that the button is up while we're telekinesising, drop the object.
            if (!m_CurrentController.GetButton() && !Input.GetMouseButton(0))
            {
                Detach();
            }
        }
        // If we're not "Telekinesising"...
        else
        {
            // Cast a ray forward to see what we hit.
            Ray ray = new Ray(transform.position, transform.forward);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, m_MaxDistance, ~m_ExclusionLayers))
            {
                // If we have a reticle, Position/Angle it just above the surface it hit.
                if (m_Reticle != null)
                {
                    m_Reticle.position = hit.point;
                    m_Reticle.rotation = Quaternion.FromToRotation(Vector3.forward, hit.normal);
                }
                // If the object we're pointing at is a TelekinesisObject and we've detected a button down (while not already telekinesising), pick it up.
                if ((m_CurrentController.GetButton() || Input.GetMouseButton(0)) && hit.collider.gameObject.GetComponent<VRTelekinesisObject>())
                {
                    Attach(hit.collider.gameObject.GetComponent<VRTelekinesisObject>());
                }
            }
        }

        // Record what direction to throw the object when we detach.
        m_DropVel = (transform.position - GetMeanVector(lastPositionQueue)) / Time.deltaTime;

        // Update a queue of positions to average to get average velocity
        if (lastPositionQueue.Count >= maxQueueSize)
        {
            lastPositionQueue.Dequeue();
        }
        lastPositionQueue.Enqueue(transform.position);
    }

    // Grab the object
    void Attach(VRTelekinesisObject obj)
    {
        if (!m_IsTelekinesising)
        {
            // Kill the line/reticle while telekinesising
            if (m_Line != null)
                m_Line.enabled = false;
            m_Reticle.GetComponentInChildren<MeshRenderer>().enabled = false;

            // Record that we're actively controlling the object.
            m_IsTelekinesising = true;

            // Update the active object to the one we clicked on.
            m_ActiveObject = obj;

            // Create a game object to hold all of the Magic Beam particles
            m_ParticleHolder = new GameObject();
            m_ParticleHolder.transform.parent = transform;
            for (int i = 0; i < m_PathParticleCount; i++)
            {
                GameObject go = Instantiate(m_PathParticle);
                go.transform.parent = m_ParticleHolder.transform;
            }
            // Enable each particle
            ParticleSetEnabled(true);
            
            // Get the object's rigid body
            Rigidbody rigidBody = m_ActiveObject.gameObject.GetComponent<Rigidbody>();
            // Remember it's drag/gravity values for restoration later.
            m_InitialDrag = rigidBody.drag;
            m_InitialAngularDrag = rigidBody.angularDrag;
            m_OriginalGravity = rigidBody.useGravity;
            // Set Angular Drag to huge amount, making the object initially stationary.
            rigidBody.angularDrag = float.MaxValue;
            // Disable gravity so we don't have to fight it while moving the object through the air.
            rigidBody.useGravity = false;
            // Set collision detection to continuous to avoid going through other objects.
            rigidBody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            // Record the initial distance between the user and the object grabbed.
            m_fDistance = Vector3.Distance(m_ActiveObject.transform.position, transform.position);

            // Notify subscribers of the new object.
            if (OnAttach != null)
                OnAttach(m_ActiveObject);
        }
    }

    // Drop the object.
    void Detach()
    {
        if (m_IsTelekinesising)
        {
            // Enable the line while not telekinesising
            if (m_Line != null)
                m_Line.enabled = true;
            m_Reticle.GetComponentInChildren<MeshRenderer>().enabled = true;

            // Record that we're done.
            m_IsTelekinesising = false;

            // Restore all of the rigidbody values (as recorded in Attach())
            Rigidbody rigidBody = m_ActiveObject.gameObject.GetComponent<Rigidbody>();
            rigidBody.drag = m_InitialDrag;
            rigidBody.angularDrag = m_InitialAngularDrag;
            rigidBody.useGravity = m_OriginalGravity;
            rigidBody.collisionDetectionMode = CollisionDetectionMode.Discrete;
            m_fDistance = 0f;

            // Add force to allow enhanced "throw" feel.
            rigidBody.AddForce(m_DropVel.x * m_ThrowForce, m_DropVel.y * m_ThrowForce, m_DropVel.z * m_ThrowForce, ForceMode.Impulse);

            // Disable all of the particles and destroy their parent.
            ParticleSetEnabled(false);
            Destroy(m_ParticleHolder);

            // Notify subscribers of the drop
            if (OnDetach != null)
                OnDetach(m_ActiveObject);

            // Clear Active Telekinesis Object reference
            m_ActiveObject = null;
        }
    }

    // Return an average velocity from a queue of recent positions.
    private Vector3 GetMeanVector(Queue<Vector3> positions)
    {
        if (positions.Count == 0)
            return Vector3.zero;
        float x = 0f;
        float y = 0f;
        float z = 0f;
        foreach (Vector3 pos in positions)
        {
            x += pos.x;
            y += pos.y;
            z += pos.z;
        }
        return new Vector3(x / positions.Count, y / positions.Count, z / positions.Count);
    }

    //Start or stop all of the particle instances in the magic beam
    private void ParticleSetEnabled(bool enable)
    {
        if (enable)
        {
            foreach (ParticleSystem ps in m_ParticleHolder.GetComponentsInChildren<ParticleSystem>())
            {
                ps.Play();
            }
        }
        else
        {
            foreach (ParticleSystem ps in m_ParticleHolder.GetComponentsInChildren<ParticleSystem>())
            {
                ps.Stop();
            }
        }
    }

    // Linear remap
    private float Remap(float value, float sourceMin, float sourceMax, float destMin, float destMax)
    {
        return Mathf.Lerp(destMin, destMax, Mathf.InverseLerp(sourceMin, sourceMax, value));
    }

    // Build a curve by tracing points on lines
    private Vector3[] QuadraticBezierCurve(Vector3 origin, Vector3 influence, Vector3 destination, int pointCount)
    {
        Vector3[] result = new Vector3[pointCount];
        if (pointCount == 0)
            return result;

        result[0] = origin;
        for (int i = 1; i < pointCount - 1; i++)
        {
            float percent = (1f / pointCount) * i;
            Vector3 point1 = Vector3.Lerp(origin, influence, percent);
            Vector3 point2 = Vector3.Lerp(influence, destination, percent);
            result[i] = Vector3.Lerp(point1, point2, percent);
        }
        result[pointCount - 1] = destination;

        return result;
    }

    [Serializable]
    public class ControllerRef
    {
        public GameObject siblingGO;
        public OVRInput.Button grabInput;
        public OVRInput.Axis2D depthInput;
        public bool invertX = false;
        public bool invertY = false;
        public bool flipXY = false;

        public bool GetButton()
        {
            return OVRInput.Get(grabInput);
        }
        public float GetX()
        {
            float val;
            if (flipXY)
            {
                val = OVRInput.Get(depthInput).y;
            }
            else
            {
                val = OVRInput.Get(depthInput).x;
            }
            if (invertX)
            {
                val = RemapAxis(val);
            }
            return val;
        }
        public float GetY()
        {
            float val;
            if (flipXY)
            {
                val = OVRInput.Get(depthInput).x;
            }
            else
            {
                val = OVRInput.Get(depthInput).y;
            }
            if (invertY)
            {
                val = RemapAxis(val);
            }
            return val;
        }

        private float RemapAxis(float value)
        {
            return Mathf.Lerp(1f, -1f, Mathf.InverseLerp(-1f, 1f, value));
        }
    }

}
