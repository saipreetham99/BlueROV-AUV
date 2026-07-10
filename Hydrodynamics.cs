using UnityEngine;

/// <summary>
/// 6-DOF hydrodynamics for an ROV, following the Fossen model.
/// Implements three of the four force terms:
///   1. Restoring  - gravity + buoyancy, including self-righting
///   2. Added mass - water that accelerates with the hull
///   3. Damping    - linear skin friction + quadratic form drag
/// Coriolis/centripetal (term 4) is intentionally left out until the core is validated.
///
/// Axis convention (Unity local space):
///   surge = +Z (forward), sway = +X (right), heave = +Y (up)
///   roll  = about Z, pitch = about X, yaw = about Y
///
/// Vector3 fields are ordered (X, Y, Z) = (sway, heave, surge) for translation
/// and (pitch, yaw, roll) for rotation.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class Hydrodynamics : MonoBehaviour
{
    [Header("Restoring")]
    [Tooltip("Buoyancy as a fraction of weight. 1.0 = neutrally buoyant (holds depth).")]
    public float buoyancyFactor = 1.0f;

    [Tooltip("Height of the centre of buoyancy above the centre of gravity (m). Drives self-righting.")]
    public float centreOfBuoyancyHeight = 0.01f;

    [Header("Added mass  (kg for translation, kg*m^2 for rotation)")]
    [Tooltip("Translational added mass: (sway X, heave Y, surge Z).")]
    public Vector3 addedMassLinear = new Vector3(12.7f, 14.57f, 5.5f);

    [Tooltip("Rotational added inertia: (pitch X, yaw Y, roll Z).")]
    public Vector3 addedMassAngular = new Vector3(0.12f, 0.12f, 0.12f);

    [Header("Linear damping (low-speed)")]
    [Tooltip("Translational: (sway X, heave Y, surge Z).")]
    public Vector3 dragLinearTranslation = new Vector3(6.22f, 5.18f, 4.03f);

    [Tooltip("Rotational: (pitch X, yaw Y, roll Z).")]
    public Vector3 dragLinearRotation = new Vector3(0.07f, 0.07f, 0.07f);

    [Header("Quadratic damping (form drag)")]
    [Tooltip("Translational: (sway X, heave Y, surge Z).")]
    public Vector3 dragQuadTranslation = new Vector3(21.66f, 36.99f, 18.18f);

    [Tooltip("Rotational: (pitch X, yaw Y, roll Z).")]
    public Vector3 dragQuadRotation = new Vector3(1.55f, 1.55f, 1.55f);

    const float G = 9.81f;

    Rigidbody rb;
    float weight;    // N
    float buoyancy;  // N

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        // We own every force; make sure Unity adds none of its own.
        rb.useGravity = false;
        rb.linearDamping = 0f;
        rb.angularDamping = 0f;

        weight = rb.mass * G;
        buoyancy = weight * buoyancyFactor;

        // Fold rotational added inertia into the inertia tensor, so any torque
        // (damping or restoring) is divided by (I_dry + I_added) automatically.
        // Note: identity rotation assumes body axes ~= principal axes (fine for a
        // roughly symmetric ROV; tune later if needed).
        Vector3 dryInertia = rb.inertiaTensor;
        rb.inertiaTensorRotation = Quaternion.identity;
        rb.inertiaTensor = dryInertia + addedMassAngular;
    }

    void FixedUpdate()
    {
        ApplyRestoring();
        ApplyDamping();
    }

    // Term 1: weight down at the CG, buoyancy up at the CB.
    // The CB sitting above the CG produces the self-righting moment for free.
    // At neutral buoyancy the net force is ~0, so only the righting torque remains.
    void ApplyRestoring()
    {
        Vector3 cg = rb.worldCenterOfMass;
        Vector3 cb = cg + transform.up * centreOfBuoyancyHeight;

        rb.AddForceAtPosition(Vector3.down * weight, cg, ForceMode.Force);
        rb.AddForceAtPosition(Vector3.up * buoyancy, cb, ForceMode.Force);
    }

    // Terms 2 + 3: quadratic-plus-linear damping, corrected for translational added mass.
    void ApplyDamping()
    {
        // --- Translational ---
        Vector3 v = transform.InverseTransformDirection(rb.linearVelocity);

        Vector3 f = new Vector3(
            -(dragLinearTranslation.x + dragQuadTranslation.x * Mathf.Abs(v.x)) * v.x,
            -(dragLinearTranslation.y + dragQuadTranslation.y * Mathf.Abs(v.y)) * v.y,
            -(dragLinearTranslation.z + dragQuadTranslation.z * Mathf.Abs(v.z)) * v.z
        );

        // Effective-mass correction. Unity integrates a = F / mass, but the true
        // acceleration is F / (mass + addedMass) per axis. Since Unity's mass is a
        // single scalar it can't hold anisotropic added mass, so we scale each axis
        // here to make the resulting acceleration match the effective mass.
        f.x *= rb.mass / (rb.mass + addedMassLinear.x);
        f.y *= rb.mass / (rb.mass + addedMassLinear.y);
        f.z *= rb.mass / (rb.mass + addedMassLinear.z);

        rb.AddForce(transform.TransformDirection(f), ForceMode.Force);

        // --- Rotational ---
        // Angular added inertia is already in the inertia tensor, so torque is
        // applied directly in body space.
        Vector3 w = transform.InverseTransformDirection(rb.angularVelocity);

        Vector3 t = new Vector3(
            -(dragLinearRotation.x + dragQuadRotation.x * Mathf.Abs(w.x)) * w.x,
            -(dragLinearRotation.y + dragQuadRotation.y * Mathf.Abs(w.y)) * w.y,
            -(dragLinearRotation.z + dragQuadRotation.z * Mathf.Abs(w.z)) * w.z
        );

        rb.AddRelativeTorque(t, ForceMode.Force);
    }
}
