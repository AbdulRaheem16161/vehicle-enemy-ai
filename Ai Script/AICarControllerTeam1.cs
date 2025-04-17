using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;


public class AICarControllerTeam1 : MonoBehaviourPun
{
    [Header(" -- GENERAL -- ")]

    private Rigidbody rb;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [Space(2)]



    [Header(" -- GENERAL VALUES -- ")]

    [SerializeField] private float gravity = -9.81f;
    [SerializeField] private float antiRollStrength = 5000f;
    [SerializeField] private Vector3 centerOfMass = new Vector3(0, -0.5f, 0);

    [Header("Steering")]
    [SerializeField] private float currentSteerAngle;
    [SerializeField] private float maxSteerAngle = 30f;
    [SerializeField] private float frontLeftSteerAngle_Value;
    [SerializeField] private float frontRightSteerAngle_Value;

    [Header("Drag")]
    [SerializeField] private float Drag_Value;
    [SerializeField] private float defaultDrag = 0f;
    [SerializeField] private float angularDrag = 0f;
    [SerializeField] private float boostedDrag = 3f;

    [Header("Car Motion")]
    [SerializeField] private float motorForce = 3000f;
    [SerializeField] private float MotorForce_Value;
    [SerializeField] private float carSpeed;

    [Space(2)]



    [Header(" -- MOVE TO POINT TO FOLLOW -- ")]

    public Transform PointToFollow;
    private GameObject targetPointObj;
    private RaycastHit hit;
    private Vector3 rayOrigin;
    private Vector3 directionToTarget;
    [SerializeField] private float DistanceFromPointToFollow;

    [Header("Values")]
    [SerializeField] private float ViewRange = 20f;
    [SerializeField] private float WillStopFollowingAfterDistance = 100f;
    [SerializeField] private float EnemyDetectingRaysLength = 50f;
    [SerializeField] private float speedThreshold = 50f;
    [SerializeField] private float A_drag;
    [SerializeField] private float minimumSpeedFor_A_Drag_ToLerp = 10f;


    [SerializeField] private bool EnemyDetected;
    [SerializeField] private bool EnemyDetectingRay;
    [SerializeField] private bool MovingTowardsTarget;

    [Space(2)]



    [Header(" -- RAYS AND OBSTACLE AVOIDANCE -- ")]

    [SerializeField] private LayerMask AICarTeam1;
    [SerializeField] private LayerMask AICarTeam2;
    [SerializeField] private LayerMask Team1sTarget;
    [SerializeField] private LayerMask obstacleLayer;
    [SerializeField] private LayerMask obstacleLayer_Walls;

    [Header("Values")]
    [SerializeField] private float ObstacleAvoidingRaysLength = 50f;
    [SerializeField] private float VilocityWithMaxDrag = 100f;
    [SerializeField] private float maxDragDuringAvoidingObstacal = 15000f;
    [SerializeField] private float DragDuringAvoidingObstacal_Value;

    [SerializeField] private bool rightHit1, rightHit2, rightHit3, rightHit4;
    [SerializeField] private bool leftHit1, leftHit2, leftHit3, leftHit4;
    [SerializeField] private bool frontHit1_R, frontHit2_R, frontHit3_R;
    [SerializeField] private bool frontHit1, frontHit2, SpecialfrontHit;
    [SerializeField] private bool sideHit1, sideHit2;

    [SerializeField] private bool Reversing;
    [SerializeField] private bool leftRaysDisabled; 
    [SerializeField] private bool rightRaysDisabled;
    [SerializeField] private bool frontRay1Disabled;
    [SerializeField] private bool frontRay2Disabled;
    [SerializeField] private bool busyInAvoidingTheObstacle;

    [Space(2)]
     


   
    [Header(" -- RANDOM MOVEMENT -- ")]
   

    [Header("On/Off Buttons")]
    [SerializeField] private bool ON_OFFMoveToTarget = true;
    [SerializeField] private bool ON_OFFRandomMovement = false;
    [SerializeField] private bool ON_OFFObsticalDetectionRays = true;

    [Header("Values")]
    [SerializeField] private float RandomSteerAmount_Value;                                          
    [SerializeField] private float minRandomSteerAngle = 15f;
    [SerializeField] private float maxRandomSteerAngle = 30f;
    [SerializeField] private float MinSteeringDuration = 0.5f;
    [SerializeField] private float MaxSteeringDuration = 3f;
    [SerializeField] private float MinDelayBWsteerings = 1f;
    [SerializeField] private float MaxDelayBWsteerings = 7f;

    [SerializeField] private bool isSteering = false;
    [SerializeField] private bool BusyInTakingaRandomTurn = false;

    [Space(2)]



    [Header(" -- JUMP TILT -- ")]
    public Transform bonnetPoint;

    [Header("Values")]
    [SerializeField] private float downwardForce = 100f;
    [SerializeField] private bool isGrounded = false;

    [Space(2)]



    [Header(" -- IS TOUCHING GROUND -- ")]
    [SerializeField] private LayerMask groundLayer;

    [Header("Values")]
    [SerializeField] private float rayLength = 0.4f;
    [SerializeField] private bool GroundDetectingRay;

    [Space(2)]



    [Header(" -- BACK FLIP -- ")]

    [Header("Values")]
    public float flipThreshold = -0.8f; 
    public float flipSpeed = 2f;

    [Space(2)]



    [Header(" -- BOUNCY COLLISION")]
    public LayerMask bounceLayers; 
    public AnimationCurve bounceCurve; 

    [Header("Values")]
    public float bounceForce = 10f; 
    public float bounceDamping = 0.5f;

    [Space(2)]



    [Header("debugging Flags")]
    [SerializeField] private bool a = false;

    [Header("Group")]
    [SerializeField] private bool group1 = false;
    [SerializeField] private bool group2 = false;
    [SerializeField] private bool group3 = false;


    void Start()
    {
        // Randomly choose a group to set to true
        int randomGroup = Random.Range(1, 4); // Generates a random number between 1 and 3

        // Reset all groups to false before setting one to true
        group1 = group2 = group3 = false;

        switch (randomGroup)
        {
            case 1:
                group1 = true;
                break;
            case 2:
                group2 = true;
                break;
            case 3:
                group3 = true;
                break;
        }

    }


    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody component is missing on " + gameObject.name);
        }
        else
        {
            rb.centerOfMass = centerOfMass;
            rb.useGravity = true;
            rb.drag = defaultDrag;
            rb.angularDrag = angularDrag;
        }

        // Initialize movement settings
        ON_OFFRandomMovement = true;
        ON_OFFMoveToTarget = true;
    }

    private void FixedUpdate()
    {
        if (targetPointObj != null)
        {
           DistanceFromPointToFollow = Vector3.Distance(transform.position, targetPointObj.transform.position);
        }
        carSpeed = rb.velocity.magnitude;
        if (carSpeed > minimumSpeedFor_A_Drag_ToLerp)
        {
            // If speed is above the threshold, calculate drag using Lerp
            A_drag = Mathf.Lerp(0f, 3f, Mathf.Clamp01(carSpeed / speedThreshold));
        }
        else
        {
            // If speed is at or below the threshold, drag is zero
            A_drag = 0f;
        }

        if (PhotonNetwork.IsMasterClient)
        {
            Gravity();
            FindingPointToFollow();
            ApplyAntiRoll();
            BackFlip();
            UpdateWheels();
            IsGrounded();
            MoveToTarget();
            RandomMovement();
            ObsticalDetectionRays();

        }
    }

    #region SyncAIPosition()
    [PunRPC]
    void SyncAIPosition(Vector3 position, Quaternion rotation)
    {
        // Non-master clients apply the position and rotation updates from the master client
        if (!PhotonNetwork.IsMasterClient)
        {
            transform.position = position;
            transform.rotation = rotation;
        }
    }
    #endregion

    #region ApplyAntiRollBar()


    private void ApplyAntiRollBar(WheelCollider leftWheel, WheelCollider rightWheel)
    {
        WheelHit hit;
        float leftCompression = 1.0f;
        float rightCompression = 1.0f;

        bool groundedLeft = leftWheel.GetGroundHit(out hit);
        if (groundedLeft)
        {
            leftCompression = (-leftWheel.transform.InverseTransformPoint(hit.point).y - leftWheel.radius) / leftWheel.suspensionDistance;
        }

        bool groundedRight = rightWheel.GetGroundHit(out hit);
        if (groundedRight)
        {
            rightCompression = (-rightWheel.transform.InverseTransformPoint(hit.point).y - rightWheel.radius) / rightWheel.suspensionDistance;
        }

        float antiRollForce = (leftCompression - rightCompression) * antiRollStrength;

        if (groundedLeft)
        {
            rb.AddForceAtPosition(leftWheel.transform.up * -antiRollForce, leftWheel.transform.position);
        }

        if (groundedRight)
        {
            rb.AddForceAtPosition(rightWheel.transform.up * antiRollForce, rightWheel.transform.position);
        }
    }
    #endregion

    #region Wheels()
    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
        wheelTransform.position = pos;
    }
    #endregion

    #region MoveToTarget()
    [PunRPC]
    private void MoveToTarget()
    {
        if (DistanceFromPointToFollow <= ViewRange && ON_OFFMoveToTarget)
        {
            RaycastHit hit;
            rayOrigin = transform.position + new Vector3(0, 1f, 0);

            int combinedLayerMask = Team1sTarget | AICarTeam2;

            for (int angle = -25; angle <= 25; angle += 5)
            {
                EnemyDetectingRay = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(angle, Vector3.up) * transform.forward, out hit, (EnemyDetectingRaysLength), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(angle, Vector3.up) * transform.forward * (EnemyDetectingRaysLength), Color.black);
            }


            if (!Reversing && !BusyInTakingaRandomTurn)
            {
                MovingTowardsTarget = true;
                if (PointToFollow != null)
                {
                    directionToTarget = (PointToFollow.position - transform.position).normalized;
                }
                float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
                float steerAmount = Mathf.Clamp(angleToTarget, -maxSteerAngle, maxSteerAngle);

                frontLeftSteerAngle_Value = Mathf.MoveTowards(frontLeftSteerAngle_Value, steerAmount, 5f);
                frontRightSteerAngle_Value = Mathf.MoveTowards(frontRightSteerAngle_Value, steerAmount, 5f);

                frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;


                if (EnemyDetectingRay)
                {
                    MotorForce_Value = 2000;
                    Drag_Value = 10f;
                    EnemyDetected = true;
                }
                else
                {
                    EnemyDetected = false;
                    if (frontRightSteerAngle_Value > 25f || frontRightSteerAngle_Value < -25f && GroundDetectingRay)
                    {
                        StartCoroutine(ResetDragAfterDelay(3f, A_drag));
                        MotorForce_Value = motorForce;
                    }
                    else
                    {
                        MotorForce_Value = motorForce;
                        Drag_Value = defaultDrag;
                    }

                    #region MotorForce_Value and Drag_cValue Change with Distance (Comented Out)
                    #endregion
                }
                rearLeftWheelCollider.motorTorque = MotorForce_Value;
                rearRightWheelCollider.motorTorque = MotorForce_Value;

                if (frontLeftSteerAngle_Value == 0f)
                {
                    frontLeftWheelCollider.motorTorque = MotorForce_Value;
                    frontRightWheelCollider.motorTorque = MotorForce_Value;
                }
                rb.drag = Drag_Value;
            }
            else
            {
                MovingTowardsTarget = false;
            }
        }

    }
    #endregion

    #region ObsticalDetectionRays()
    [PunRPC]
    private void ObsticalDetectionRays()
    {
        if (ON_OFFObsticalDetectionRays)
        {

            RaycastHit hit;
            rayOrigin = transform.position + new Vector3(0, 1f, 0);

            int combinedLayerMask = obstacleLayer | obstacleLayer_Walls | AICarTeam1;

            // Special Front Ray
            SpecialfrontHit = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(0, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 6f), combinedLayerMask);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(0, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 6f), Color.grey);

            // front Ray
            if (!frontRay1Disabled)
            {
                frontHit1 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(7, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 4f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(7, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 4f), Color.yellow);
            }
            if (!frontRay2Disabled)
            {
                frontHit2 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-7, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 4f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-7, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 4f), Color.yellow);
            }
            // left Right Rays
            if (!leftRaysDisabled)
            {
                leftHit1 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(15, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                leftHit2 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(30, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(15, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.red);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(30, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.red);

                leftHit3 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(45, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                leftHit4 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(60, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(45, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.red);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(60, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.red);
            }
            if (!rightRaysDisabled)
            {
                rightHit1 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-15, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                rightHit2 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-30, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-15, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.blue);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-30, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.blue);

                rightHit3 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-45, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                rightHit4 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-60, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 2f), combinedLayerMask);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-45, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.blue);
                Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-60, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 2f), Color.blue);
            }
            // Side Rays
            sideHit1 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(90, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 8f), combinedLayerMask);
            sideHit2 = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-90, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength * 8f), combinedLayerMask);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(90, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 8f), Color.magenta);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-90, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength * 8f), Color.magenta);

            // front Rays for reverse
            frontHit1_R = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(0, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength / 2f), combinedLayerMask);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(0, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength / 1.25f), Color.black);

            frontHit2_R = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(15, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength / 2f), combinedLayerMask);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(15, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength / 1.25f), Color.black);

            frontHit3_R = Physics.Raycast(rayOrigin, Quaternion.AngleAxis(-15, Vector3.up) * transform.forward, out hit, (ObstacleAvoidingRaysLength / 2f), combinedLayerMask);
            Debug.DrawRay(rayOrigin, Quaternion.AngleAxis(-15, Vector3.up) * transform.forward * (ObstacleAvoidingRaysLength / 1.25f), Color.black);



            if (rightHit1 || rightHit2 || rightHit3 || rightHit4 || leftHit1 || leftHit2 || leftHit3 || leftHit4 || frontHit1_R || frontHit2_R || frontHit3_R || frontHit1 || frontHit2)
            {
                busyInAvoidingTheObstacle = true;
                StartCoroutine(ResetBusyInAvoidingTheObstacleAfterDelay(0.1f));

                if (frontHit1 && sideHit2 && !frontRay1Disabled && !EnemyDetected)
                {

                    //frontHit1 = false;
                    frontRay2Disabled = true;
                    frontLeftSteerAngle_Value = 30f;
                    frontRightSteerAngle_Value = 30f;

                    frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                    frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;
                    StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                    StartCoroutine(EnableFrontRay2AfterDelay(2f));

                    if (GroundDetectingRay)
                    {
                        DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                        DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                        StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                    }
                }
                else if (frontHit1 && !frontRay1Disabled && !EnemyDetected)
                {
                    //frontHit1 = false;
                    frontRay2Disabled = true;
                    frontLeftSteerAngle_Value = -30f;
                    frontRightSteerAngle_Value = -30f;

                    frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                    frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;
                    StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                    StartCoroutine(EnableFrontRay2AfterDelay(2f));
                    if (GroundDetectingRay)
                    {
                        DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                        DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                        StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                    }
                }
                if (frontHit2 && sideHit1 && !frontRay2Disabled && !EnemyDetected)
                {
                    //frontHit2 = false;
                    frontRay1Disabled = true;
                    frontLeftSteerAngle_Value = -30f;
                    frontRightSteerAngle_Value = -30f;

                    frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                    frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;
                    StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                    StartCoroutine(EnableFrontRay1AfterDelay(2f));

                    if (GroundDetectingRay)
                    {
                        DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                        DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                        StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                    }
                }
                else if (frontHit2 && !frontRay2Disabled && !EnemyDetected)
                {
                    //frontHit2 = false;
                    frontRay1Disabled = true;
                    frontLeftSteerAngle_Value = 30f;
                    frontRightSteerAngle_Value = 30f;

                    frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                    frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;
                    StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                    StartCoroutine(EnableFrontRay1AfterDelay(2f));

                    if (GroundDetectingRay)
                    {
                        DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                        DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                        StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                    }
                }


                if (rightHit1 || rightHit2 || rightHit3 || rightHit4 && !rightRaysDisabled)
                {
                    if (!Reversing)
                    {
                        frontLeftSteerAngle_Value = 30f;
                        frontRightSteerAngle_Value = 30f;

                        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;

                        leftRaysDisabled = true;
                        StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                        StartCoroutine(EnableLeftRaysAfterDelay(0.3f));

                        if (GroundDetectingRay)
                        {
                            DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                            DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                            StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                        }
                    }
                }
                if (leftHit1 || leftHit2 || leftHit3 || leftHit4 && !leftRaysDisabled)
                {
                    if (!Reversing)
                    {
                        frontLeftSteerAngle_Value = -30f;
                        frontRightSteerAngle_Value = -30f;

                        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
                        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;

                        rightRaysDisabled = true;
                        StartCoroutine(ResetSteeringAfterAvoidingObstacal(1f));
                        StartCoroutine(EnableRightRaysAfterDelay(0.3f));

                        if (GroundDetectingRay)
                        {
                            DragDuringAvoidingObstacal_Value = (carSpeed / VilocityWithMaxDrag) * maxDragDuringAvoidingObstacal;
                            DragDuringAvoidingObstacal_Value = Mathf.Clamp(Drag_Value, 0f, maxDragDuringAvoidingObstacal);
                            StartCoroutine(ResetDragAfterDelay(5f, DragDuringAvoidingObstacal_Value));
                        }
                    }
                }
                if (frontHit1_R || frontHit2_R || frontHit3_R)
                {
                    Reversing = true;
                    frontRay1Disabled = true;
                    frontRay2Disabled = true;
                    rightRaysDisabled = true;
                    leftRaysDisabled = true;

                    StartCoroutine(ResetSteeringAfterAvoidingObstacal(2f));
                    StartCoroutine(TurningAfterReversing(2f));
                    StartCoroutine(EnableRightRaysAfterDelay(5f));
                    StartCoroutine(EnableLeftRaysAfterDelay(5f));
                    StartCoroutine(EnableFrontRay1AfterDelay(5f));
                    StartCoroutine(EnableFrontRay1AfterDelay(5f));
                }
                if (SpecialfrontHit && carSpeed >= 70f && GroundDetectingRay)
                {
                    StartCoroutine(ResetDragAfterDelay(1f, 11f));
                }
                else if (SpecialfrontHit && carSpeed >= 40f && GroundDetectingRay)
                {
                    StartCoroutine(ResetDragAfterDelay(1f, 8f));
                }
                else if (SpecialfrontHit && carSpeed >= 20f && GroundDetectingRay)
                {
                    StartCoroutine(ResetDragAfterDelay(1f, 5f));
                }
            }
        }
    }
    #endregion

    #region Coroutines
    private IEnumerator ResetBusyInAvoidingTheObstacleAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        busyInAvoidingTheObstacle = false;
    }

    private IEnumerator ResetSteeringAfterAvoidingObstacal(float delay)
    {
        yield return new WaitForSeconds(delay);
        frontLeftSteerAngle_Value = 0f;
        frontRightSteerAngle_Value = 0f;

        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;
    }

    private IEnumerator EnableFrontRay1AfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        frontRay1Disabled = false;
    }
    private IEnumerator EnableFrontRay2AfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        frontRay2Disabled = false;
    }

    private IEnumerator EnableRightRaysAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        rightRaysDisabled = false;
    }

    private IEnumerator EnableLeftRaysAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        leftRaysDisabled = false;
    }
    private IEnumerator ResetDragAfterDelay(float delay, float drag)
    {
        Drag_Value = drag;
        rb.drag = Drag_Value;
        yield return new WaitForSeconds(delay);
        Drag_Value = defaultDrag;
        rb.drag = Drag_Value;
    }
    private IEnumerator TurningAfterReversing(float delay)
    {
        frontLeftSteerAngle_Value = 0f;
        frontRightSteerAngle_Value = 0f;

        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;

        MotorForce_Value = -motorForce;
        rearLeftWheelCollider.motorTorque = MotorForce_Value;
        rearRightWheelCollider.motorTorque = MotorForce_Value;

        if (frontLeftSteerAngle_Value == 0f)
        {
            MotorForce_Value = -motorForce;
            frontLeftWheelCollider.motorTorque = MotorForce_Value;
            frontRightWheelCollider.motorTorque = MotorForce_Value;
        }

        yield return new WaitForSeconds(1.5f);

        MotorForce_Value = motorForce;
        rearLeftWheelCollider.motorTorque = MotorForce_Value;
        rearRightWheelCollider.motorTorque = MotorForce_Value;

        if (frontLeftSteerAngle_Value == 0f)
        {
            MotorForce_Value = motorForce;
            frontLeftWheelCollider.motorTorque = MotorForce_Value;
            frontRightWheelCollider.motorTorque = MotorForce_Value;
        }

        Reversing = false;
    }
    #endregion

    #region RandomMovement()
    [PunRPC]
    private void RandomMovement()
    {
        if (ON_OFFRandomMovement)
        {
            if (!isSteering)  // Check to ensure random steering isn't already in process
            {
                isSteering = true;
                if (EnemyDetectingRay)
                {

                    MotorForce_Value = 4000;
                    rearLeftWheelCollider.motorTorque = MotorForce_Value;
                    rearRightWheelCollider.motorTorque = MotorForce_Value;
                    frontLeftWheelCollider.motorTorque = MotorForce_Value;
                    frontRightWheelCollider.motorTorque = MotorForce_Value;
                }
                else
                {
                    MotorForce_Value = motorForce;
                    rearLeftWheelCollider.motorTorque = MotorForce_Value;
                    rearRightWheelCollider.motorTorque = MotorForce_Value;
                    if (frontLeftSteerAngle_Value == 0f)
                    {
                        frontLeftWheelCollider.motorTorque = MotorForce_Value;
                        frontRightWheelCollider.motorTorque = MotorForce_Value;
                    }
                    rb.drag = Drag_Value;
                    Invoke("RandomSteering", Random.Range(MinDelayBWsteerings, MaxDelayBWsteerings)); // Invoke RandomSteering after a random delay between 1 and 8 seconds.
                }


            }
        }
    }

    [PunRPC]
    private void RandomSteering()
    {
        BusyInTakingaRandomTurn = true;
        RandomSteerAmount_Value = Random.Range(minRandomSteerAngle, maxRandomSteerAngle);

        frontLeftSteerAngle_Value = RandomSteerAmount_Value;
        frontRightSteerAngle_Value = RandomSteerAmount_Value;

        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;

        // Set the steer angles for random duration (0.5 to 3 seconds)
        Invoke("ResetSteering", Random.Range(MinSteeringDuration, MaxSteeringDuration));
    }

    [PunRPC]
    private void ResetSteering()
    {
        frontLeftSteerAngle_Value = 0f;
        frontRightSteerAngle_Value = 0f;
        frontLeftWheelCollider.steerAngle = frontLeftSteerAngle_Value;
        frontRightWheelCollider.steerAngle = frontRightSteerAngle_Value;

        // Allow random movement to start again
        isSteering = false;
        BusyInTakingaRandomTurn = false;
    }
    #endregion

    #region FindingPointToFollow()
    private void FindingPointToFollow()
    {
        // Check if target is null (destroyed) or has moved farther than maxDistance
        if (targetPointObj == null || DistanceFromPointToFollow > WillStopFollowingAfterDistance || !targetPointObj.activeInHierarchy)
        {
            GameObject[] pointObjects = null; // Declare it here, so it's accessible in the entire block

            if (group1 == true)
            {
                pointObjects = GameObject.FindGameObjectsWithTag("Point to follow for Team 1 Group 1");
            }
            else if (group2 == true)
            {
                pointObjects = GameObject.FindGameObjectsWithTag("Point to follow for Team 1 Group 2");
            }
            else if (group3 == true)
            {
                pointObjects = GameObject.FindGameObjectsWithTag("Point to follow for Team 1 Group 3");
            }

            // Check if there are any objects with the tag
            if (pointObjects != null && pointObjects.Length > 0)
            {
                // Set an initial value for the closest distance (use a large value)
                float closestDistance = Mathf.Infinity;
                GameObject closestObject = null;

                // Iterate through each object and find the closest one
                foreach (GameObject pointObj in pointObjects)
                {
                    // Calculate the distance from the current object to the pointObj
                    float distanceToObj = Vector3.Distance(transform.position, pointObj.transform.position);

                    // If the current object is closer than the previous closest, update the closest
                    if (distanceToObj < closestDistance) // Use distanceToObj here instead
                    {
                        closestDistance = distanceToObj;
                        closestObject = pointObj;
                    }
                }

                // Update targetPointObj and PointToFollow if a closer object is found
                if (closestObject != null)
                {
                    targetPointObj = closestObject;
                    PointToFollow = targetPointObj.transform;
                }
            }
        }
    }


    #endregion

    private void  Gravity()
    {
        rb.AddForce(Vector3.up * gravity, ForceMode.Acceleration);
    }

    private void JumpTily()
    {
        if (!GroundDetectingRay && rb != null)
        {
            Vector3 downwardForceVector = Vector3.down * downwardForce;
            rb.AddForceAtPosition(downwardForceVector, bonnetPoint.position, ForceMode.Impulse);
        }
        else if (rb == null)
        {
            Debug.LogError("Rigidbody is null in JumpTily!");
        }
    }


    void OnCollisionEnter(Collision collision)
    {
        if (rb == null)
        {
            Debug.LogError("Rigidbody is null on " + gameObject.name);
            return;
        }

        if ((bounceLayers.value & (1 << collision.gameObject.layer)) != 0)
        {
            foreach (ContactPoint contact in collision.contacts)
            {
                Vector3 bounceDirection = -contact.normal;
                float adjustedBounceForce = bounceForce * (1 - bounceDamping);

                rb.AddForce(bounceDirection * adjustedBounceForce, ForceMode.Impulse);
            }
        }
    }


    private void IsGrounded()
    {
        RaycastHit hit;
        GroundDetectingRay = Physics.Raycast(transform.position, Vector3.down, out hit, rayLength, groundLayer);
        Debug.DrawRay(transform.position, Vector3.down * rayLength, Color.red);
    }

    private void ApplyAntiRoll()
    {
        ApplyAntiRollBar(frontLeftWheelCollider, frontRightWheelCollider);
        ApplyAntiRollBar(rearLeftWheelCollider, rearRightWheelCollider);
    }

    private void BackFlip()
    {
        if (transform.up.y < flipThreshold)
        {
            Quaternion uprightRotation = Quaternion.Euler(0, transform.rotation.eulerAngles.y, 0);
            transform.rotation = Quaternion.Slerp(transform.rotation, uprightRotation, flipSpeed * Time.deltaTime);

            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            else
            {
                Debug.LogError("Rigidbody is null in BackFlip!");
            }
        }
    }
}
