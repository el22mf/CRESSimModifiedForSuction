using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Crtk;
using System.Collections;
using ROSRobotUtils;
using RosMessageTypes.Std;
using RosMessageTypes.Cisst;
using PhysX5ForUnity;
using System;


public class PSM_ROS_Control : MonoBehaviour
{
    // PSM Settigs
    [SerializeField] string PSMName = "PSM1";
    [SerializeField] public Transform baseFrame;
    [SerializeField] protected PSMControllerBase m_psmController;
    [SerializeField] Transform psmBase;
     protected PxTransformData m_psmPoseTarget;

    // PSM data stores
    private float[] jointSetpoint;
    private PxTransformData cartesianSetpoint = new PxTransformData(new Vector3((float)-0.2, (float)-0.25, (float)0.1), new Quaternion((float)0, (float)0, (float)0, (float)1));
    private PxTransformData prevCartesianSetpoint;

    private float jawAngle;
    private PxTransformData basePose;

    // ROS connection
    private ROSConnection m_rosConnection;

    // Child topic base strings
    private string baseTopicCartesianCmd = "/servo_cp";
    private string baseTopicJointCmd = "/servo_jp";
    private string baseTopicJointState = "/measured_js";
    private string baseTopicCartesianState = "/measured_cp";
    private string baseTopicJawCmd = "/jaw/servo_jp";

    // ROS topic names (will be updated based on PSMName)
    private string topicCartesianCmd;
    private string topicJointCmd;
    private string topicJointState;
    private string topicCartesianState;
    private string topicJawCmd;

    private float feedbackInterval = 0.002f;
    private float lastFeedbackTime = 0f;

    // Messages and services
    private JointStateMsg msg_JointState;
    private JointStateMsg msg_JointCommand;
    private PoseStampedMsg msg_CartesianState;
    private PoseStampedMsg msg_CartesianCommand;
    private JointStateMsg msg_JawCommand;
    private bool initialised = false;


    void Start()
    {
        
        // Update topic names using PSMName prefix
        topicCartesianCmd = "/" + PSMName + baseTopicCartesianCmd;
        topicJointCmd = "/" + PSMName + baseTopicJointCmd;
        topicJointState = "/" + PSMName + baseTopicJointState;
        topicCartesianState = "/" + PSMName + baseTopicCartesianState;
        topicJawCmd = "/" + PSMName + baseTopicJawCmd;


        //Conncect to ROS
        // TOpics
        m_rosConnection = ROSConnection.GetOrCreateInstance();
        m_rosConnection.Subscribe<PoseStampedMsg>(topicCartesianCmd, CartesianCommandCallback);
        m_rosConnection.Subscribe<JointStateMsg>(topicJointCmd, JointCommandCallback);
        m_rosConnection.Subscribe<JointStateMsg>(topicJawCmd, JawCommandCallback);
        m_rosConnection.RegisterPublisher<JointStateMsg>(topicJointState);
        m_rosConnection.RegisterPublisher<PoseStampedMsg>(topicCartesianState);

        msg_JointState = new JointStateMsg();
        
        msg_JointState.position = new double[((PSMSuctionIrrigator6DofController)m_psmController).m_robot.NumJoints];
        msg_JointState.velocity = new double[((PSMSuctionIrrigator6DofController)m_psmController).m_robot.NumJoints];
        msg_JointState.effort   = new double[((PSMSuctionIrrigator6DofController)m_psmController).m_robot.NumJoints];
        msg_JointState.header   = new HeaderMsg();
        msg_JointState.header.frame_id = PSMName;

        // Make sure header is set properly for ROS1 (ROS2 doeasnt need seq in the header)
        #if ROS2
        #else
            msg_JointState.header.seq = 0;
        #endif

        // IK and FK
        basePose = new PxTransformData(baseFrame.position, psmBase.rotation);
        // m_rosConnection.RegisterRosService<QueryInverseKinematicsRequest, QueryInverseKinematicsResponse>(serviceIK);
        // m_rosConnection.RegisterRosService<QueryForwardKinematicsRequest, QueryForwardKinematicsResponse>(serviceFK);

        

        
        jawAngle = 0;
    }

    void FixedUpdate()
    {
        // if (!initialised)
        // {
            // Initialize joint setpoint
        // ((PSMLargeNeedleDriverController)m_psmController).DriveJoints(jointSetpoint);

        // // wait 1 second
        // StartCoroutine(wait(5.0f));
        // initialised = true;
        // }
        
        if (jointSetpoint != null)
        {
            ((PSMSuctionIrrigator6DofController)m_psmController).DriveJoints(jointSetpoint);
            jointSetpoint = null;
        }
        if (cartesianSetpoint.position != prevCartesianSetpoint.position || cartesianSetpoint.quaternion != prevCartesianSetpoint.quaternion)
        {
            ((PSMSuctionIrrigator6DofController)m_psmController).DriveCartesianPose(cartesianSetpoint);
            prevCartesianSetpoint = cartesianSetpoint;
        }
        // Debug.Log("Base pose: " + basePose.position + " " + basePose.quaternion);
        // Debug.Log("Previous Cartesian pose: " + prevCartesianSetpoint.position + " " + prevCartesianSetpoint.quaternion);
        // Debug.Log("Current Cartesian pose: " + cartesianSetpoint.position + " " + cartesianSetpoint.quaternion);
        // Debug.Log("Tes for logic: Position change?: " + (cartesianSetpoint.position != prevCartesianSetpoint.position) + " Rotation change?: " + (cartesianSetpoint.quaternion != prevCartesianSetpoint.quaternion));
    }
    void Update()
    {
        if (Time.time - lastFeedbackTime >= feedbackInterval)
        {
            lastFeedbackTime = Time.time;
            PublishRobotState();        
        }
    }
    
    // Publish current joint positions over ROS as feedback
    private void PublishRobotState()
    {
        float[] jointsFloat = (  (PSMSuctionIrrigator6DofController)m_psmController).m_robot.JointPositions;
        double[] jointsDouble = ((PSMSuctionIrrigator6DofController)m_psmController).GetJointPositionsDouble();

        double timeAsDouble = Time.timeAsDouble;
        int seconds = (int)timeAsDouble;
        double fractionalSeconds = timeAsDouble - seconds;
        uint nanoseconds = (uint)(fractionalSeconds * 1e9);

        msg_JointState.header.stamp = new TimeMsg(seconds, nanoseconds);
        msg_JointState.position = jointsDouble;
        // Debug.Log("Publishing Joint positions: " + jointsFloat[0] + " " + jointsFloat[1] + " " + jointsFloat[2] + " " + jointsFloat[3] + " " + jointsFloat[4] + " " + jointsFloat[5] + " " + jointsFloat[6]);
        m_rosConnection.Publish(topicJointState, msg_JointState);

        // Update FK
        PxTransformData currentEEPose = ((PSMSuctionIrrigator6DofController)m_psmController).GetEEPose();

        msg_CartesianState = new PoseStampedMsg(); 
        msg_CartesianState.header = new HeaderMsg();
        msg_CartesianState.header.stamp = new TimeMsg(seconds, nanoseconds);
        msg_CartesianState.header.frame_id = PSMName;
        msg_CartesianState.pose = new PoseMsg();
        msg_CartesianState.pose.position = currentEEPose.position.To<FLU>();
        msg_CartesianState.pose.orientation = currentEEPose.quaternion.To<FLU>();
        // Debug.Log("Publishing Cartesian pose: " + msg_CartesianState.pose.position + " " + msg_CartesianState.pose.orientation);
        m_rosConnection.Publish(topicCartesianState, msg_CartesianState);

    }

    // TODO
    // Callback for receiving Cartesian pose commands
    private void CartesianCommandCallback(PoseStampedMsg msg)
    {
        PxTransformData targetPoseWorld = new PxTransformData(msg.pose.position.From<FLU>(), msg.pose.orientation.From<FLU>());
        Debug.Log("Recieved msg pose: " + msg.pose.position + " " + msg.pose.orientation);
        PxTransformData targetPose = new PxTransformData();
        Matrix4x4 baseMatrix = Matrix4x4.TRS(basePose.position, basePose.quaternion, Vector3.one);
        Matrix4x4 targetWorldMatrix = Matrix4x4.TRS(targetPoseWorld.position, targetPoseWorld.quaternion, Vector3.one);
        Matrix4x4 targetMatrix = baseMatrix * targetWorldMatrix;

        targetPose.position = targetMatrix.GetColumn(3);
        targetPose.quaternion = targetMatrix.rotation;

        Debug.Log("World base pose: " + basePose.position + " " + basePose.quaternion);
        Debug.Log("Received World Cartesian command: " + targetPoseWorld.position + " " + targetPoseWorld.quaternion);
        Debug.Log("Received Cartesian command: " + targetPose.position + " " + targetPose.quaternion);
        cartesianSetpoint = targetPose;
        // ((PSMLargeNeedleDriverController)m_psmController).DriveCartesianPose(targetPose, jawAngle);
    }

    // Callback to take ROS joint commands and update the local joint state
    private void JointCommandCallback(JointStateMsg msg)
    {
        float[] newJoints = new float[msg.position.Length];
        for (int i = 0; i < msg.position.Length; i++)
        {
            newJoints[i] = (float)msg.position[i];
        }
        Debug.Log(newJoints.Length);
        //newJoints[2] = -newJoints[2];
        //newJoints[6] = jawAngle;
        jointSetpoint = newJoints;
        // Debug.Log("Received joint command: " + newJoints[0] + " " + newJoints[1] + " " + newJoints[2] + " " + newJoints[3] + " " + newJoints[4] + " " + newJoints[5] + " " + newJoints[6]);
        //((PSMLargeNeedleDriverController)m_psmController).DriveJoints(new float[] { newJoints[0], newJoints[1], newJoints[2], newJoints[3], newJoints[4], newJoints[5], newJoints[6] });
        // long time = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        // Debug.Log("Time: " + Math.Sin(time));
        // ((PSMLargeNeedleDriverController)m_psmController).DriveJoints(new float[] { 0.1f, (float)Math.Sin(time), -0.1f, 0, 0, 0, 0 });
    }

    private void JawCommandCallback(JointStateMsg msg)
    {
        Debug.Log("Received jaw command: " + msg.position[0]);
        jawAngle = (float)msg.position[0];
    }

    IEnumerator wait(float seconds)
    {
        //Print the time of when the function is first called.
        Debug.Log("Started Coroutine at timestamp : " + Time.time);

        //yield on a new YieldInstruction that waits for 5 seconds.
        yield return new WaitForSeconds(5);

        //After we have waited 5 seconds print the time again.
        Debug.Log("Finished Coroutine at timestamp : " + Time.time);
    }
}

    // private void GetEEPose()
    // {
    //     PxTransformData eePose = new PxTransformData();
    //     eePose.position = m_psmEETooltipJointOnSelf.position;
    //     eePose.quaternion = m_psmEETooltipJointOnSelf.rotation;
    // }

    // private PxTransformData SolveFK(float[] jointPositions)
    // {
    //     Matrix4x4 fk;
    //     // Getting the end-effector pos by doing FK
    //     // TODO: this is inefficient; use the current EE tooltip pose instead
    //     Physx.GetRobotForwardKinematics(m_robot.NativeObjectPtr, ref jointPositions[0], out fk);

    //     Quaternion currentRotation = MatrixToRotation(fk);
    //     Vector3 currentPosition = MatrixToPosition(fk);

    //     PxTransformData currentPose = new PxTransformData(currentPosition, currentRotation);
    //     Debug.Log("Current pose: " + currentPose.position + " " + currentPose.quaternion);
    //     return currentPose;
    // }


