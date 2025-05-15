// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using Unity.Robotics.ROSTCPConnector.ROSGeometry;
// using RosMessageTypes.Geometry;
// using RosMessageTypes.Sensor;
// using RosMessageTypes.BuiltinInterfaces;
// using RosMessageTypes.Crtk;
// using System.Collections;
// using ROSRobotUtils;
// using RosMessageTypes.Std;
// using RosMessageTypes.Cisst;
// using PhysX5ForUnity;
// using System;

// public class PSM_ROS_Control : PSMControllerBase
// {
//     // PSM Settigs
//     [SerializeField] string PSMName = "PSM1";
//     public GameObject base_frame;

//     // PSM data stores
//     private float[] jointPositionsIK;
//     private PxTransformData cartesianPoseFK;

//     private float jawAngle;
//     private PxTransformData basePose;

//     // ROS connection
//     private ROSConnection m_rosConnection;

//     // Child topic base strings
//     private string baseTopicCartesianCmd = "/servo_cp";
//     private string baseTopicJointCmd = "/servo_jp";
//     private string baseTopicJointState = "/measured_js";
//     private string baseTopicCartesianState = "/measured_cp";
//     private string baseTopicWrenchState = "/measured_cf";
//     private string baseTopicJawCmd = "/jaw/servo_jp";

//     // Service topics
//     private string serviceIK = "/inverse_kinematics";
//     private string serviceFK = "/forward_kinematics";

//     // ROS topic names (will be updated based on PSMName)
//     private string topicCartesianCmd;
//     private string topicJointCmd;
//     private string topicJointState;
//     private string topicCartesianState;
//     private string topicJawCmd;

//     private float feedbackInterval = 0.002f;
//     private float lastFeedbackTime = 0f;

//     // Messages and services
//     private JointStateMsg msg_JointState;
//     private JointStateMsg msg_JointCommand;
//     private PoseStampedMsg msg_CartesianState;
//     private PoseStampedMsg msg_CartesianCommand;
//     private JointStateMsg msg_JawCommand;

//     private QueryForwardKinematicsRequest srvFKRequest;
//     private QueryForwardKinematicsResponse srvFKResponse;
//     private QueryInverseKinematicsRequest srvIKRequest;
//     private QueryInverseKinematicsResponse srvIKResponse;


//     void Start()
//     {
//         // Update topic names using PSMName prefix
//         topicCartesianCmd = "/" + PSMName + baseTopicCartesianCmd;
//         topicJointCmd = "/" + PSMName + baseTopicJointCmd;
//         topicJointState = "/" + PSMName + baseTopicJointState;
//         topicCartesianState = "/" + PSMName + baseTopicCartesianState;
//         topicJawCmd = "/" + PSMName + baseTopicJawCmd;

//         serviceIK = "/" + PSMName + serviceIK;
//         serviceFK = "/" + PSMName + serviceFK;

//         //Conncect to ROS
//         // TOpics
//         m_rosConnection = ROSConnection.GetOrCreateInstance();
//         m_rosConnection.Subscribe<PoseStampedMsg>(topicCartesianCmd, CartesianCommandCallback);
//         m_rosConnection.Subscribe<JointStateMsg>(topicJointCmd, JointCommandCallback);
//         m_rosConnection.RegisterPublisher<JointStateMsg>(topicJointState);
//         m_rosConnection.RegisterPublisher<PoseStampedMsg>(topicCartesianState);

//         msg_JointState = new JointStateMsg();
        
//         msg_JointState.position = new double[m_robot.NumJoints];
//         msg_JointState.velocity = new double[m_robot.NumJoints];
//         msg_JointState.effort   = new double[m_robot.NumJoints];
//         msg_JointState.header   = new HeaderMsg();
//         msg_JointState.header.frame_id = PSMName;

//         // Make sure header is set properly for ROS1 (ROS2 doeasnt need seq in the header)
//         #if ROS2
//         #else
//             msg_JointState.header.seq = 0;
//         #endif

//         // IK and FK
//         basePose = new PxTransformData(base_frame.transform.position, base_frame.transform.rotation);
//         // m_rosConnection.RegisterRosService<QueryInverseKinematicsRequest, QueryInverseKinematicsResponse>(serviceIK);
//         // m_rosConnection.RegisterRosService<QueryForwardKinematicsRequest, QueryForwardKinematicsResponse>(serviceFK);

//         jawAngle = 0;

//         DriveJoints(new float[] { -0.8f, 0f, -1.5f, 0, 0, 0, 0 });

//     }

//     void Update()
//     {
//         if (Time.time - lastFeedbackTime >= feedbackInterval)
//         {
//             lastFeedbackTime = Time.time;
//             PublishRobotState();
//         }
//     }

//     public override void DriveJoints(float[] jointPositions)
//     {
//         if (jointPositions == null || jointPositions.Length != 7)
//         {
//             throw new ArgumentException("jointPositions must be an array of 7 floats.");
//         }

//         float[] extendedJointPositions = new float[8];

//         for (int i = 0; i < 5; i++)
//         {
//             extendedJointPositions[i] = jointPositions[i];
//         }

//         extendedJointPositions[7] = jointPositions[5];

//         extendedJointPositions[5] = jointPositions[5] - jointPositions[6] / 2;
//         extendedJointPositions[6] = jointPositions[5] + jointPositions[6] / 2;

//         base.DriveJoints(extendedJointPositions);
//     }

//     public override void DriveCartesianPose(PxTransformData t)
//     {
//         DriveCartesianPose(t, 0);
//     }

//     public void DriveCartesianPose(PxTransformData t, float angleGrasper)
//     {
//          float[] qInit = new float[7];
//         for (int i = 0; i < 5; i++)
//         {
//             qInit[i] = m_robot.JointPositions[i];
//         }
//         qInit[5] = m_robot.JointPositions[7];        
                
//         t.quaternion = t.quaternion * m_rotationTooltipToJoint;
//         bool result = Physx.GetRobotInverseKinematics(m_robot.NativeObjectPtr, ref qInit[0], ref t, 1e-3f, 100, 0.01f);
//         if (!result)
//         {
//             float[] qActual = m_robot.JointPositions;
//             string printStr = "q_actual: ";
//             for (int i = 0; i < 6; i++)
//             {
//                 printStr += qActual[i];
//                 printStr += ", ";
//             }
//             print(printStr);
//             printStr = "q_solved: ";
//             for (int i = 0; i < 6; i++)
//             {
//                 printStr += qInit[i];
//                 printStr += ", ";
//             }
//             print(printStr);
//             throw new Exception("IK Failed!");
//         }
//         // set grasper angle and drive the joints
//         qInit[6] = angleGrasper;
//         DriveJoints(qInit);
//     }

//     private double[] GetJointPositionsDouble()
//     {
//             float[] jointPositions = m_robot.JointPositions;
//             double[] jointPositionsDouble = new double[jointPositions.Length];
//             for (int i = 0; i < jointPositions.Length; i++)
//             {
//                 jointPositionsDouble[i] = jointPositions[i];
//             }
//             return jointPositionsDouble;
//     }

//     // Publish current joint positions over ROS as feedback
//     private void PublishRobotState()
//     {
//         float[] jointsFloat = m_robot.JointPositions;
//         double[] jointsDouble = GetJointPositionsDouble();

//         double timeAsDouble = Time.timeAsDouble;
//         uint seconds = (uint)timeAsDouble;
//         double fractionalSeconds = timeAsDouble - seconds;
//         uint nanoseconds = (uint)(fractionalSeconds * 1e9);

//         msg_JointState.header.stamp = new TimeMsg(seconds, nanoseconds);
//         msg_JointState.position = jointsDouble;
//         // Debug.Log("Publishing Joint positions: " + jointsFloat[0] + " " + jointsFloat[1] + " " + jointsFloat[2] + " " + jointsFloat[3] + " " + jointsFloat[4] + " " + jointsFloat[5] + " " + jointsFloat[6]);
//         m_rosConnection.Publish(topicJointState, msg_JointState);

//         // Update FK
//         PxTransformData currentEEPose = SolveFK(jointsFloat);

//         msg_CartesianState = new PoseStampedMsg();
//         msg_CartesianState.header = new HeaderMsg();
//         msg_CartesianState.header.stamp = new TimeMsg(seconds, nanoseconds);
//         msg_CartesianState.header.frame_id = PSMName;
//         msg_CartesianState.pose = new PoseMsg();
//         msg_CartesianState.pose.position = currentEEPose.position.To<FLU>();
//         msg_CartesianState.pose.orientation = currentEEPose.quaternion.To<FLU>();
//         // Debug.Log("Publishing Cartesian pose: " + msg_CartesianState.pose.position + " " + msg_CartesianState.pose.orientation);
//         m_rosConnection.Publish(topicCartesianState, msg_CartesianState);

//     }

//     // TODO
//     // Callback for receiving Cartesian pose commands
//     private void CartesianCommandCallback(PoseStampedMsg msg)
//     {
//         PxTransformData targetPoseWorld = new PxTransformData(msg.pose.position.From<FLU>(), msg.pose.orientation.From<FLU>());
//         PxTransformData targetPose = new PxTransformData();
//         targetPose.position = targetPoseWorld.position - basePose.position;
//         targetPose.quaternion = targetPoseWorld.quaternion * Quaternion.Inverse(basePose.quaternion);

//         Debug.Log("Current EE pose: " + m_psmEETooltipJointOnSelf.position + " " + m_psmEETooltipJointOnSelf.rotation);

//         Debug.Log("Received Cartesian command: " + targetPose.position + " " + targetPose.quaternion);
//         DriveCartesianPose(targetPose, jawAngle);
//         jointPositionsIK = null;
//     }

//     // Callback to take ROS joint commands and update the local joint state
//     private void JointCommandCallback(JointStateMsg msg)
//     {
//         float[] newJoints = new float[msg.position.Length+1];
//         for (int i = 0; i < msg.position.Length; i++)
//         {
//             newJoints[i] = (float)msg.position[i];
//         }
//         newJoints[2] = -newJoints[2];
//         newJoints[6] = jawAngle;
//         Debug.Log("Received joint command: " + newJoints[0] + " " + newJoints[1] + " " + newJoints[2] + " " + newJoints[3] + " " + newJoints[4] + " " + newJoints[5] + " " + newJoints[6]);
//         DriveJoints(newJoints);
//     }

//     private void JawCommandCallback(JointStateMsg msg)
//     {
//         jawAngle = (float)msg.position[0];
//     }

//     private void SolveIK(PxTransformData targetPose)
//     {   
//         srvIKRequest = new QueryInverseKinematicsRequest();
        
//         srvIKRequest.jp = new double[] { 0, 0, 0, 0, 0, 0, 0 };
//         srvFKRequest.jp = GetJointPositionsDouble();

//         srvIKRequest.cp = new PoseMsg();
//         srvIKRequest.cp.position = targetPose.position.To<FLU>();
//         srvIKRequest.cp.orientation = targetPose.quaternion.To<FLU>();

//         m_rosConnection.SendServiceMessage<QueryInverseKinematicsResponse>(serviceIK, srvIKRequest, CallbackIKService);

//     }

//     private void CallbackIKService(QueryInverseKinematicsResponse response)
//     {        
//         jointPositionsIK = new float[response.jp.Length];
//         for (int i = 0; i < response.jp.Length; i++)
//         {
//             jointPositionsIK[i] = (float)response.jp[i];
//         }
//     }

//     private void GetEEPose()
//     {
//         PxTransformData eePose = new PxTransformData();
//         eePose.position = m_psmEETooltipJointOnSelf.position;
//         eePose.quaternion = m_psmEETooltipJointOnSelf.rotation;
//     }

//     private PxTransformData SolveFK(float[] jointPositions)
//     {
//         Matrix4x4 fk;
//         // Getting the end-effector pos by doing FK
//         // TODO: this is inefficient; use the current EE tooltip pose instead
//         Physx.GetRobotForwardKinematics(m_robot.NativeObjectPtr, ref jointPositions[0], out fk);

//         Quaternion currentRotation = MatrixToRotation(fk);
//         Vector3 currentPosition = MatrixToPosition(fk);

//         PxTransformData currentPose = new PxTransformData(currentPosition, currentRotation);
//         Debug.Log("Current pose: " + currentPose.position + " " + currentPose.quaternion);
//         return currentPose;


//         // double[] jointPositionsDouble = new double[jointPositions.Length];
//         // for (int i = 0; i < jointPositions.Length; i++)
//         // {
//         //     jointPositionsDouble[i] = (double)jointPositions[i];
//         // }
        
//         // srvFKRequest = new QueryForwardKinematicsRequest();
//         // srvFKRequest.jp = jointPositionsDouble;
//         // Debug.Log("Joint positions: " + jointPositionsDouble[0] + " " + jointPositionsDouble[1] + " " + jointPositionsDouble[2] + " " + jointPositionsDouble[3] + " " + jointPositionsDouble[4] + " " + jointPositionsDouble[5] + " " + jointPositionsDouble[6]);

//         // m_rosConnection.SendServiceMessage<QueryForwardKinematicsResponse>(serviceFK, srvFKRequest, callbackFKService);
//     }

//     private void callbackFKService(QueryForwardKinematicsResponse response)
//     {

//         cartesianPoseFK = new PxTransformData();
//         cartesianPoseFK.position = response.cp.position.From<FLU>();
//         cartesianPoseFK.quaternion = response.cp.orientation.From<FLU>();
//         Debug.Log(response.cp.position + " " + response.cp.orientation);


//     }

//        static Quaternion MatrixToRotation(Matrix4x4 m)
//     {
//         // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
//         Quaternion q = new Quaternion();
//         q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
//         q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
//         q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
//         q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
//         q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
//         q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
//         q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
//         return q;
//     }

//     static Vector3 MatrixToPosition(Matrix4x4 m)
//     {
//         Vector3 result = new Vector3();
//         result.x = m[0, 3];
//         result.y = m[1, 3];
//         result.z = m[2, 3];
//         return result;
//     }
// }
