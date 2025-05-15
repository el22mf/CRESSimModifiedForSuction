using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

public class LeftAndRightEyeCameraPublisher : MonoBehaviour
{
    public RenderTexture leftRenderTexture;
    public RenderTexture rightRenderTexture;

    public string leftImageTopic = "/left_camera/image_raw";
    public string rightImageTopic = "/right_camera/image_raw";
    public string leftInfoTopic = "/left_camera/camera_info";
    public string rightInfoTopic = "/right_camera/camera_info";

    public string leftFrameId = "left_eye_camera_frame";
    public string rightFrameId = "right_eye_camera_frame";

    public float publishMessageFrequency = 10f;

    private ROSConnection ros;
    private float timeElapsed;

    private const double fx = 257.6;
    private const double fy = 193.2;
    private const double cx = 160.0;
    private const double cy = 120.0;
    private const double baseline = 0.6;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(leftImageTopic);
        ros.RegisterPublisher<ImageMsg>(rightImageTopic);
        ros.RegisterPublisher<CameraInfoMsg>(leftInfoTopic);
        ros.RegisterPublisher<CameraInfoMsg>(rightInfoTopic);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1.0f / publishMessageFrequency)
        {
            // Generate ROS 2 timestamp using Unity time
            TimeMsg stamp = new TimeMsg
            {
                sec = (int)(Time.timeSinceLevelLoad),
                nanosec = (uint)((Time.timeSinceLevelLoad % 1) * 1e9)
            };

            // Publish images
            PublishCameraImage(leftRenderTexture, leftImageTopic, leftFrameId, stamp);
            PublishCameraImage(rightRenderTexture, rightImageTopic, rightFrameId, stamp);

            // Publish camera info
            PublishCameraInfo(leftInfoTopic, leftFrameId, leftRenderTexture.width, leftRenderTexture.height, fx, fy, cx, cy, 0.0, stamp);
            PublishCameraInfo(rightInfoTopic, rightFrameId, rightRenderTexture.width, rightRenderTexture.height, fx, fy, cx, cy, -fx * baseline, stamp);

            timeElapsed = 0;
        }
    }

    private void PublishCameraImage(RenderTexture renderTexture, string topicName, string frameId, TimeMsg stamp)
    {
        Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
        RenderTexture.active = renderTexture;
        texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
        RenderTexture.active = null;

        Color32[] originalPixels = texture.GetPixels32();
        Color32[] flippedPixels = new Color32[originalPixels.Length];
        int width = texture.width;
        int height = texture.height;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int srcIndex = y * width + x;
                int dstIndex = y * width + (width - 1 - x); // horizontal flip
                flippedPixels[dstIndex] = originalPixels[srcIndex];
            }
        }

        byte[] bytes = new byte[flippedPixels.Length * 3];
        for (int i = 0; i < flippedPixels.Length; i++)
        {
            bytes[i * 3] = flippedPixels[i].b;
            bytes[i * 3 + 1] = flippedPixels[i].g;
            bytes[i * 3 + 2] = flippedPixels[i].r;
        }


        ImageMsg msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = stamp,
                frame_id = frameId
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "bgr8",
            step = (uint)(width * 3),
            is_bigendian = 0,
            data = bytes
        };

        ros.Publish(topicName, msg);
    }

    private void PublishCameraInfo(string topicName, string frameId, int width, int height, double fx, double fy, double cx, double cy, double tx, TimeMsg stamp)
    {
        CameraInfoMsg msg = new CameraInfoMsg
        {
            header = new HeaderMsg
            {
                stamp = stamp,
                frame_id = frameId
            },
            width = (uint)width,
            height = (uint)height,
            distortion_model = "plumb_bob",
            d = new double[5] { 0, 0, 0, 0, 0 },
            k = new double[9] {
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1
            },
            r = new double[9] {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            },
            p = new double[12] {
                fx, 0, cx, tx,
                0, fy, cy, 0,
                0, 0, 1, 0
            },
            binning_x = 0,
            binning_y = 0,
            roi = new RegionOfInterestMsg()
        };

        ros.Publish(topicName, msg);
    }
}
