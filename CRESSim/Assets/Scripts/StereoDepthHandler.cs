using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using System;

public class StereoDepthHandler : MonoBehaviour
{
    public Camera leftCamera;
    public Camera rightCamera;
    public RenderTexture leftRenderTexture;
    public RenderTexture rightRenderTexture;

    public string depthMap = "/stereo/depthsons";
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;

    public float focalLength = 0.8f;  // in pixels
    public float baseline = 0.6f;     // in meters

    private Texture2D leftTexture;
    private Texture2D rightTexture;
    private Texture2D downscaledDepthMap100x100;

    private ROSConnection ros;

    private float[] depthFloats;  // 320×240 float depth map

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Register publishers
        ros.RegisterPublisher<ImageMsg>(depthMap);

        // Register a subscriber on the same topic
        ros.Subscribe<ImageMsg>(depthMap, OnDepthReceived);

        // Allocate buffers
        leftTexture = new Texture2D(leftRenderTexture.width, leftRenderTexture.height, TextureFormat.RGBA32, false);
        rightTexture = new Texture2D(rightRenderTexture.width, rightRenderTexture.height, TextureFormat.RGBA32, false);
        depthFloats = new float[320 * 240];
        downscaledDepthMap100x100 = new Texture2D(100, 100, TextureFormat.RGBA32, false);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            CaptureCameraImages();
            CalculateDepth();
            PublishDepthToROS();
            DownscaleDepthMap();
            timeElapsed = 0f;
        }
    }

    void CaptureCameraImages()
    {
        RenderTexture.active = leftRenderTexture;
        leftTexture.ReadPixels(new Rect(0, 0, leftRenderTexture.width, leftRenderTexture.height), 0, 0);
        leftTexture.Apply();

        RenderTexture.active = rightRenderTexture;
        rightTexture.ReadPixels(new Rect(0, 0, rightRenderTexture.width, rightRenderTexture.height), 0, 0);
        rightTexture.Apply();
    }

    void CalculateDepth()
    {
        int w = Mathf.Min(320, leftTexture.width);
        int h = Mathf.Min(240, leftTexture.height);

        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
            {
                var l = leftTexture.GetPixel(x, y).r * 255f;
                var r = rightTexture.GetPixel(x, y).r * 255f;
                int disparity = Mathf.Abs((int)l - (int)r);
                float depth = disparity > 0 ? (focalLength * baseline) / disparity : 0f;
                depthFloats[y * 320 + x] = depth;
            }
    }

    void DownscaleDepthMap()
    {
        int ow = 320, oh = 240, nw = 100, nh = 100;
        for (int y = 0; y < nh; y++)
            for (int x = 0; x < nw; x++)
            {
                int ix = x * ow / nw, iy = y * oh / nh;
                float d = depthFloats[iy * 320 + ix];
                byte b = (byte)Mathf.Clamp(d * 255f, 0, 255);
                downscaledDepthMap100x100.SetPixel(x, y, new Color32(b, b, b, 255));
            }
        downscaledDepthMap100x100.Apply();
    }

    void PublishDepthToROS()
    {
        int w = 320, h = 240;
        byte[] bytes = new byte[w * h * 4];
        for (int i = 0; i < depthFloats.Length; i++)
            Buffer.BlockCopy(BitConverter.GetBytes(depthFloats[i]), 0, bytes, i * 4, 4);

        var depthMsg = CreateImageMsg(bytes, w, h);
        ros.Publish(depthMap, depthMsg);

    }

    ImageMsg CreateImageMsg(byte[] data, int w, int h)
    {
        return new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.timeSinceLevelLoad,
                    nanosec = (uint)((Time.timeSinceLevelLoad % 1f) * 1e9f)
                },
                frame_id = "stereo/depth_frame"
            },
            height = (uint)h,
            width = (uint)w,
            encoding = "32FC1",
            is_bigendian = 0,
            step = (uint)(w * 4),
            data = data
        };
    }


    void OnDepthReceived(ImageMsg msg)
    {
        int w = (int)msg.width;
        int h = (int)msg.height;
        int cx = w / 2, cy = h / 2;

        Debug.Log($"Received depth map {w}×{h}, showing 50 eg values around center:");

        int count = 0;
        int radius = 3;
        for (int dy = -radius; dy <= radius && count < 50; dy++)
        {
            for (int dx = -radius; dx <= radius && count < 50; dx++)
            {
                int x = cx + dx, y = cy + dy;
                if (x < 0 || x >= w || y < 0 || y >= h) continue;

                int idx = (y * w + x) * 4;
                float depth = BitConverter.ToSingle(msg.data, idx);
                Debug.Log($"  [{x},{y}] = {depth:F4} m");
                count++;
            }
        }

        if (count == 0)
            Debug.LogWarning("No valid samples found.");
    }
}
