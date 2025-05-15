using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;  // Add this for Time message

public class CameraPublisher : MonoBehaviour
{
    public RenderTexture renderTexture;
    ROSConnection ros;
    public string topicName = "/camera/image_raw"; // Ensure the ROS2 topic starts with a '/'

    // The frequency of publishing messages
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize the ROS connection for ROS2
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Create a Texture2D with the same dimensions as the RenderTexture
            Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
            RenderTexture.active = renderTexture;

            // Read the image data from the RenderTexture into the Texture2D
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);

            // Get the pixel data from the Texture2D
            Color32[] pixels = texture.GetPixels32();
            byte[] bytes = new byte[pixels.Length * 4];  // RGBA32 uses 4 bytes per pixel

            // Convert Color32 array to byte array
            for (int i = 0; i < pixels.Length; i++)
            {
                bytes[i * 4] = pixels[i].b;   // Swap Red & Blue
                bytes[i * 4 + 1] = pixels[i].g;
                bytes[i * 4 + 2] = pixels[i].r;
                bytes[i * 4 + 3] = pixels[i].a;
            }


            // Create the ROS2 message with the image data
            ImageMsg msg = new ImageMsg
            {
                header = new RosMessageTypes.Std.HeaderMsg
                {
                    stamp = new TimeMsg
                    {
                        sec = (int)(Time.timeSinceLevelLoad),  // Time since level started (in seconds)
                        nanosec = (uint)((Time.timeSinceLevelLoad % 1) * 1000000000)  // Nanoseconds
                    },
                    frame_id = "raw_camera_frame"  // Add a unique frame ID

                },
                height = (uint)renderTexture.height,
                width = (uint)renderTexture.width,
                encoding = "bgra8",  // Change encoding
                is_bigendian = 0,
                step = (uint)(renderTexture.width * 4),  // width * bytes per pixel
                data = bytes
            };

            // Publish the image message
            ros.Publish(topicName, msg);

            // Reset timer
            timeElapsed = 0;
        }
    }
}
