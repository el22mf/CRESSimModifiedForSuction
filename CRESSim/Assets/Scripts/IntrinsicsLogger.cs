using System.IO;
using UnityEngine;

public class IntrinsicsLogger : MonoBehaviour
{
    public Camera cam; // Optional: assign manually
    public string fileName = "intrinsics_log.txt"; // Output file

    void Start()
    {
        if (cam == null)
            cam = GetComponent<Camera>();

        if (cam == null)
        {
            Debug.LogError("No camera assigned to IntrinsicsLogger.");
            return;
        }

        string path = Path.Combine(Application.dataPath, "..", fileName);

        using (StreamWriter writer = new StreamWriter(path, false))
        {
            writer.WriteLine("Camera Intrinsic Parameters:");
            writer.WriteLine("Focal Length: " + cam.focalLength);
            writer.WriteLine("Field of View: " + cam.fieldOfView);
            writer.WriteLine("Near Clip Plane: " + cam.nearClipPlane);
            writer.WriteLine("Far Clip Plane: " + cam.farClipPlane);
            writer.WriteLine("Sensor Size: " + cam.sensorSize);
            writer.WriteLine("Lens Shift: " + cam.lensShift);
            writer.WriteLine("Resolution: " + cam.pixelWidth + " x " + cam.pixelHeight);
            writer.WriteLine();

            // Calculate intrinsic matrix
            float[,] K = GetIntrinsicMatrix(cam);
            writer.WriteLine("Intrinsic Matrix (row-major):");
            for (int i = 0; i < 3; i++)
                writer.WriteLine($"{K[i, 0]:F4}\t{K[i, 1]:F4}\t{K[i, 2]:F4}");
        }

        Debug.Log("Camera intrinsics written to: " + path);
    }

    float[,] GetIntrinsicMatrix(Camera cam)
    {
        float pixelAspect = (float)cam.pixelWidth / (float)cam.pixelHeight;

        float alpha_u = cam.focalLength * ((float)cam.pixelWidth / cam.sensorSize.x);
        float alpha_v = cam.focalLength * pixelAspect * ((float)cam.pixelHeight / cam.sensorSize.y);

        float u_0 = (float)cam.pixelWidth / 2f;
        float v_0 = (float)cam.pixelHeight / 2f;

        return new float[3, 3]
        {
            { alpha_u,    0f,     u_0 },
            {    0f,    alpha_v,  v_0 },
            {    0f,      0f,     1f  }
        };
    }
}
