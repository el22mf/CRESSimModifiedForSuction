//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Cisst
{
    [Serializable]
    public class ConvertFloat64ArrayResponse : Message
    {
        public const string k_RosMessageName = "cisst_msgs/ConvertFloat64Array";
        public override string RosMessageName => k_RosMessageName;

        public double[] output;

        public ConvertFloat64ArrayResponse()
        {
            this.output = new double[0];
        }

        public ConvertFloat64ArrayResponse(double[] output)
        {
            this.output = output;
        }

        public static ConvertFloat64ArrayResponse Deserialize(MessageDeserializer deserializer) => new ConvertFloat64ArrayResponse(deserializer);

        private ConvertFloat64ArrayResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.output, sizeof(double), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.output);
            serializer.Write(this.output);
        }

        public override string ToString()
        {
            return "ConvertFloat64ArrayResponse: " +
            "\noutput: " + System.String.Join(", ", output.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
