using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class GrabPublisher : UnityPublisher<MessageTypes.Std.Bool>
    {
        public string FrameId = "Unity";
        private MessageTypes.Std.Bool message;
        [SerializeField] bool _message;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Std.Bool();
        }

        private void FixedUpdate()
        {
            OVRInput.FixedUpdate();
            message.data = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger);
            _message = message.data;
            Publish(message);
        }
    }
}
