using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PointPublisher : UnityPublisher<MessageTypes.Geometry.Point>
    {
        public Transform PublishedTransform;
        private Vector3 startingPosition;
        public string FrameId = "Unity";

        private MessageTypes.Geometry.Point message;
        private bool isTrackingMovement;
        [SerializeField] public Vector3 _message;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Geometry.Point();
            isTrackingMovement = false;
        }

        private void FixedUpdate()
        {
            OVRInput.FixedUpdate();
            if (OVRInput.GetDown(OVRInput.Button.PrimaryHandTrigger)) {
                if (isTrackingMovement = !isTrackingMovement) {
                    startingPosition = PublishedTransform.position.Unity2Ros();
                };
            }
            if (isTrackingMovement) UpdateMessage();
        }

        private void UpdateMessage()
        {
            GetGeometryPoint(PublishedTransform.position.Unity2Ros(), message);
            Publish(message);
        }

        private void GetGeometryPoint(Vector3 position, MessageTypes.Geometry.Point geometryPoint)
        {
            Vector3 deltaPosition = position - startingPosition;
            geometryPoint.x = deltaPosition.x;
            geometryPoint.y = deltaPosition.y;
            geometryPoint.z = deltaPosition.z;
            _message.x = deltaPosition.x;
            _message.y = deltaPosition.y;
            _message.z = deltaPosition.z;
        }
    }
}
