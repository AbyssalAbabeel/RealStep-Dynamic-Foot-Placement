using UnityEngine;

/// <summary>
/// Moves the object back and forth between two points (left to right and back).
/// </summary>
namespace FootIKSystem
{
    public class PingPongMover : MonoBehaviour
    {
        [Header("Movement Settings")]
        public float moveDistance = 5f;    // Maximum distance from start point
        public float moveSpeed = 2f;       // Speed of movement

        private Vector3 startPoint;
        private Vector3 targetPoint;
        private bool movingRight = true;

        void Start()
        {
            // Save the initial position
            startPoint = transform.position;
            targetPoint = startPoint + Vector3.right * moveDistance;
        }

        void Update()
        {
            // Move toward the target point
            Vector3 target = movingRight ? targetPoint : startPoint;
            transform.position = Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);

            // When reached the target, switch direction
            if (Vector3.Distance(transform.position, target) < 0.01f)
            {
                movingRight = !movingRight;
            }
        }
    }
}