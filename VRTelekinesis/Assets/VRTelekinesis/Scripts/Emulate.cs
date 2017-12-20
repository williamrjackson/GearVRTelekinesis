using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Emulate : MonoBehaviour {
#if UNITY_EDITOR
    public GameObject rightClick;
    GameObject active;
    private float speed = 100F;
    private float rotationSpeed = 2.0F;
    private bool captureMouse = false;
    float pitch;
    float yaw;  // Update is called once per frame

    void Update () {
        float translationZ = Input.GetAxisRaw("Vertical") * speed;
        float translationX = Input.GetAxisRaw("Horizontal") * speed;
        translationZ *= Time.deltaTime;
        translationX *= Time.deltaTime;

        if (Input.GetMouseButtonDown(1))
        {
            active = rightClick;
            captureMouse = !captureMouse;
            if (captureMouse)
            {
                Cursor.lockState = CursorLockMode.Locked;
            }
            else
            {
                Cursor.lockState = CursorLockMode.None;
            }
        }

        if (captureMouse)
        {
            pitch += rotationSpeed * Input.GetAxis("Mouse Y");
            yaw += rotationSpeed * Input.GetAxis("Mouse X");

            pitch = Mathf.Clamp(pitch, -90f, 90f);

            yaw %= 360f;

            active.transform.eulerAngles = new Vector3(-pitch, yaw, 0f);
        }

    }
#endif
}
