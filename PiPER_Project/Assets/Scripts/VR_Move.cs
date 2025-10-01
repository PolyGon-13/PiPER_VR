using UnityEngine;

public class VR_Move : MonoBehaviour
{
    [Header("VR")]
    public OVRHand rightHand;
    //public OVRHand leftHand;
    public bool vr_start = false;

    [Header("Finger")]
    public Transform thumbTip;
    public Transform indexTip;

    [Header("Gripper Joint")]
    public ArticulationBody joint7;
    public ArticulationBody joint8;

    [Header("Params")]
    float closeDistance = 0.02f;
    float maxDistance = 0.12f;
    public float maxTravel = 0.035f;
    public float followSmooth = 12f;

    void Start()
    {
        if (rightHand == null) Debug.LogError("[PiperIK] OVRHand가 할당되지 않았습니다.");
        if (thumbTip == null || indexTip == null) Debug.LogError("[VR_Move] 손가락이 할당되지 않았습니다.");
        if (joint7 == null || joint8 == null) Debug.LogError("[VR_Move] 그리퍼 관절이 할당되지 않았습니다.");
    }

    void FixedUpdate()
    {
        if (!thumbTip || !indexTip || !joint7 || !joint8) return;

        if (rightHand != null && rightHand.IsTracked)
        {
            vr_start = true;
            Gripper_Move_By_Finger();
        }
        else
            vr_start = false;
    }

    void Gripper_Move_By_Finger()
    {
        float dist = Vector3.Distance(thumbTip.position, indexTip.position); // 엄지와 검지 사이 거리

        float t = Mathf.InverseLerp(closeDistance, maxDistance, dist);
        t = Mathf.Clamp01(t);

        var d7 = joint7.xDrive;
        var d8 = joint8.xDrive;

        float target7 = Mathf.Lerp(d7.lowerLimit, d7.upperLimit, t);
        float target8 = Mathf.Lerp(d8.upperLimit, d8.lowerLimit, t);

        SetDrive(joint7, target7);
        SetDrive(joint8, target8);
    }

    void SetDrive(ArticulationBody ab, float target)
    {
        var drive = ab.xDrive;

        target = Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);

        float dt = Time.deltaTime;
        float newTarget = Mathf.Lerp(drive.target, target, 1f - Mathf.Exp(-followSmooth * dt));

        drive.target = newTarget;
        ab.xDrive = drive;
    }
}