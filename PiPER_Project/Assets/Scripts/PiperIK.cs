using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class PiperIK : MonoBehaviour
{
    [Header("Ref Code")]
    public VR_Move vr;

    [Header("Joint")]
    public ArticulationBody[] joint = new ArticulationBody[6];
    public ArticulationBody jawLeft;
    public ArticulationBody jawRight;
    private float[] angle = new float[6];

    [Header("Target")]
    public GameObject target;
    public GameObject ee_Target;

    private float?[] minAngle = new float?[6];
    private float?[] maxAngle = new float?[6];

    private float[] minAngleVelocity = new float[6];
    private float[] maxAngleVelocity = new float[6];

    private float d_tolerance = 0.01f;
    private float r_tolerance = 0.5f;
    private int maxIterations = 10;
    private float[] preAngle = new float[6];

    private float sigma = 0.5f;
    private float mu = 0f;

    // 목표 각도 반환
    public float[] GetJointAnglesDeg()
    {
        return (float[])angle.Clone();
    }

    // 현재 각도 반환
    public float[] GetCurrentAnglesDeg()
    {
        if (joint == null) return null;

        int n = joint.Length; // 관절 개수
        var current_deg = new float[n];

        for (int i = 0; i < n; i++)
        {
            var ab = joint[i];
            if (ab == null)
            {
                Debug.LogWarning($"[PiperIK] joint[{i}] 미할당");
                current_deg[i] = 0f;
                continue;
            }

            var jp = ab.jointPosition;
            if (jp.dofCount <= 0)
            {
                current_deg[i] = 0f;
                Debug.LogWarning($"[PiperIK] joint[{i}]의 자유도가 0");
                continue;
            }

            current_deg[i] = jp[0] * Mathf.Rad2Deg; // 라디안을 도 단위로 변환
        }
        return current_deg;
    }

    // 그리퍼의 개방폭 (단위: m)
    public float GetGripperPosition()
    {
        if (jawLeft == null && jawRight == null) return 0f;

        float Read_AB(ArticulationBody ab)
        {
            if (ab == null) return 0f;

            var d = ab.xDrive;
            if (float.IsFinite(d.target)) return d.target; // target 값이 유효한지 확인

            var jp = ab.jointPosition; // 현재 조인트의 상태를 읽음
            if (jp.dofCount > 0 && float.IsFinite((float)jp[0])) return (float)jp[0];
            // dofCount : 조인트의 자유도 수 (Revolute=1, Fixed=0, Spherical=3, Prismatic=1)
            // jp[0] : 조인트의 첫 번째 자유도 값
            Debug.LogWarning("[PiperIK] 그리퍼 조인트의 위치를 읽을 수 없습니다.");
            return 0f;
        }

        // 좌우 그리퍼의 위치를 읽음
        float l = Read_AB(jawLeft);
        float r = Read_AB(jawRight);

        float gap = Mathf.Abs(l - r);
        float unity_jaw_max_gap = 0.035f * 2f; // 유니티 그리퍼의 최대 개방폭

        if (gap < 0f) gap = 0f;
        if (unity_jaw_max_gap > 0f) gap = Mathf.Min(gap, unity_jaw_max_gap);

        return gap;
    }

    void Start()
    {
        if (ee_Target == null || target == null) Debug.LogError("[PiperIK] target이 할당되지 않았습니다.");

        for (int i = 0; i < joint.Length; i++)
        {
            var ab = joint[i];

            if (ab == null)
            {
                Debug.LogError($"[PiperIK] joint[{i}] 미할당 → IK 비활성화");
                return;
            }

            var j = ab.xDrive;
            angle[i] = j.target;
            preAngle[i] = j.target;

            if (j.lowerLimit < j.upperLimit)
            {
                minAngle[i] = j.lowerLimit;
                maxAngle[i] = j.upperLimit;
            }
            else
            {
                minAngle[i] = null;
                maxAngle[i] = null;
            }
        }

        SetLimitVelocity(-60f, 60f);
    }

    void FixedUpdate()
    {
        if (ee_Target == null || target == null) return;

        if (target.CompareTag("Target"))
        {
            SolveIK();
        }
        else if (target.CompareTag("Target_VR"))
        {
            if (vr.vr_start)
                SolveIK();
        }
    }

    void SolveIK()
    {
        for (int iterations = 0; iterations < maxIterations; iterations++)
        {
            bool reachedTarget = true;

            Matrix<double> jacobian = CalculateJacobian();
            Vector<double> positionError = CalculatePositionError();
            Vector<double> rotationError = CalculateRotationError();

            Vector<double> jointVelocities = GDLSWithSVD(jacobian, positionError, rotationError);

            for (int i = 0; i < 6; i++)
            {
                float angleChange = (float)jointVelocities[i] * Time.deltaTime * Mathf.Rad2Deg;
                angleChange = Mathf.Clamp(angleChange, minAngleVelocity[i] * Time.deltaTime, maxAngleVelocity[i] * Time.deltaTime);

                float newAngle = preAngle[i] + angleChange;

                if (minAngle[i].HasValue && maxAngle[i].HasValue)
                    newAngle = Mathf.Clamp(newAngle, minAngle[i].Value, maxAngle[i].Value);

                var d = joint[i].xDrive;

                float maxDegPerSec = Mathf.Abs(maxAngleVelocity[i]);
                float smoothed = Mathf.MoveTowardsAngle(d.target, newAngle, maxDegPerSec * Time.deltaTime);
                d.target = smoothed;
                joint[i].xDrive = d;

                preAngle[i] = d.target;
                angle[i] = d.target;

                if (Vector3.Distance(ee_Target.transform.position, target.transform.position) >= d_tolerance ||
                    Quaternion.Angle(ee_Target.transform.rotation, target.transform.rotation) >= r_tolerance)
                {
                    reachedTarget = false;
                }
            }

            if (reachedTarget)
            {
                return;
            }
        }
    }

    Matrix<double> CalculateJacobian()
    {
        Matrix<double> jacobian = DenseMatrix.OfArray(new double[6, 6]);
        for (int i = 0; i < 6; i++)
        {
            Vector3 jointPosition = joint[i].transform.TransformPoint(joint[i].anchorPosition);
            Vector3 toEndEffector = ee_Target.transform.position - jointPosition;
            Vector3 axis = (joint[i].transform.rotation * joint[i].anchorRotation) * Vector3.right;
            axis.Normalize();
            Vector3 cross = Vector3.Cross(axis, toEndEffector);

            jacobian[0, i] = cross.x;
            jacobian[1, i] = cross.y;
            jacobian[2, i] = cross.z;
            jacobian[3, i] = axis.x;
            jacobian[4, i] = axis.y;
            jacobian[5, i] = axis.z;
        }
        return jacobian;
    }

    Vector<double> CalculatePositionError()
    {
        Vector3 positionError = target.transform.position - ee_Target.transform.position;
        return DenseVector.OfArray(new double[] { positionError.x, positionError.y, positionError.z, 0, 0, 0 });
    }

    Vector<double> CalculateRotationError()
    {
        Quaternion targetRotation = target.transform.rotation;
        Quaternion currentRotation = ee_Target.transform.rotation;
        Quaternion rotationDiff = targetRotation * Quaternion.Inverse(currentRotation);
        Vector3 rotationError = rotationDiff.eulerAngles;

        rotationError.x = WrapAngle(rotationError.x);
        rotationError.y = WrapAngle(rotationError.y);
        rotationError.z = WrapAngle(rotationError.z);

        return DenseVector.OfArray(new double[] { 0, 0, 0, rotationError.x, rotationError.y, rotationError.z });
    }

    float WrapAngle(float angle)
    {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        else if (angle < -180) angle += 360;
        return angle;
    }

    Vector<double> GDLSWithSVD(Matrix<double> jacobian, Vector<double> positionError, Vector<double> rotationError)
    {
        double wPos = 1.0;
        double wRot = 0.3;

        double rx_rad = Mathf.Deg2Rad * (float)rotationError[3];
        double ry_rad = Mathf.Deg2Rad * (float)rotationError[4];
        double rz_rad = Mathf.Deg2Rad * (float)rotationError[5];

        var e = DenseVector.OfArray(new double[] {
            wPos * positionError[0],
            wPos * positionError[1],
            wPos * positionError[2],
            wRot * rx_rad,
            wRot * ry_rad,
            wRot * rz_rad
        });

        var svd = jacobian.Svd();
        Matrix<double> U = svd.U;
        Vector<double> S = svd.S.Clone();
        Matrix<double> V = svd.VT.Transpose();

        double lambda = CalculateGaussianDampingFactor(positionError);

        for (int i = 0; i < S.Count; i++)
        {
            double si = S[i];
            S[i] = si / (si * si + lambda * lambda);
        }

        Matrix<double> dampedS = DiagonalMatrix.OfDiagonal(S.Count, S.Count, S);
        Matrix<double> dampedJacobian = V * dampedS * U.Transpose();
        Vector<double> dampedVelocity = dampedJacobian * e;

        return dampedVelocity;
    }

    double CalculateGaussianDampingFactor(Vector<double> positionError)
    {
        double errorMagnitude = positionError.SubVector(0, 3).L2Norm();
        double sigma = this.sigma;
        double mu = this.mu;

        double lambda = Mathf.Exp(-(float)(errorMagnitude - mu) * (float)(errorMagnitude - mu) / (2.0f * (float)sigma * (float)sigma));
        return lambda;
    }

    void SetLimitVelocity(float min, float max)
    {
        for (int i = 0; i < joint.Length; i++)
        {
            minAngleVelocity[i] = min;
            maxAngleVelocity[i] = max;
        }
    }
}
