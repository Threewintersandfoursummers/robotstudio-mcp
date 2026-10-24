using System;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio.Stations;

namespace RsMcpBridge
{
    /// <summary>
    /// Damped least squares inverse kinematics built on top of Mechanism.CalculateForwardKinematics.
    /// RobotStudio 6.08 without a virtual controller has no controller kinematics service,
    /// which makes Mechanism.CalculateInverseKinematics throw. This numeric solver covers that gap.
    /// </summary>
    internal static class NumericIk
    {
        const double Deg2Rad = Math.PI / 180.0;

        internal static bool Solve(Mechanism m, double[] targetMm, double[] targetEulerDeg,
                                   double[] seed, out double[] joints, out string note)
        {
            note = null;
            joints = (double[])seed.Clone();

            Matrix4 target = new Matrix4(
                new Vector3(targetMm[0] / 1000.0, targetMm[1] / 1000.0, targetMm[2] / 1000.0),
                new Vector3(targetEulerDeg[0] * Deg2Rad, targetEulerDeg[1] * Deg2Rad, targetEulerDeg[2] * Deg2Rad));

            ForwardKinematicsResult fres;
            int n = joints.Length;
            double lambda = 1e-3;

            for (int iter = 0; iter < 400; iter++)
            {
                Matrix4 fk = m.CalculateForwardKinematics(joints, out fres);
                if (fk.IsNaN) { note = "forward kinematics returned NaN"; return false; }

                double[] err = Error(target, fk);
                double norm = Norm(err);
                if (norm < 1e-5) { note = "converged in " + iter + " iterations"; return true; }

                // numeric Jacobian of the 6-D error w.r.t. joints
                double[,] jac = new double[6, n];
                for (int i = 0; i < n; i++)
                {
                    double[] qp = (double[])joints.Clone();
                    double h = 1e-6;
                    qp[i] += h;
                    Matrix4 fk2 = m.CalculateForwardKinematics(qp, out fres);
                    if (fk2.IsNaN) continue;
                    double[] err2 = Error(target, fk2);
                    for (int k = 0; k < 6; k++) jac[k, i] = (err2[k] - err[k]) / h;
                }

                // dq = (J^T J + lambda I)^-1 J^T err
                double[,] jtj = new double[n, n];
                double[] jte = new double[n];
                for (int a = 0; a < n; a++)
                {
                    for (int b = 0; b < n; b++)
                    {
                        double s = 0;
                        for (int k = 0; k < 6; k++) s += jac[k, a] * jac[k, b];
                        jtj[a, b] = s + (a == b ? lambda : 0.0);
                    }
                    double t = 0;
                    for (int k = 0; k < 6; k++) t += jac[k, a] * err[k];
                    jte[a] = t;
                }

                double[] dq = SolveLinear(jtj, jte);
                if (dq == null) { note = "singular jacobian"; return false; }

                double maxStep = 0;
                for (int i = 0; i < n; i++)
                {
                    if (dq[i] > 0.25) dq[i] = 0.25;
                    if (dq[i] < -0.25) dq[i] = -0.25;
                    joints[i] += dq[i];
                    if (Math.Abs(dq[i]) > maxStep) maxStep = Math.Abs(dq[i]);
                }

                if (maxStep < 1e-9 && norm > 1e-3) { note = "stalled, residual " + Math.Round(norm, 5); return false; }
            }

            note = "max iterations";
            return false;
        }

        /// <summary>6-D error: world-frame translation (m) then rotation vector (rad).</summary>
        static double[] Error(Matrix4 target, Matrix4 current)
        {
            var e = new double[6];
            Vector3 pt = target.Translation;
            Vector3 pc = current.Translation;
            e[0] = pt.x - pc.x;
            e[1] = pt.y - pc.y;
            e[2] = pt.z - pc.z;

            double[] qt = Quat(target);
            double[] qc = Quat(current);
            double[] qe = Mul(qt, Conj(qc));   // rotation that takes current onto target
            if (qe[0] < 0) { qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]; }

            double w = Math.Min(1.0, Math.Max(-1.0, qe[0]));
            double angle = 2.0 * Math.Acos(w);
            double s = Math.Sqrt(Math.Max(1e-12, 1.0 - w * w));
            double ax = qe[1] / s, ay = qe[2] / s, az = qe[3] / s;
            if (angle > Math.PI) angle -= 2 * Math.PI;

            e[3] = ax * angle;
            e[4] = ay * angle;
            e[5] = az * angle;
            return e;
        }

        static double[] Quat(Matrix4 m)
        {
            Quaternion q = m.Quaternion;   // (w, x, y, z)
            return new double[] { q[0], q[1], q[2], q[3] };
        }

        static double[] Conj(double[] q)
        {
            return new double[] { q[0], -q[1], -q[2], -q[3] };
        }

        static double[] Mul(double[] a, double[] b)
        {
            return new double[]
            {
                a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
            };
        }

        static double Norm(double[] v)
        {
            double s = 0;
            foreach (double x in v) s += x * x;
            return Math.Sqrt(s);
        }

        static double[] SolveLinear(double[,] a, double[] b)
        {
            int n = b.Length;
            var m = new double[n, n + 1];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) m[i, j] = a[i, j];
                m[i, n] = b[i];
            }

            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(m[r, col]) > Math.Abs(m[pivot, col])) pivot = r;
                if (Math.Abs(m[pivot, col]) < 1e-12) return null;

                if (pivot != col)
                    for (int j = col; j <= n; j++)
                    {
                        double t = m[col, j]; m[col, j] = m[pivot, j]; m[pivot, j] = t;
                    }

                for (int r = 0; r < n; r++)
                {
                    if (r == col) continue;
                    double f = m[r, col] / m[col, col];
                    if (f == 0) continue;
                    for (int j = col; j <= n; j++) m[r, j] -= f * m[col, j];
                }
            }

            var x = new double[n];
            for (int i = 0; i < n; i++) x[i] = m[i, n] / m[i, i];
            return x;
        }
    }
}
