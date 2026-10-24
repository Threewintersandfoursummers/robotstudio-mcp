using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;

namespace RsMcpBridge
{
    internal static class Commands
    {
        const double Rad2Deg = 180.0 / Math.PI;
        const double Deg2Rad = Math.PI / 180.0;

        internal static object Execute(string cmd, Dictionary<string, object> a)
        {
            switch (cmd)
            {
                // ---------------- system ----------------
                case "ping": return Ping();
                case "rs.reflect": return Reflect(a);
                case "debug.transform": return DebugTransform();
                case "debug.ik": return DebugIk(a);
                case "debug.render": return DebugRender(a);

                // ---------------- station ----------------
                case "station.new": return StationNew(a);
                case "station.open": return StationOpen(a);
                case "station.save": return StationSave(a);
                case "station.close": return StationClose(a);
                case "station.info": return StationInfo();

                // ---------------- robots / mechanisms ----------------
                case "robot.list": return RobotList();
                case "robot.add": return RobotAdd(a);
                case "robot.joints.get": return RobotJointsGet(a);
                case "robot.joints.set": return RobotJointsSet(a);
                case "robot.home": return RobotHome(a);
                case "robot.fk": return RobotFk(a);
                case "robot.ik": return RobotIk(a);
                case "robot.reach": return RobotReach(a);
                case "robot.limits": return RobotLimits(a);

                // ---------------- component library ----------------
                case "lib.robots": return LibRobots(a);

                // ---------------- RAPID model ----------------
                case "rapid.tasks": return RapidTasks();
                case "rapid.targets": return RapidTargets(a);
                case "rapid.add_target": return RapidAddTarget(a);
                case "rapid.add_routine": return RapidAddRoutine(a);
                case "rapid.add_move": return RapidAddMove(a);
                case "rapid.code": return RapidCode(a);

                // ---------------- simulation ----------------
                case "sim.state": return SimState();
                case "sim.start": Simulator.Start(); return SimState();
                case "sim.stop": Simulator.Stop(); return SimState();
                case "sim.pause": Simulator.Pause(); return SimState();
                case "sim.resume": Simulator.Resume(); return SimState();
                case "sim.step": Simulator.Step(); return SimState();
                case "sim.speed": Simulator.SimulationSpeed = Bridge.Dbl(a, "value", 1.0); return SimState();

                // ---------------- IO ----------------
                case "io.list": return IoList(a);
                case "io.get": return IoGet(a);
                case "io.set": return IoSet(a);

                default:
                    throw new InvalidOperationException("Unknown command: " + cmd);
            }
        }

        // ================= helpers =================

        static Station Stn
        {
            get
            {
                Station s = Station.ActiveStation;
                if (s == null)
                    throw new InvalidOperationException(
                        "No active station. Use station.new or station.open first.");
                return s;
            }
        }

        static Station TryStn()
        {
            try { return Station.ActiveStation; }
            catch { return null; }
        }

        static Mechanism[] Mechs(Station s)
        {
            var list = new List<Mechanism>();
            foreach (GraphicComponent gc in s.GraphicComponents.ToArray())
            {
                Mechanism m = gc as Mechanism;
                if (m != null) list.Add(m);
            }
            return list.ToArray();
        }

        static Mechanism GetMech(Station s, string name)
        {
            Mechanism[] all = Mechs(s);
            if (all.Length == 0)
                throw new InvalidOperationException("Station has no robot/mechanism.");

            if (string.IsNullOrEmpty(name))
            {
                if (all.Length == 1) return all[0];
                throw new InvalidOperationException("Multiple mechanisms, specify 'robot'. Available: " +
                    string.Join(", ", all.Select(m => m.Name).ToArray()));
            }

            foreach (Mechanism m in all)
            {
                if (string.Equals(m.Name, name, StringComparison.OrdinalIgnoreCase)) return m;
                if (string.Equals(m.ModelName, name, StringComparison.OrdinalIgnoreCase)) return m;
            }
            throw new InvalidOperationException("Mechanism not found: " + name + ". Available: " +
                string.Join(", ", all.Select(m => m.Name).ToArray()));
        }

        static RsTask GetTask(Station s, string name)
        {
            if (s.ActiveTask != null && string.IsNullOrEmpty(name)) return s.ActiveTask;
            if (string.IsNullOrEmpty(name))
                throw new InvalidOperationException("No active RAPID task.");

            var ctrl = s.Irc5Controllers.ToArray();
            foreach (RsIrc5Controller c in ctrl)
            {
                foreach (RsTask t in c.Tasks.ToArray())
                    if (string.Equals(t.Name, name, StringComparison.OrdinalIgnoreCase)) return t;
            }
            foreach (RsTask t in VariantTasks(s))
                if (string.Equals(NameOf(t), name, StringComparison.OrdinalIgnoreCase)) return t;
            if (s.ActiveTask != null && string.Equals(s.ActiveTask.Name, name, StringComparison.OrdinalIgnoreCase))
                return s.ActiveTask;
            throw new InvalidOperationException("RAPID task not found: " + name);
        }

        static string NameOf(RsTask t)
        {
            if (t == null) return null;
            string n = null;
            try { n = t.DisplayName; } catch { }
            if (string.IsNullOrEmpty(n)) n = t.Name;
            return n;
        }

        static string NameOf(Mechanism m)
        {
            if (m == null) return null;
            string n = null;
            try { n = m.DisplayName; } catch { }
            if (string.IsNullOrEmpty(n)) n = m.Name;
            return n;
        }

        static double[] ToDeg(double[] rad)
        {
            var r = new double[rad.Length];
            for (int i = 0; i < rad.Length; i++) r[i] = rad[i] * Rad2Deg;
            return r;
        }

        static double[] ToRad(double[] deg)
        {
            var r = new double[deg.Length];
            for (int i = 0; i < deg.Length; i++) r[i] = deg[i] * Deg2Rad;
            return r;
        }

        static object Round(double v)
        {
            double r = Math.Round(v, 4);
            return r;
        }

        static object RoundAll(double[] v)
        {
            var l = new List<object>();
            foreach (double d in v) l.Add(Round(d));
            return l;
        }

        static Dictionary<string, object> Pose(Transform t)
        {
            // Transform: X/Y/Z in mm, RX/RY/RZ in RADIANS (Euler ZYX).
            var d = new Dictionary<string, object>();
            d["x"] = Round(t.X);
            d["y"] = Round(t.Y);
            d["z"] = Round(t.Z);
            d["rx"] = Round(t.RX * Rad2Deg);
            d["ry"] = Round(t.RY * Rad2Deg);
            d["rz"] = Round(t.RZ * Rad2Deg);
            return d;
        }

        static Dictionary<string, object> Pose(Matrix4 m)
        {
            Vector3 p = m.Translation;
            Vector3 e = m.EulerZYX;
            var d = new Dictionary<string, object>();
            d["x"] = Round(p.x * 1000.0);
            d["y"] = Round(p.y * 1000.0);
            d["z"] = Round(p.z * 1000.0);
            d["rx"] = Round(e.x * Rad2Deg);
            d["ry"] = Round(e.y * Rad2Deg);
            d["rz"] = Round(e.z * Rad2Deg);
            return d;
        }

        static void ApplyPose(Transform t, Dictionary<string, object> pose)
        {
            if (pose == null) return;
            if (pose.ContainsKey("x")) t.X = Convert.ToDouble(pose["x"], CultureInfo.InvariantCulture);
            if (pose.ContainsKey("y")) t.Y = Convert.ToDouble(pose["y"], CultureInfo.InvariantCulture);
            if (pose.ContainsKey("z")) t.Z = Convert.ToDouble(pose["z"], CultureInfo.InvariantCulture);
            if (pose.ContainsKey("rx")) t.RX = Convert.ToDouble(pose["rx"], CultureInfo.InvariantCulture) * Deg2Rad;
            if (pose.ContainsKey("ry")) t.RY = Convert.ToDouble(pose["ry"], CultureInfo.InvariantCulture) * Deg2Rad;
            if (pose.ContainsKey("rz")) t.RZ = Convert.ToDouble(pose["rz"], CultureInfo.InvariantCulture) * Deg2Rad;
        }

        // ================= system =================

        static object Ping()
        {
            Station s = TryStn();
            var d = new Dictionary<string, object>();
            d["bridge"] = "RsMcpBridge 1.0";
            d["port"] = Bridge.Port;
            d["robotstudio"] = typeof(Station).Assembly.GetName().Version.ToString();
            d["uiThread"] = Bridge.Ui != null;
            d["station"] = s == null ? null : s.Name;
            d["simState"] = Simulator.State.ToString();
            d["dataDirectory"] = RobotStudioAPI.DataDirectory;
            return d;
        }

        static object DebugTransform()
        {
            var d = new Dictionary<string, object>();
            RsRobTarget rt = new RsRobTarget();
            rt.Frame.X = 100; rt.Frame.Y = 200; rt.Frame.Z = 300;
            d["afterXYZ"] = Pose(rt.Frame);
            double[] e1 = { rt.Frame.Matrix.EulerZYX.x * Rad2Deg, rt.Frame.Matrix.EulerZYX.y * Rad2Deg, rt.Frame.Matrix.EulerZYX.z * Rad2Deg };
            d["eulerZYXdeg"] = RoundAll(e1);
            d["translation"] = RoundAll(new double[] { rt.Frame.Matrix.Translation.x, rt.Frame.Matrix.Translation.y, rt.Frame.Matrix.Translation.z });
            Quaternion q0 = rt.Frame.Matrix.Quaternion;
            d["quatIdentity"] = RoundAll(new double[] { q0[0], q0[1], q0[2], q0[3] });

            rt.Frame.RX = 90;
            d["afterRX90"] = Pose(rt.Frame);
            rt.Frame.RX = 0; rt.Frame.RZ = 30;
            d["afterRZ30"] = Pose(rt.Frame);
            Quaternion q1 = rt.Frame.Matrix.Quaternion;
            d["quatRZ30"] = RoundAll(new double[] { q1[0], q1[1], q1[2], q1[3] });
            double[] e2 = { rt.Frame.Matrix.EulerZYX.x * Rad2Deg, rt.Frame.Matrix.EulerZYX.y * Rad2Deg, rt.Frame.Matrix.EulerZYX.z * Rad2Deg };
            d["eulerRZ30"] = RoundAll(e2);

            Station s = TryStn();
            if (s != null)
            {
                d["stationName"] = s.Name;
                d["stationDisplay"] = s.DisplayName;
                d["activeTask"] = NameOf(s.ActiveTask);
                d["taskRawName"] = s.ActiveTask == null ? null : s.ActiveTask.Name;
            }
            return d;
        }

        static object DebugIk(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            RsTask task = s.ActiveTask;
            RsWorkObject wobj = ResolveWobj(task, null);
            RsToolData tool = ResolveTool(task, null);

            double[] seed = m.GetJointValues();
            ForwardKinematicsResult fr;
            Matrix4 fkM = m.CalculateForwardKinematics(seed, out fr);

            var d = new Dictionary<string, object>();
            d["seedDeg"] = RoundAll(ToDeg(seed));
            var mmPose = Bridge.Map(a, "pose");
            d["fkTranslation"] = RoundAll(new double[] { fkM.Translation.x, fkM.Translation.y, fkM.Translation.z });

            RsRobTarget rt = new RsRobTarget();
            if (mmPose != null) ApplyPose(rt.Frame, mmPose);
            else rt.Frame.Matrix = fkM;

            double[] j1 = null, j2 = null, j3 = null, j4 = null;
            string e1 = null, e2 = null, e3 = null, e4 = null;
            bool ok1 = false, ok2 = false, ok3 = false, ok4 = false;
            int[] active = m.GetActiveJoints();
            d["activeJoints"] = active;
            try { ok1 = m.CalculateInverseKinematics(rt, wobj, tool, active, out j1); } catch (Exception ex) { e1 = ex.Message; }
            try { ok2 = m.CalculateInverseKinematics(rt, wobj, tool, null, out j2); } catch (Exception ex) { e2 = ex.Message; }
            Matrix4 mm = fkM;
            mm.Translate(0, 0, 0);
            double[] j3a;
            try { ok3 = m.CalculateInverseKinematics(fkM, tool.Frame.Matrix, false, out j3); } catch (Exception ex) { e3 = ex.Message; }
            try { ok4 = m.CalculateInverseKinematics(fkM, seed, null, tool.Frame.Matrix, false, out j4); } catch (Exception ex) { e4 = ex.Message; }

            d["ikA_robTarget"] = ok1 ? (object)RoundAll(ToDeg(j1)) : (e1 ?? "false");
            d["ikB_nullAxes"] = ok2 ? (object)RoundAll(ToDeg(j2)) : (e2 ?? "false");
            d["ikC_matrix"] = ok3 ? (object)RoundAll(ToDeg(j3)) : (e3 ?? "false");
            d["ikD_matrixSeed"] = ok4 ? (object)RoundAll(ToDeg(j4)) : (e4 ?? "false");
            d["rtFramePose"] = Pose(rt.Frame);

            double[] pv = mmPose != null ? PoseArray(mmPose) : null;
            if (pv != null)
            {
                string note;
                double[] nj;
                bool nok = NumericIk.Solve(m, new double[] { pv[0], pv[1], pv[2] },
                                           new double[] { pv[3], pv[4], pv[5] },
                                           seed, out nj, out note);
                d["numeric"] = nok ? (object)RoundAll(ToDeg(nj)) : (object)note;
                if (nok)
                {
                    ForwardKinematicsResult fr2;
                    Matrix4 chk = m.CalculateForwardKinematics(nj, out fr2);
                    d["numericCheck"] = Pose(chk);
                }
            }
            return d;
        }

        static object DebugRender(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));
            var res = new List<object>();
            foreach (RsPathProcedure p in t.PathProcedures.ToArray())
            {
                foreach (RsInstruction ins in p.Instructions.ToArray())
                {
                    var row = new Dictionary<string, object>();
                    row["display"] = ins.DisplayName;
                    var args = new List<object>();
                    foreach (RsInstructionArgument arg in ins.InstructionArguments)
                        args.Add(arg == null ? null : arg.Value);
                    row["args"] = args;
                    row["rendered"] = RenderInstruction(ins);
                    res.Add(row);
                }
            }
            return res;
        }

        static object Reflect(Dictionary<string, object> a)
        {
            string typeName = Bridge.Str(a, "type");
            string filter = Bridge.Str(a, "filter", null);
            var asms = new[] {
                typeof(Station).Assembly,
                typeof(Project).Assembly,
                typeof(Matrix4).Assembly
            };

            Type t = null;
            foreach (var asm in asms)
            {
                t = asm.GetType(typeName, false);
                if (t != null) break;
                t = asm.GetTypes().FirstOrDefault(x => x.Name == typeName);
                if (t != null) break;
            }
            if (t == null) throw new InvalidOperationException("Type not found: " + typeName);

            var members = new List<object>();
            foreach (var m in t.GetMembers(System.Reflection.BindingFlags.Public |
                                          System.Reflection.BindingFlags.Instance |
                                          System.Reflection.BindingFlags.Static))
            {
                string sig = m.ToString();
                if (filter != null && sig.IndexOf(filter, StringComparison.OrdinalIgnoreCase) < 0) continue;
                members.Add(sig);
            }
            var d = new Dictionary<string, object>();
            d["type"] = t.FullName;
            d["members"] = members;
            return d;
        }

        // ================= station =================

        static Dictionary<string, object> StationSummary(Station s)
        {
            var d = new Dictionary<string, object>();
            d["name"] = s.Name;
            d["displayName"] = s.DisplayName;
            d["project"] = Project.ActiveProject == null ? null : Project.ActiveProject.Name;
            d["file"] = s.FileInfo == null ? null : s.FileInfo.FullName;
            d["dirty"] = s.IsDirty;
            d["activeTask"] = NameOf(s.ActiveTask);
            d["robots"] = Mechs(s).Select(m => m.Name).ToArray();
            d["controllers"] = s.Irc5Controllers.ToArray().Select(c => c.Name).ToArray();
            d["simState"] = Simulator.State.ToString();
            return d;
        }

        static object StationNew(Dictionary<string, object> a)
        {
            string name = Bridge.Str(a, "name", "Station");
            string robot = Bridge.Str(a, "robot", null);

            Station s = new Station();
            if (!string.IsNullOrEmpty(name)) s.Name = name;

            if (!string.IsNullOrEmpty(robot))
            {
                AddMechanism(s, robot, null);
            }
            return StationSummary(s);
        }

        static object StationOpen(Dictionary<string, object> a)
        {
            string path = Bridge.Str(a, "path");
            if (string.IsNullOrEmpty(path)) throw new InvalidOperationException("path required");
            if (!File.Exists(path)) throw new FileNotFoundException("Station file not found: " + path);

            Station s = Station.Load(path, false);
            return StationSummary(s);
        }

        static object StationSave(Dictionary<string, object> a)
        {
            Station s = Stn;
            string path = Bridge.Str(a, "path", null);
            if (string.IsNullOrEmpty(path)) s.Save();
            else s.SaveAs(path);
            return StationSummary(s);
        }

        static object StationClose(Dictionary<string, object> a)
        {
            Station s = TryStn();
            if (s != null) s.Close();
            return new Dictionary<string, object> { { "closed", true } };
        }

        static object StationInfo()
        {
            Station s = TryStn();
            if (s == null)
                return new Dictionary<string, object> { { "station", null } };
            return StationSummary(s);
        }

        // ================= robots =================

        static object RobotList()
        {
            Station s = Stn;
            var res = new List<object>();
            foreach (Mechanism m in Mechs(s))
            {
                var d = new Dictionary<string, object>();
                d["name"] = m.Name;
                d["model"] = m.ModelName;
                d["type"] = m.MechanismType.ToString();
                d["joints"] = m.GetJointValues().Length;
                d["activeJoints"] = m.GetActiveJoints().Length;
                d["task"] = NameOf(m.Task);
                d["controller"] = ControllerOf(s, m);
                try { d["jointDeg"] = RoundAll(ToDeg(m.GetJointValues())); } catch { }
                res.Add(d);
            }
            return res;
        }

        static string ControllerOf(Station s, Mechanism m)
        {
            if (m.Task == null) return null;
            foreach (RsIrc5Controller c in s.Irc5Controllers.ToArray())
                foreach (RsTask t in c.Tasks.ToArray())
                    if (t == m.Task) return c.Name;
            return null;
        }

        static string RobotLibraryPath(string lib)
        {
            if (string.IsNullOrEmpty(lib)) throw new InvalidOperationException("robot library required");
            if (File.Exists(lib)) return lib;
            string root = Path.Combine(RobotStudioAPI.DataDirectory, "ABB Library", "Robots");
            string p = Path.Combine(root, lib);
            if (File.Exists(p)) return p;
            if (!lib.EndsWith(".rslib", StringComparison.OrdinalIgnoreCase)) p = Path.Combine(root, lib + ".rslib");
            if (File.Exists(p)) return p;
            throw new FileNotFoundException("Robot library not found: " + lib);
        }

        static Mechanism AddMechanism(Station s, string lib, string name)
        {
            string path = RobotLibraryPath(lib);
            GraphicComponentLibrary glib = GraphicComponentLibrary.Load(path, true);
            GraphicComponent root = glib.RootComponent.CopyInstance();
            Mechanism mech = root as Mechanism;
            if (mech == null)
                throw new InvalidOperationException(Path.GetFileName(path) + " is not a mechanism (robot).");

            mech.Name = string.IsNullOrEmpty(name) ? Path.GetFileNameWithoutExtension(path) : name;
            s.GraphicComponents.Add(mech);
            if (s.ActiveTask == null && mech.Task != null) s.ActiveTask = mech.Task;
            return mech;
        }

        static object RobotAdd(Dictionary<string, object> a)
        {
            Station s = Stn;
            string lib = Bridge.Str(a, "lib", Bridge.Str(a, "robot"));
            Mechanism m = AddMechanism(s, lib, Bridge.Str(a, "name", null));
            var d = new Dictionary<string, object>();
            d["name"] = m.Name;
            d["model"] = m.ModelName;
            d["joints"] = m.GetJointValues().Length;
            d["task"] = NameOf(m.Task);
            d["controller"] = ControllerOf(s, m);
            d["station"] = s.Name;
            return d;
        }

        static double[] SetUnits(Dictionary<string, object> a, double[] vals)
        {
            string u = Bridge.Str(a, "units", "deg").ToLowerInvariant();
            return u.StartsWith("rad") ? vals : ToRad(vals);
        }

        static object RobotJointsGet(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            double[] rad = m.GetJointValues();
            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["radians"] = RoundAll(rad);
            d["degrees"] = RoundAll(ToDeg(rad));
            d["jointTypes"] = m.GetJointTypes().Select(x => x.ToString()).ToArray();
            return d;
        }

        static object RobotJointsSet(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            double[] raw = Bridge.Arr(a, "joints");
            if (raw == null) throw new InvalidOperationException("joints array required");

            double[] rad = SetUnits(a, raw);
            bool ok = m.SetJointValues(rad, false);
            double[] now = m.GetJointValues();

            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["applied"] = ok;
            d["degrees"] = RoundAll(ToDeg(now));
            d["radians"] = RoundAll(now);
            return d;
        }

        static object RobotHome(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            double[] home = m.GetHomePosition();
            m.SetJointValues(home, false);
            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["degrees"] = RoundAll(ToDeg(m.GetJointValues()));
            return d;
        }

        static object RobotFk(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            double[] raw = Bridge.Arr(a, "joints", m.GetJointValues());
            double[] rad = Bridge.Has(a, "joints") ? SetUnits(a, raw) : raw;

            ForwardKinematicsResult res;
            Matrix4 mat = m.CalculateForwardKinematics(rad, out res);

            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["jointsDeg"] = RoundAll(ToDeg(rad));
            d["pose"] = Pose(mat);
            d["reachable"] = res == null ? "unknown" : res.ToString();
            return d;
        }

        static RsWorkObject ActiveWobj(Station s, RsTask task)
        {
            if (task != null && task.ActiveWorkObject != null) return task.ActiveWorkObject;
            return new RsWorkObject();
        }

        static RsToolData ActiveTool(Station s, RsTask task)
        {
            if (task != null && task.ActiveTool != null) return task.ActiveTool;
            return new RsToolData();
        }

        static double[] PoseArray(Dictionary<string, object> pose)
        {
            var v = new double[6];
            string[] k = { "x", "y", "z", "rx", "ry", "rz" };
            for (int i = 0; i < 6; i++)
            {
                if (!pose.ContainsKey(k[i]) || pose[k[i]] == null) return null;
                v[i] = Convert.ToDouble(pose[k[i]], CultureInfo.InvariantCulture);
            }
            return v;
        }

        static object RobotIk(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            var pose = Bridge.Map(a, "pose");
            if (pose == null) throw new InvalidOperationException("pose {x,y,z,rx,ry,rz} required");
            double[] pv = PoseArray(pose);
            if (pv == null) throw new InvalidOperationException("pose needs x,y,z,rx,ry,rz numbers");

            double[] seed = Bridge.Arr(a, "seed");
            if (seed != null) seed = SetUnits(a, seed);
            else seed = m.GetJointValues();

            // Fast path: RobotStudio's own kinematics (needs a controller in most builds).
            RsRobTarget rt = new RsRobTarget();
            ApplyPose(rt.Frame, pose);

            RsTask task = s.ActiveTask;
            RsWorkObject wobj = task != null ? ResolveWobj(task, Bridge.Str(a, "wobj", null)) : new RsWorkObject();
            RsToolData tool = task != null ? ResolveTool(task, Bridge.Str(a, "tool", null)) : new RsToolData();

            double[] joints = null;
            bool ok = false;
            string err1 = null;
            try { ok = m.CalculateInverseKinematics(rt, wobj, tool, new int[0], out joints); }
            catch (Exception ex) { err1 = ex.Message; }

            if (!ok)
            {
                try { ok = m.CalculateInverseKinematics(rt.Frame.Matrix, tool.Frame.Matrix, false, out joints); }
                catch (Exception ex) { err1 = (err1 ?? "") + " | " + ex.Message; }
            }
            string solver = ok ? "robstudio" : null;

            string ikNote = null;
            if (!ok)
            {
                double[] seeds0 = m.GetJointValues();
                double[] home;
                try { home = m.GetHomePosition(); } catch { home = seeds0; }
                var c = new double[seeds0.Length];
                double[] alt = new double[seeds0.Length];
                for (int i = 0; i < c.Length; i++) { c[i] = Math.PI / 2; alt[i] = -Math.PI / 2; }

                double[][] trials = new double[][] { seed, seeds0, home, c, alt };
                foreach (double[] sd in trials)
                {
                    string note;
                    double[] nj;
                    bool nok = NumericIk.Solve(m, new double[] { pv[0], pv[1], pv[2] },
                                               new double[] { pv[3], pv[4], pv[5] },
                                               sd, out nj, out note);
                    if (nok) { joints = nj; ok = true; solver = "numeric"; ikNote = note; break; }
                    ikNote = note;
                }
                if (!ok) err1 = (err1 ?? "") + " | numeric: " + ikNote;
            }
            if (!ok && err1 != null) d0(err1);

            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["ok"] = ok;
            d["solver"] = solver;
            if (ikNote != null) d["note"] = ikNote;
            if (!ok) d["hint"] = "pose not solvable within joint limits; try a different orientation or a closer point";
            d["workObject"] = wobj.Name;
            d["tool"] = tool.Name;
            if (ok && joints != null)
            {
                d["degrees"] = RoundAll(ToDeg(joints));
                d["radians"] = RoundAll(joints);
            }
            return d;
        }

        static void d0(string s) { Bridge.Log("ik: " + s); }

        static object RobotReach(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            var pose = Bridge.Map(a, "pose");
            if (pose == null) throw new InvalidOperationException("pose {x,y,z,rx,ry,rz} required");

            RsRobTarget rt = new RsRobTarget();
            ApplyPose(rt.Frame, pose);

            RsTask task = s.ActiveTask;
            bool ok = m.CanReach(rt, ActiveWobj(s, task), ActiveTool(s, task));

            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["reachable"] = ok;
            d["pose"] = pose;
            return d;
        }

        static object RobotLimits(Dictionary<string, object> a)
        {
            Station s = Stn;
            Mechanism m = GetMech(s, Bridge.Str(a, "robot", null));
            double[] lo, hi;
            m.GetJointLimits(out lo, out hi);

            var d = new Dictionary<string, object>();
            d["robot"] = m.Name;
            d["minDeg"] = RoundAll(ToDeg(lo));
            d["maxDeg"] = RoundAll(ToDeg(hi));
            d["types"] = m.GetJointTypes().Select(x => x.ToString()).ToArray();
            return d;
        }

        // ================= library =================

        static object LibRobots(Dictionary<string, object> a)
        {
            string root = Path.Combine(RobotStudioAPI.DataDirectory, "ABB Library", "Robots");
            string pattern = Bridge.Str(a, "pattern", null);
            int limit = (int)Bridge.Dbl(a, "limit", 200);

            if (!Directory.Exists(root)) throw new DirectoryNotFoundException("Robot library folder not found: " + root);

            var files = Directory.GetFiles(root, "*.rslib");
            var res = new List<object>();
            foreach (string f in files.OrderBy(x => x))
            {
                string n = Path.GetFileNameWithoutExtension(f);
                if (pattern != null && n.IndexOf(pattern, StringComparison.OrdinalIgnoreCase) < 0) continue;
                res.Add(n);
                if (res.Count >= limit) break;
            }

            var d = new Dictionary<string, object>();
            d["root"] = root;
            d["total"] = files.Length;
            d["matched"] = res.Count;
            d["robots"] = res;
            return d;
        }

        // ================= RAPID model =================

        static object RapidTasks()
        {
            Station s = Stn;
            var res = new List<object>();
            foreach (RsTask t in VariantTasks(s))
            {
                var d = new Dictionary<string, object>();
                d["name"] = NameOf(t);
                d["motion"] = t.IsMotionTask;
                d["active"] = (s.ActiveTask == t);
                d["mechanism"] = t.Mechanism == null ? null : t.Mechanism.Name;
                d["tool"] = t.ActiveTool == null ? null : t.ActiveTool.Name;
                d["workObject"] = t.ActiveWorkObject == null ? null : t.ActiveWorkObject.Name;
                d["displayName"] = NameOf(t);
                d["targets"] = t.Targets.ToArray().Length;
                d["routines"] = t.PathProcedures.ToArray().Select(p => p.Name).ToArray();
                d["declarations"] = t.DataDeclarations.ToArray()
                    .Select(x => x.Name + " : " + x.GetType().Name).ToArray();
                res.Add(d);
            }
            return res;
        }

        static IEnumerable<RsTask> VariantTasks(Station s)
        {
            var seen = new HashSet<string>();
            foreach (RsIrc5Controller c in s.Irc5Controllers.ToArray())
            {
                foreach (RsTask t in c.Tasks.ToArray())
                    if (seen.Add(t.Name)) yield return t;
            }
            if (s.ActiveTask != null && seen.Add(s.ActiveTask.Name)) yield return s.ActiveTask;
        }

        static object RapidTargets(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));
            var res = new List<object>();
            foreach (RsTarget tg in t.Targets.ToArray())
            {
                var d = new Dictionary<string, object>();
                d["name"] = tg.Name;
                d["pose"] = Pose(tg.RobTarget.Frame);
                d["workObject"] = tg.WorkObject == null ? null : tg.WorkObject.Name;
                d["config"] = tg.RobTarget.ConfigurationData.GetConfigurationData();
                res.Add(d);
            }
            return res;
        }

        static object RapidAddTarget(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));
            string name = Bridge.Str(a, "name");
            var pose = Bridge.Map(a, "pose");
            if (pose == null) throw new InvalidOperationException("pose {x,y,z,rx,ry,rz} required");

            RsWorkObject wobj = ResolveWobj(t, Bridge.Str(a, "wobj", null));

            RsRobTarget rt = new RsRobTarget();
            string safe = UniqueName(t, string.IsNullOrEmpty(name) ? "p" : name);
            rt.Name = safe;
            ApplyPose(rt.Frame, pose);
            t.DataDeclarations.Add(rt);

            RsTarget tg = new RsTarget(wobj, rt);
            tg.Name = safe;
            t.Targets.Add(tg);

            var d = new Dictionary<string, object>();
            d["name"] = safe;
            d["workObject"] = wobj.Name;
            d["pose"] = Pose(rt.Frame);
            return d;
        }

        static string UniqueName(RsTask t, string wanted)
        {
            var used = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (RsDataDeclaration d in t.DataDeclarations.ToArray()) used.Add(d.Name);
            foreach (RsPathProcedure p in t.PathProcedures.ToArray()) used.Add(p.Name);
            foreach (RsTarget g in t.Targets.ToArray()) used.Add(g.Name);
            if (!used.Contains(wanted)) return wanted;
            for (int i = 1; i < 10000; i++)
            {
                string cand = wanted + "_" + i.ToString(culture);
                if (!used.Contains(cand)) return cand;
            }
            return wanted;
        }

        static readonly CultureInfo culture = CultureInfo.InvariantCulture;

        static RsWorkObject ResolveWobj(RsTask t, string name)
        {
            if (!string.IsNullOrEmpty(name))
            {
                foreach (RsDataDeclaration d in t.DataDeclarations.ToArray())
                {
                    RsWorkObject w = d as RsWorkObject;
                    if (w != null && string.Equals(w.Name, name, StringComparison.OrdinalIgnoreCase)) return w;
                }
                throw new InvalidOperationException("WorkObject not found: " + name);
            }
            if (t.ActiveWorkObject != null) return t.ActiveWorkObject;
            RsWorkObject nw = new RsWorkObject();
            nw.Name = t.GetValidRapidName("wobj0", "", 1);
            t.DataDeclarations.Add(nw);
            return nw;
        }

        static RsToolData ResolveTool(RsTask t, string name)
        {
            if (!string.IsNullOrEmpty(name))
            {
                foreach (RsDataDeclaration d in t.DataDeclarations.ToArray())
                {
                    RsToolData w = d as RsToolData;
                    if (w != null && string.Equals(w.Name, name, StringComparison.OrdinalIgnoreCase)) return w;
                }
                throw new InvalidOperationException("ToolData not found: " + name);
            }
            if (t.ActiveTool != null) return t.ActiveTool;
            RsToolData nt = new RsToolData();
            nt.Name = t.GetValidRapidName("tool0", "", 1);
            t.DataDeclarations.Add(nt);
            return nt;
        }

        static object RapidAddRoutine(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));
            string name = Bridge.Str(a, "name", "Path");
            string safe = UniqueName(t, name);

            RsPathProcedure p = new RsPathProcedure(safe);
            t.PathProcedures.Add(p);

            var d = new Dictionary<string, object>();
            d["name"] = p.Name;
            d["task"] = t.Name;
            return d;
        }

        static RsPathProcedure GetRoutine(RsTask t, string name)
        {
            RsPathProcedure[] all = t.PathProcedures.ToArray();
            if (all.Length == 0) throw new InvalidOperationException("Task has no routine.");
            if (string.IsNullOrEmpty(name))
            {
                if (all.Length == 1) return all[0];
                return all[all.Length - 1];
            }
            foreach (RsPathProcedure p in all)
                if (string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase)) return p;
            foreach (RsPathProcedure p in all)
                if (string.Equals(p.DisplayName, name, StringComparison.OrdinalIgnoreCase)) return p;
            throw new InvalidOperationException("Routine not found: " + name);
        }

        static object RapidAddMove(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));
            RsPathProcedure routine = GetRoutine(t, Bridge.Str(a, "routine", null));

            string motion = Bridge.Str(a, "motion", "L").ToUpperInvariant();
            string targetName = Bridge.Str(a, "target");
            if (string.IsNullOrEmpty(targetName))
                throw new InvalidOperationException("target (robtarget name) required");

            RsRobTarget rt = null;
            foreach (RsDataDeclaration d in t.DataDeclarations.ToArray())
            {
                RsRobTarget c = d as RsRobTarget;
                if (c != null && string.Equals(c.Name, targetName, StringComparison.OrdinalIgnoreCase)) { rt = c; break; }
            }
            if (rt == null) throw new InvalidOperationException("robtarget not found: " + targetName);

            RsToolData tool = ResolveTool(t, Bridge.Str(a, "tool", null));
            RsWorkObject wobj = ResolveWobj(t, Bridge.Str(a, "wobj", null));

            RsMoveInstruction ins;
            if (motion.StartsWith("ABS"))
            {
                RsJointTarget jt = null;
                foreach (RsDataDeclaration d in t.DataDeclarations.ToArray())
                {
                    RsJointTarget c = d as RsJointTarget;
                    if (c != null && string.Equals(c.Name, targetName, StringComparison.OrdinalIgnoreCase)) { jt = c; break; }
                }
                if (jt == null) throw new InvalidOperationException("jointtarget not found: " + targetName);
                ins = new RsMoveInstruction(t, "MoveAbs", "Default", jt.Name);
            }
            else if (motion.StartsWith("C"))
            {
                string via = Bridge.Str(a, "via");
                ins = new RsMoveInstruction(t, "Move", "Default", MotionType.Circular,
                    rt.Name, via ?? rt.Name, tool.Name);
            }
            else if (motion.StartsWith("J"))
            {
                ins = new RsMoveInstruction(t, "Move", "Default", MotionType.Joint,
                    rt.Name, rt.Name, tool.Name);
            }
            else
            {
                ins = new RsMoveInstruction(t, "Move", "Default", MotionType.Linear,
                    rt.Name, rt.Name, tool.Name);
            }

            TrySetArg(ins, "Speed", Bridge.Str(a, "speed", null));
            TrySetArg(ins, "Zone", Bridge.Str(a, "zone", null));

            routine.Instructions.Add(ins);

            var res = new Dictionary<string, object>();
            res["routine"] = routine.Name;
            res["instruction"] = RenderInstruction(ins);
            res["instructionCount"] = routine.Instructions.ToArray().Length;
            res["workObject"] = wobj.Name;
            res["tool"] = tool.Name;
            return res;
        }

        static void TrySetArg(RsInstruction ins, string argName, string value)
        {
            if (string.IsNullOrEmpty(value)) return;
            try
            {
                if (ins.InstructionArguments.Contains(argName))
                    ins.InstructionArguments[argName].Value = value;
            }
            catch { }
        }

        static string ArgValue(RsInstruction ins, string name)
        {
            try
            {
                if (ins.InstructionArguments.Contains(name))
                {
                    RsInstructionArgument a = ins.InstructionArguments[name];
                    return a == null ? null : a.Value;
                }
            }
            catch { }
            return null;
        }

        static string Join(params string[] parts)
        {
            var l = new List<string>();
            foreach (string p in parts) if (!string.IsNullOrEmpty(p)) l.Add(p);
            return string.Join(", ", l.ToArray());
        }

        static string RenderMove(RsMoveInstruction mi)
        {
            string proc = mi.DisplayName;
            string bare = proc;
            int sp = proc.IndexOf(' ');
            if (sp > 0) bare = proc.Substring(0, sp);

            string speed = ArgValue(mi, "Speed");
            string zone = ArgValue(mi, "Zone");
            string tool = ArgValue(mi, "Tool");
            string wobj = ArgValue(mi, "Wobj");

            string toPoint = null, cirPoint = null, jointPos = null;
            try { RsInstructionArgument a = mi.GetToPointArgument(); if (a != null) toPoint = a.Value; } catch { }
            try { RsInstructionArgument a = mi.GetCirPointArgument(); if (a != null) cirPoint = a.Value; } catch { }
            try { RsInstructionArgument a = mi.GetToJointPosArgument(); if (a != null) jointPos = a.Value; } catch { }

            if (!string.IsNullOrEmpty(jointPos))
                return bare + " " + Join(jointPos, speed, zone, tool) + ";";

            if (!string.IsNullOrEmpty(cirPoint) && !string.IsNullOrEmpty(toPoint))
                return bare + " " + Join(cirPoint, toPoint, speed, zone, tool) + ";";

            if (!string.IsNullOrEmpty(toPoint))
                return bare + " " + Join(toPoint, speed, zone, tool) + ";";

            return RenderInstruction(mi);
        }

        static string RenderInstruction(RsInstruction ins)
        {
            RsMoveInstruction mi = ins as RsMoveInstruction;
            if (mi != null)
            {
                try { return RenderMove(mi); } catch { }
            }

            string display = null;
            try { display = ins.DisplayName; } catch { }

            var values = new List<string>();
            foreach (RsInstructionArgument arg in ins.InstructionArguments)
            {
                if (arg == null) continue;
                string v = arg.Value;
                if (!string.IsNullOrEmpty(v)) values.Add(v);
            }

            string head = display;
            if (string.IsNullOrEmpty(head)) head = ins.GetType().Name;

            // DisplayName sometimes already contains the first argument(s).
            var rest = new List<string>();
            foreach (string v in values)
            {
                if (head.EndsWith(" " + v, StringComparison.OrdinalIgnoreCase)) continue;
                rest.Add(v);
            }
            string body = string.Join(", ", rest.ToArray());
            if (body.Length == 0) return head.Trim() + ";";
            return (head.Trim() + " " + body).Trim() + ";";
        }

        static object RapidCode(Dictionary<string, object> a)
        {
            Station s = Stn;
            RsTask t = GetTask(s, Bridge.Str(a, "task", null));

            var sb = new System.Text.StringBuilder();
            sb.AppendLine("MODULE " + (NameOf(t) ?? t.Name) + "_MOD");
            sb.AppendLine("  ! generated from RobotStudio station: " + s.Name);

            foreach (RsDataDeclaration d in t.DataDeclarations.ToArray())
            {
                RsToolData td = d as RsToolData;
                if (td != null) sb.AppendLine("  PERS tooldata " + td.Name + ";");
                RsWorkObject wo = d as RsWorkObject;
                if (wo != null) sb.AppendLine("  PERS wobjdata " + wo.Name + ";");
                RsRobTarget rt = d as RsRobTarget;
                if (rt != null) sb.AppendLine("  CONST robtarget " + rt.Name + " := " + RobTargetLiteral(rt) + ";");
                RsJointTarget jt = d as RsJointTarget;
                if (jt != null)
                {
                    double[] j;
                    try { j = ToDeg(jt.GetJointValues(t.Mechanism, false)); }
                    catch { j = new double[0]; }
                    sb.AppendLine("  CONST jointtarget " + jt.Name + " := [[" +
                        string.Join(",", j.Select(x => Math.Round(x, 3).ToString(CultureInfo.InvariantCulture)).ToArray()) +
                        "],[9E9,9E9,9E9,9E9,9E9,9E9]];");
                }
            }

            foreach (RsPathProcedure p in t.PathProcedures.ToArray())
            {
                sb.AppendLine();
                sb.AppendLine("  PROC " + p.Name + "()");
                foreach (RsInstruction ins in p.Instructions.ToArray())
                    sb.AppendLine("    " + RenderInstruction(ins));
                sb.AppendLine("  ENDPROC");
            }

            sb.AppendLine("ENDMOD");

            var res = new Dictionary<string, object>();
            res["task"] = NameOf(t);
            res["code"] = sb.ToString();
            return res;
        }

        static string RobTargetLiteral(RsRobTarget rt)
        {
            Transform f = rt.Frame;
            var sb = new System.Text.StringBuilder();
            sb.Append("[[" + Math.Round(f.X, 3).ToString(CultureInfo.InvariantCulture));
            sb.Append("," + Math.Round(f.Y, 3).ToString(CultureInfo.InvariantCulture));
            sb.Append("," + Math.Round(f.Z, 3).ToString(CultureInfo.InvariantCulture) + "],[");

            double[] q = QuaternionOf(f);
            sb.Append(string.Join(",", q.Select(x => Math.Round(x, 5).ToString(CultureInfo.InvariantCulture)).ToArray()));
            sb.Append("],[");

            int[] cfg = null;
            try { cfg = rt.ConfigurationData.GetConfigurationData(); } catch { }
            if (cfg != null && cfg.Length >= 3)
                sb.Append(cfg[0] + "," + cfg[1] + "," + cfg[2] + "," + (cfg.Length > 3 ? cfg[3] : 0));
            else
                sb.Append("0,0,0,0");
            sb.Append("],[9E9,9E9,9E9,9E9,9E9,9E9]]");
            return sb.ToString();
        }

        static double[] QuaternionOf(Transform f)
        {
            try
            {
                Quaternion q = f.Matrix.Quaternion;   // (w,x,y,z)
                return new double[] { q[1], q[2], q[3], q[0] };
            }
            catch
            {
                return new double[] { 1, 0, 0, 0 };
            }
        }

        // ================= simulation =================

        static object SimState()
        {
            var d = new Dictionary<string, object>();
            d["state"] = Simulator.State.ToString();
            d["speed"] = Simulator.SimulationSpeed;
            d["time"] = Round(Simulator.CurrentTime);
            return d;
        }

        // ================= IO =================

        static object IoList(Dictionary<string, object> a)
        {
            Station s = Stn;
            var res = new List<object>();
            foreach (IOSignal sig in s.IOSignals.ToArray())
            {
                var d = new Dictionary<string, object>();
                d["name"] = sig.Name;
                d["type"] = sig.SignalType.ToString();
                d["input"] = sig.IsInput;
                d["value"] = sig.Value;
                res.Add(d);
            }
            return res;
        }

        static IOSignal FindSignal(string name)
        {
            Station s = Stn;
            foreach (IOSignal sig in s.IOSignals.ToArray())
                if (string.Equals(sig.Name, name, StringComparison.OrdinalIgnoreCase)) return sig;
            throw new InvalidOperationException("IOSignal not found: " + name);
        }

        static object IoGet(Dictionary<string, object> a)
        {
            string name = Bridge.Str(a, "name");
            IOSignal sig = FindSignal(name);
            var d = new Dictionary<string, object>();
            d["name"] = sig.Name;
            d["value"] = sig.Value;
            d["type"] = sig.SignalType.ToString();
            d["input"] = sig.IsInput;
            return d;
        }

        static object IoSet(Dictionary<string, object> a)
        {
            string name = Bridge.Str(a, "name");
            IOSignal sig = FindSignal(name);
            string raw = Bridge.Str(a, "value");
            if (sig.SignalType.ToString().ToLowerInvariant().Contains("analog"))
                sig.Value = Convert.ToDouble(raw, CultureInfo.InvariantCulture);
            else
                sig.Value = Convert.ToBoolean(raw);

            var d = new Dictionary<string, object>();
            d["name"] = sig.Name;
            d["value"] = sig.Value;
            return d;
        }
    }
}
