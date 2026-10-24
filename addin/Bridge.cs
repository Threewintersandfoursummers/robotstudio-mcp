using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Web.Script.Serialization;
using System.Windows;
using System.Windows.Threading;

namespace RsMcpBridge
{
    /// <summary>
    /// RobotStudio 6.08 general add-in entry point.
    /// Starts a loopback TCP JSON server so an external MCP process can drive the station.
    /// </summary>
    public class AddinStart
    {
        // The host looks for "static public void AddinMain()".
        public static void AddinMain()
        {
            Bridge.Start();
        }
    }

    internal static class Bridge
    {
        internal const int BasePort = 47950;
        internal const int PortCount = 12;

        internal static Dispatcher Ui;
        internal static TcpListener Listener;
        internal static int Port = 0;
        internal static readonly object Gate = new object();
        internal static string DataDir;

        internal static string LogPath
        {
            get { return Path.Combine(DataDir, "bridge.log"); }
        }

        internal static void Log(string msg)
        {
            try
            {
                File.AppendAllText(LogPath,
                    DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff") + "  " + msg + Environment.NewLine,
                    Encoding.UTF8);
            }
            catch { }
        }

        internal static void Start()
        {
            try
            {
                DataDir = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "RsMcpBridge");
                Directory.CreateDirectory(DataDir);

                Ui = Application.Current != null && Application.Current.Dispatcher != null
                    ? Application.Current.Dispatcher
                    : Dispatcher.CurrentDispatcher;

                Log("AddinMain pid=" + System.Diagnostics.Process.GetCurrentProcess().Id +
                    " thread " + Thread.CurrentThread.ManagedThreadId +
                    " dispatcher=" + (Ui != null) +
                    " exe=" + System.Diagnostics.Process.GetCurrentProcess().ProcessName);

                var t = new Thread(Listen) { IsBackground = true, Name = "RsMcpBridge" };
                t.Start();
            }
            catch (Exception ex)
            {
                Log("FATAL Start: " + ex);
            }
        }

        static void Listen()
        {
            for (int i = 0; i < PortCount; i++)
            {
                int p = BasePort + i;
                try
                {
                    var l = new TcpListener(IPAddress.Loopback, p);
                    l.Start();
                    Listener = l;
                    Port = p;
                    break;
                }
                catch { }
            }
            if (Listener == null)
            {
                Log("FATAL: no free port in range " + BasePort + ".." + (BasePort + PortCount - 1));
                return;
            }

            try
            {
                File.WriteAllText(Path.Combine(DataDir, "port.txt"), Port.ToString(), Encoding.UTF8);
            }
            catch { }
            Log("Listening on " + Listener.LocalEndpoint);
            Thread st = new Thread(SelfTest) { IsBackground = true };
            st.Start();

            while (true)
            {
                try
                {
                    TcpClient client = Listener.AcceptTcpClient();
                    var th = new Thread(() => Serve(client)) { IsBackground = true };
                    th.Start();
                }
                catch (Exception ex)
                {
                    Log("accept: " + ex.Message);
                    Thread.Sleep(200);
                }
            }
        }

        static void SelfTest()
        {
            Thread.Sleep(1500);
            try
            {
                using (var c = new TcpClient())
                {
                    c.Connect(IPAddress.Loopback, Port);
                    Log("selftest: self-connect OK");
                }
            }
            catch (Exception ex)
            {
                Log("selftest: self-connect FAILED " + ex.Message);
            }
        }

        static void Serve(TcpClient client)
        {
            try
            {
                client.NoDelay = true;
                using (var stream = client.GetStream())
                using (var reader = new StreamReader(stream, new UTF8Encoding(false)))
                using (var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true })
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (line.Trim().Length == 0) continue;
                        string resp = HandleLine(line);
                        writer.WriteLine(resp);
                    }
                }
            }
            catch (Exception ex)
            {
                Log("serve: " + ex.Message);
            }
            finally
            {
                try { client.Close(); } catch { }
            }
        }

        static readonly JavaScriptSerializer Json = new JavaScriptSerializer
        {
            MaxJsonLength = int.MaxValue,
            RecursionLimit = 200
        };

        static string HandleLine(string line)
        {
            object id = null;
            try
            {
                var req = Json.DeserializeObject(line) as Dictionary<string, object>;
                if (req == null) return Err(null, "Malformed request");

                if (req.ContainsKey("id")) id = req["id"];
                string cmd = req.ContainsKey("cmd") ? Convert.ToString(req["cmd"]) : null;
                var args = req.ContainsKey("args") && req["args"] is Dictionary<string, object>
                    ? (Dictionary<string, object>)req["args"]
                    : new Dictionary<string, object>();

                if (string.IsNullOrEmpty(cmd)) return Err(id, "Missing cmd");

                object result = InvokeOnUi(cmd, args);
                var payload = new Dictionary<string, object>();
                payload["id"] = id;
                payload["ok"] = true;
                payload["result"] = result;
                return Json.Serialize(payload);
            }
            catch (Exception ex)
            {
                return Err(id, Unwrap(ex));
            }
        }

        static string Unwrap(Exception ex)
        {
            var inner = ex;
            while (inner.InnerException != null) inner = inner.InnerException;
            if (inner is System.Reflection.TargetInvocationException && inner.InnerException != null)
                inner = inner.InnerException;
            return inner.GetType().Name + ": " + inner.Message;
        }

        static string Err(object id, string message)
        {
            var payload = new Dictionary<string, object>();
            payload["id"] = id;
            payload["ok"] = false;
            payload["error"] = message;
            return Json.Serialize(payload);
        }

        /// <summary>All RobotStudio API work is marshalled onto the UI thread.</summary>
        static object InvokeOnUi(string cmd, Dictionary<string, object> args)
        {
            object result = null;
            Exception failure = null;

            Action work = () =>
            {
                lock (Gate)
                {
                    try { result = Commands.Execute(cmd, args); }
                    catch (Exception ex) { failure = ex; }
                }
            };

            if (Ui == null || Ui.CheckAccess()) work();
            else Ui.Invoke(DispatcherPriority.Normal, work);

            if (failure != null) throw failure;

            // keep the viewport in sync with whatever we changed
            SoftRefresh();
            return result;
        }

        static void SoftRefresh()
        {
            try
            {
                var stn = ABB.Robotics.RobotStudio.Stations.Station.ActiveStation;
                if (stn != null) ABB.Robotics.RobotStudio.Stations.Station.UpdateGraphics();
            }
            catch { }
        }

        // ---------- argument helpers ----------

        internal static string Str(Dictionary<string, object> a, string key)
        {
            if (a == null || !a.ContainsKey(key)) return null;
            var v = a[key];
            if (v == null) return null;
            return Convert.ToString(v);
        }

        internal static string Str(Dictionary<string, object> a, string key, string def)
        {
            string s = Str(a, key);
            return string.IsNullOrEmpty(s) ? def : s;
        }

        internal static bool Has(Dictionary<string, object> a, string key)
        {
            return a != null && a.ContainsKey(key) && a[key] != null;
        }

        internal static double Dbl(Dictionary<string, object> a, string key, double def)
        {
            if (a == null || !a.ContainsKey(key) || a[key] == null) return def;
            try { return Convert.ToDouble(a[key], CultureInfo.InvariantCulture); }
            catch { return def; }
        }

        internal static bool Bool(Dictionary<string, object> a, string key, bool def)
        {
            if (a == null || !a.ContainsKey(key) || a[key] == null) return def;
            var v = a[key];
            if (v is bool) return (bool)v;
            return Convert.ToString(v).Equals("true", StringComparison.OrdinalIgnoreCase);
        }

        internal static Dictionary<string, object> Map(Dictionary<string, object> a, string key)
        {
            if (a == null || !a.ContainsKey(key)) return null;
            return a[key] as Dictionary<string, object>;
        }

        internal static double[] Arr(Dictionary<string, object> a, string key)
        {
            if (a == null || !a.ContainsKey(key) || a[key] == null) return null;
            var arr = a[key] as object[];
            if (arr == null) return null;
            var res = new double[arr.Length];
            for (int i = 0; i < arr.Length; i++)
                res[i] = Convert.ToDouble(arr[i], CultureInfo.InvariantCulture);
            return res;
        }

        internal static double[] Arr(Dictionary<string, object> a, string key, double[] def)
        {
            var v = Arr(a, key);
            return v ?? def;
        }

        internal static List<object> List(params object[] items)
        {
            var l = new List<object>();
            foreach (var i in items) l.Add(i);
            return l;
        }
    }
}
