using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Threading;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using Vizcon.OSC;

class AppConfig
{
    public string OscTargetIp { get; set; } = "127.0.0.1";
    public int OscPort { get; set; } = 39539;
    public double TagSizeMeters { get; set; } = 0.05;
    public float SmoothAlpha { get; set; } = 0.35f;
    public int CameraIndex { get; set; } = 0;
    public int CameraWidth { get; set; } = 640;
    public int CameraHeight { get; set; } = 480;
    public int CameraFps { get; set; } = 30;
}

class Program
{
    [DllImport("winmm.dll")] static extern int timeBeginPeriod(int uPeriod);
    [DllImport("winmm.dll")] static extern int timeEndPeriod(int uPeriod);

    static AppConfig LoadConfig(string path)
    {
        var cfg = new AppConfig();
        if (!File.Exists(path))
        {
            Console.WriteLine($"WARNUNG: '{path}' nicht gefunden — Standardwerte werden verwendet.");
            return cfg;
        }
        foreach (var raw in File.ReadLines(path))
        {
            var line = raw.Trim();
            if (line.StartsWith('#') || !line.Contains('=')) continue;
            var sep = line.IndexOf('=');
            var key = line[..sep].Trim();
            var val = line[(sep + 1)..].Trim();
            var comment = val.IndexOf('#');
            if (comment >= 0) val = val[..comment].Trim();

            switch (key)
            {
                case "OscTargetIp": cfg.OscTargetIp = val; break;
                case "OscPort": cfg.OscPort = int.Parse(val); break;
                case "TagSizeMeters": cfg.TagSizeMeters = double.Parse(val, CultureInfo.InvariantCulture); break;
                case "SmoothAlpha": cfg.SmoothAlpha = float.Parse(val, CultureInfo.InvariantCulture); break;
                case "CameraIndex": cfg.CameraIndex = int.Parse(val); break;
                case "CameraWidth": cfg.CameraWidth = int.Parse(val); break;
                case "CameraHeight": cfg.CameraHeight = int.Parse(val); break;
                case "CameraFps": cfg.CameraFps = int.Parse(val); break;
            }
        }
        return cfg;
    }

    static void Main()
    {
        var cfg = LoadConfig("config.ini");
        Console.WriteLine($"OSC → {cfg.OscTargetIp}:{cfg.OscPort}  Tag {cfg.TagSizeMeters * 100:F0}mm  Kamera {cfg.CameraWidth}x{cfg.CameraHeight}@{cfg.CameraFps}fps  α={cfg.SmoothAlpha}");

        var oscClient = new UDPSender(cfg.OscTargetIp, cfg.OscPort);

        using var capture = new VideoCapture(cfg.CameraIndex);
        capture.Set(VideoCaptureProperties.FrameWidth, cfg.CameraWidth);
        capture.Set(VideoCaptureProperties.FrameHeight, cfg.CameraHeight);
        capture.Set(VideoCaptureProperties.Fps, cfg.CameraFps);
        capture.Set(VideoCaptureProperties.AutoExposure, 0.75);

        using var dictionary = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.DictAprilTag_36h11);
        var detectorParameters = new DetectorParameters()
        {
            AdaptiveThreshWinSizeMin = 3,
            AdaptiveThreshWinSizeMax = 23,
            AdaptiveThreshWinSizeStep = 10,
            PolygonalApproxAccuracyRate = 0.08,
        };

        using var frame = new Mat();
        using var gray = new Mat();

        const string calibFile = "calibration.json";

        int camW = (int)capture.Get(VideoCaptureProperties.FrameWidth);
        int camH = (int)capture.Get(VideoCaptureProperties.FrameHeight);
        double cx = camW / 2.0;
        double cy = camH / 2.0;

        double RunCalibration()
        {
            double[] targetDistances = { 0.10, 0.50, 1.0 };
            double[] calculatedFocalLengths = new double[3];
            int currentStep = 0;

            Console.WriteLine("=== KALIBRIERUNGS-MODUS ===");

            Cv2.NamedWindow("Kalibrierung", WindowFlags.Normal);
            Cv2.ResizeWindow("Kalibrierung", camW, camH);

            while (currentStep < 3)
            {
                capture.Read(frame);
                if (frame.Empty()) continue;

                Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
                CvAruco.DetectMarkers(gray, dictionary, out Point2f[][] corners, out int[] ids, detectorParameters, out _);

                int crossX = frame.Width / 2;
                int crossY = frame.Height / 2;
                Cv2.Line(frame, new Point(crossX, 0), new Point(crossX, frame.Height), Scalar.White, 1, LineTypes.AntiAlias);
                Cv2.Line(frame, new Point(0, crossY), new Point(frame.Width, crossY), Scalar.White, 1, LineTypes.AntiAlias);
                Cv2.Circle(frame, new Point(crossX, crossY), 20, Scalar.White, 1, LineTypes.AntiAlias);

                string msg = $"Bitte Tag FRONTAL bei {targetDistances[currentStep]}m halten und ENTER druecken. ({camW}x{camH})";
                Cv2.PutText(frame, msg, new Point(20, 40), HersheyFonts.HersheySimplex, 0.5, Scalar.Red, 2);
                CvAruco.DrawDetectedMarkers(frame, corners, ids);
                Cv2.ImShow("Kalibrierung", frame);

                int key = Cv2.WaitKey(1);
                if (key == 13 && ids != null && ids.Length > 0)
                {
                    double pWidth = Math.Sqrt(Math.Pow(corners[0][1].X - corners[0][0].X, 2) +
                                              Math.Pow(corners[0][1].Y - corners[0][0].Y, 2));

                    calculatedFocalLengths[currentStep] = (pWidth * targetDistances[currentStep]) / cfg.TagSizeMeters;
                    Console.WriteLine($"Messung bei {targetDistances[currentStep]}m erfolgreich. f = {calculatedFocalLengths[currentStep]:F2}");
                    currentStep++;
                }
            }

            double f = calculatedFocalLengths.Average();

            string saveJson = JsonSerializer.Serialize(new { fFinal = f, cx, cy });
            File.WriteAllText(calibFile, saveJson);

            Console.WriteLine($"\nKalibrierung beendet! Brennweite f = {f:F2}");
            Console.WriteLine($"Kalibrierung gespeichert in '{calibFile}'.");
            Console.WriteLine("VMC-Sendemodus. Beenden: ESC  |  Neu-Kalibrierung: C  |  Koordinaten: K\n");
            Cv2.DestroyWindow("Kalibrierung");

            return f;
        }

        // --- KALIBRIERUNG LADEN ODER DURCHFÜHREN ---
        double fFinal;

        if (File.Exists(calibFile))
        {
            string json = File.ReadAllText(calibFile);
            var doc = JsonDocument.Parse(json);
            fFinal = doc.RootElement.GetProperty("fFinal").GetDouble();
            Console.WriteLine($"Kalibrierung geladen aus '{calibFile}': f = {fFinal:F2}");
            Console.WriteLine("VMC-Sendemodus. Beenden: ESC  |  Neu-Kalibrierung: C  |  Koordinaten: K\n");
        }
        else
        {
            fFinal = RunCalibration();
        }

        using var cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
        cameraMatrix.Set<double>(0, 0, fFinal); cameraMatrix.Set<double>(0, 1, 0); cameraMatrix.Set<double>(0, 2, cx);
        cameraMatrix.Set<double>(1, 0, 0); cameraMatrix.Set<double>(1, 1, fFinal); cameraMatrix.Set<double>(1, 2, cy);
        cameraMatrix.Set<double>(2, 0, 0); cameraMatrix.Set<double>(2, 1, 0); cameraMatrix.Set<double>(2, 2, 1);

        using var distCoeffs = new Mat();

        // --- VMC-SENDEMODUS ---
        var smoothPos = new Dictionary<int, (float x, float y, float z)>();
        var smoothRot = new Dictionary<int, Quaternion>();

        timeBeginPeriod(1);
        var timer = new Stopwatch();
        var uptime = Stopwatch.StartNew();
        int frameMs = 1000 / cfg.CameraFps;
        double fps = 0;
        bool showCoords = false;

        using var rvecs = new Mat();
        using var tvecs = new Mat();
        using var rMat = new Mat();

        while (true)
        {
            timer.Restart();
            capture.Read(frame);
            if (frame.Empty()) break;

            Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
            CvAruco.DetectMarkers(gray, dictionary, out Point2f[][] corners, out int[] ids, detectorParameters, out _);

            if (ids != null && ids.Length > 0)
            {
                CvAruco.EstimatePoseSingleMarkers(corners, (float)cfg.TagSizeMeters, cameraMatrix, distCoeffs, rvecs, tvecs);

                for (int i = 0; i < ids.Length; i++)
                {
                    Vec3d rvec = rvecs.At<Vec3d>(i);
                    Vec3d tvec = tvecs.At<Vec3d>(i);

                    float tx = (float)tvec.Item0;
                    float ty = (float)tvec.Item1;
                    float tz = (float)tvec.Item2;

                    Cv2.Rodrigues(rvec, rMat);

                    double r00 = rMat.Get<double>(0, 0);
                    double r11 = rMat.Get<double>(1, 1);
                    double r22 = rMat.Get<double>(2, 2);
                    double r21 = rMat.Get<double>(2, 1);
                    double r12 = rMat.Get<double>(1, 2);
                    double r02 = rMat.Get<double>(0, 2);
                    double r20 = rMat.Get<double>(2, 0);
                    double r10 = rMat.Get<double>(1, 0);
                    double r01 = rMat.Get<double>(0, 1);

                    float qx, qy, qz, qw;
                    double tr = r00 + r11 + r22;
                    if (tr > 0)
                    {
                        double s = Math.Sqrt(tr + 1.0) * 2;
                        qw = (float)(0.25 * s);
                        qx = (float)((r21 - r12) / s);
                        qy = (float)((r02 - r20) / s);
                        qz = (float)((r10 - r01) / s);
                    }
                    else if (r00 > r11 && r00 > r22)
                    {
                        double s = Math.Sqrt(1.0 + r00 - r11 - r22) * 2;
                        qw = (float)((r21 - r12) / s);
                        qx = (float)(0.25 * s);
                        qy = (float)((r01 + r10) / s);
                        qz = (float)((r02 + r20) / s);
                    }
                    else if (r11 > r22)
                    {
                        double s = Math.Sqrt(1.0 + r11 - r00 - r22) * 2;
                        qw = (float)((r02 - r20) / s);
                        qx = (float)((r01 + r10) / s);
                        qy = (float)(0.25 * s);
                        qz = (float)((r12 + r21) / s);
                    }
                    else
                    {
                        double s = Math.Sqrt(1.0 + r22 - r00 - r11) * 2;
                        qw = (float)((r10 - r01) / s);
                        qx = (float)((r02 + r20) / s);
                        qy = (float)((r12 + r21) / s);
                        qz = (float)(0.25 * s);
                    }

                    float oqx = qx, oqy = -qy, oqz = qz, oqw = -qw;
                    if (oqw < 0) { oqx = -oqx; oqy = -oqy; oqz = -oqz; oqw = -oqw; }

                    int id = ids[i];
                    if (!smoothPos.TryGetValue(id, out var sp))
                    {
                        sp = (tx, -ty, tz);
                        smoothRot[id] = new Quaternion(oqx, oqy, oqz, oqw);
                    }
                    var (sx, sy, sz) = sp;
                    sx = cfg.SmoothAlpha * tx + (1 - cfg.SmoothAlpha) * sx;
                    sy = cfg.SmoothAlpha * -ty + (1 - cfg.SmoothAlpha) * sy;
                    sz = cfg.SmoothAlpha * tz + (1 - cfg.SmoothAlpha) * sz;
                    smoothPos[id] = (sx, sy, sz);

                    var sq = Quaternion.Slerp(smoothRot[id], new Quaternion(oqx, oqy, oqz, oqw), cfg.SmoothAlpha);
                    smoothRot[id] = sq;

                    oscClient.Send(new OscMessage("/VMC/Ext/T", (float)uptime.Elapsed.TotalSeconds));
                    oscClient.Send(new OscMessage("/VMC/Ext/Tra/Pos", $"AprilTag_{ids[i]}", sx, sy, sz, sq.X, sq.Y, sq.Z, sq.W));

                    if (showCoords)
                        Console.Write($"\rTag {ids[i]:D2} | x={tx:F3}  y={ty:F3}  z={tz:F3}  rx={rvec.Item0:F3}  ry={rvec.Item1:F3}  rz={rvec.Item2:F3}  {fps:F1} fps   ");
                }
            }

            if (Console.KeyAvailable)
            {
                var key = Console.ReadKey(intercept: true).Key;
                if (key == ConsoleKey.Escape) break;
                if (key == ConsoleKey.C)
                {
                    fFinal = RunCalibration();
                    cameraMatrix.Set<double>(0, 0, fFinal);
                    cameraMatrix.Set<double>(1, 1, fFinal);
                }
                if (key == ConsoleKey.K)
                {
                    showCoords = !showCoords;
                    if (!showCoords) Console.Write("\r" + new string(' ', Console.WindowWidth - 1) + "\r");
                }
            }

            int sleepTime = frameMs - (int)timer.ElapsedMilliseconds;
            if (sleepTime > 0) Thread.Sleep(sleepTime);

            if (timer.ElapsedMilliseconds > 0)
                fps = 1000.0 / timer.ElapsedMilliseconds;
        }

        timeEndPeriod(1);
    }
}
