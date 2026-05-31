using System;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using Vizcon.OSC;

class Program
{
    [DllImport("winmm.dll")] static extern int timeBeginPeriod(int uPeriod);
    [DllImport("winmm.dll")] static extern int timeEndPeriod(int uPeriod);

    static void Main()
    {
        double tagSize = 0.05; // Größe des Tags in Metern (50mm)
        var oscClient = new UDPSender("192.168.179.17", 39539);

        using var capture = new VideoCapture(0);
        capture.Set(VideoCaptureProperties.FrameWidth, 640);
        capture.Set(VideoCaptureProperties.FrameHeight, 480);
        capture.Set(VideoCaptureProperties.Fps, 30);
        capture.Set(VideoCaptureProperties.AutoExposure, 0.75);

        using var dictionary = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.DictAprilTag_36h11);
        var detectorParameters = new DetectorParameters()
        {
            AdaptiveThreshWinSizeMin = 3,
            AdaptiveThreshWinSizeMax = 23,
            AdaptiveThreshWinSizeStep = 10,
            PolygonalApproxAccuracyRate = 0.08, // Default 0.05 → toleranter bei unscharfen Kanten
        };

        using var frame = new Mat();
        using var gray = new Mat();

        const string calibFile = "calibration.json";
        double cx = 320.0;
        double cy = 240.0;

        int camW = (int)capture.Get(VideoCaptureProperties.FrameWidth);
        int camH = (int)capture.Get(VideoCaptureProperties.FrameHeight);

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

                // Zentrierungskreuz
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
                if (key == 13 && ids != null && ids.Length > 0) // 13 = ENTER
                {
                    double pWidth = Math.Sqrt(Math.Pow(corners[0][1].X - corners[0][0].X, 2) +
                                              Math.Pow(corners[0][1].Y - corners[0][0].Y, 2));

                    calculatedFocalLengths[currentStep] = (pWidth * targetDistances[currentStep]) / tagSize;
                    Console.WriteLine($"Messung bei {targetDistances[currentStep]}m erfolgreich. f = {calculatedFocalLengths[currentStep]:F2}");
                    currentStep++;
                }
            }

            double f = calculatedFocalLengths.Average();

            string saveJson = System.Text.Json.JsonSerializer.Serialize(new { fFinal = f, cx, cy });
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
            var doc = System.Text.Json.JsonDocument.Parse(json);
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
        const float smoothAlpha = 0.35f; // 0 = max. Glättung (viel Lag), 1 = kein Smoothing
        var smoothPos = new Dictionary<int, (float x, float y, float z)>();
        var smoothRot = new Dictionary<int, Quaternion>();

        timeBeginPeriod(1); // Windows-Timer auf 1ms Auflösung für präzises Sleep
        var timer = new System.Diagnostics.Stopwatch();
        var uptime = System.Diagnostics.Stopwatch.StartNew();
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
                CvAruco.EstimatePoseSingleMarkers(corners, (float)tagSize, cameraMatrix, distCoeffs, rvecs, tvecs);

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

                    // EMA-Smoothing
                    int id = ids[i];
                    if (!smoothPos.TryGetValue(id, out var sp))
                    {
                        sp = (tx, -ty, tz);
                        smoothRot[id] = new Quaternion(oqx, oqy, oqz, oqw);
                    }
                    var (sx, sy, sz) = sp;
                    sx = smoothAlpha * tx  + (1 - smoothAlpha) * sx;
                    sy = smoothAlpha * -ty + (1 - smoothAlpha) * sy;
                    sz = smoothAlpha * tz  + (1 - smoothAlpha) * sz;
                    smoothPos[id] = (sx, sy, sz);

                    var sq = Quaternion.Slerp(smoothRot[id], new Quaternion(oqx, oqy, oqz, oqw), smoothAlpha);
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

            int sleepTime = 33 - (int)timer.ElapsedMilliseconds; // ~30 fps cap
            if (sleepTime > 0) System.Threading.Thread.Sleep(sleepTime);

            // FPS aus der vollständigen Frame-Dauer (inkl. Sleep) für die nächste Anzeige
            if (timer.ElapsedMilliseconds > 0)
                fps = 1000.0 / timer.ElapsedMilliseconds;
        }

        timeEndPeriod(1);
    }
}
