using System;
using System.Linq;
using System.Numerics;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using Vizcon.OSC;

class Program
{
    static void Main()
    {
        double tagSize = 0.10; // Größe des Tags in Metern (z.B. 10cm)
        var oscClient = new UDPSender("127.0.0.1", 39539);
        
        using var capture = new VideoCapture(0);
        capture.Set(VideoCaptureProperties.FrameWidth, 640);
        capture.Set(VideoCaptureProperties.FrameHeight, 480);
        
        // Exakter Name des Enums (DictAprilTag_36h11)
        using var dictionary = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.DictAprilTag_36h11);
        var detectorParameters = new DetectorParameters(); 
        
        using var frame = new Mat();
        using var gray = new Mat();

        const string calibFile = "calibration.json";
        double cx = 320.0;
        double cy = 240.0;
        double fFinal;

        if (File.Exists(calibFile))
        {
            string json = File.ReadAllText(calibFile);
            var doc = System.Text.Json.JsonDocument.Parse(json);
            fFinal = doc.RootElement.GetProperty("fFinal").GetDouble();
            Console.WriteLine($"Kalibrierung geladen aus '{calibFile}': f = {fFinal:F2}");
            Console.WriteLine("Wechsle in VMC-Sendemodus. Beenden mit 'ESC'.\n");
        }
        else
        {
            // --- SCHRITT 1: EINMESS-ROUTINE ---
            double[] targetDistances = { 0.3, 1.0, 3.0 };
            double[] calculatedFocalLengths = new double[3];
            int currentStep = 0;

            Console.WriteLine("=== KALIBRIERUNGS-MODUS ===");

            Cv2.NamedWindow("Kalibrierung", WindowFlags.AutoSize);

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

                string msg = $"Bitte Tag FRONTAL bei {targetDistances[currentStep]}m halten und ENTER druecken.";
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

            fFinal = calculatedFocalLengths.Average();

            string saveJson = System.Text.Json.JsonSerializer.Serialize(new { fFinal, cx, cy });
            File.WriteAllText(calibFile, saveJson);

            Console.WriteLine($"\nKalibrierung beendet! Brennweite f = {fFinal:F2}");
            Console.WriteLine($"Kalibrierung gespeichert in '{calibFile}'.");
            Console.WriteLine("Wechsle in VMC-Sendemodus. Beenden mit 'ESC'.\n");
            Cv2.DestroyWindow("Kalibrierung");
        }
        
        using var cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
        cameraMatrix.Set<double>(0, 0, fFinal); cameraMatrix.Set<double>(0, 1, 0);      cameraMatrix.Set<double>(0, 2, cx);
        cameraMatrix.Set<double>(1, 0, 0);      cameraMatrix.Set<double>(1, 1, fFinal); cameraMatrix.Set<double>(1, 2, cy);
        cameraMatrix.Set<double>(2, 0, 0);      cameraMatrix.Set<double>(2, 1, 0);      cameraMatrix.Set<double>(2, 2, 1);

        using var distCoeffs = new Mat(); 

        // --- SCHRITT 2: VMC-SENDEMODUS ---
        var timer = new System.Diagnostics.Stopwatch();

        while (true)
        {
            timer.Restart();
            capture.Read(frame);
            if (frame.Empty()) break;

            Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
            CvAruco.DetectMarkers(gray, dictionary, out Point2f[][] corners, out int[] ids, detectorParameters, out _);

            if (ids != null && ids.Length > 0)
            {
                using var rvecs = new Mat();
                using var tvecs = new Mat();
                CvAruco.EstimatePoseSingleMarkers(corners, (float)tagSize, cameraMatrix, distCoeffs, rvecs, tvecs);

                for (int i = 0; i < ids.Length; i++)
                {
                    Vec3d t = rvecs.At<Vec3d>(i); // rvecs/tvecs Speicherung auslesen
                    Vec3d rvec = rvecs.At<Vec3d>(i);
                    Vec3d tvec = tvecs.At<Vec3d>(i);

                    float tx = (float)tvec.Item0;
                    float ty = (float)tvec.Item1;
                    float tz = (float)tvec.Item2;

                    using var rMat = new Mat();
                    Cv2.Rodrigues(rvec, rMat);
                    
                    // KORREKTUR: Verwende .Get<double>() statt .At<double>(), um C#-Syntaxkonflikte zu vermeiden
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
                    if (tr > 0) {
                        double s = Math.Sqrt(tr + 1.0) * 2;
                        qw = (float)(0.25 * s);
                        qx = (float)((r21 - r12) / s);
                        qy = (float)((r02 - r20) / s);
                        qz = (float)((r10 - r01) / s);
                    } else {
                        qw = 1; qx = 0; qy = 0; qz = 0;
                    }

                    var message = new OscMessage("/VMC/Ext/Tra/Pos", $"AprilTag_{ids[i]}", tx, -ty, tz, qx, -qy, qz, -qw);
                    oscClient.Send(message);

                    Cv2.DrawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, (float)tagSize * 0.5f);
                }
            }

            Cv2.ImShow("VMC-Sender aktiv", frame);
            if (Cv2.WaitKey(1) == 27) break; // ESC

            int sleepTime = 33 - (int)timer.ElapsedMilliseconds;
            if (sleepTime > 0) System.Threading.Thread.Sleep(sleepTime);
        }
    }
}