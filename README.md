# AprilTagVMC

Erkennt AprilTag-Marker per Webcam und sendet Position und Rotation als **VMC-OSC-Pakete** an einen Empfänger im Netzwerk — z. B. an VirtualMotionCapture oder einen kompatiblen Avatar-Controller.

## Voraussetzungen

- Windows 10/11 (x64)
- .NET 10 SDK (zum Kompilieren)
- Webcam
- AprilTag-Marker aus dem Dictionary **36h11** (Markergröße: 50 mm)

## Kompilieren

```
dotnet publish -r win-x64 --self-contained -p:PublishSingleFile=true
```

Die fertige `.exe` liegt danach unter `bin\Release\net10.0\win-x64\publish\`.

## Konfiguration

Alle Einstellungen stehen in `config.ini` im selben Verzeichnis wie die `.exe`.
Die Datei wird nur gelesen, nie vom Programm verändert.

```ini
# OSC-Ziel
OscTargetIp  = 192.168.179.17
OscPort      = 39539

# Physische Kantenlänge des gedruckten Markers in Metern (z.B. 0.05 = 50 mm)
TagSizeMeters = 0.05

# Kamera
CameraIndex  = 0        # 0 = erste verfügbare Kamera
CameraWidth  = 640
CameraHeight = 480
CameraFps    = 30

# Glättung: 0.0 = maximale Glättung (viel Lag)   1.0 = kein Smoothing
SmoothAlpha  = 0.35
```

Fehlt `config.ini`, startet das Programm mit den eingebauten Standardwerten und gibt eine Warnung aus.

## Kalibrierung

Beim ersten Start (bzw. wenn keine `calibration.json` vorhanden ist) startet automatisch der Kalibrierungsmodus:

1. Tag **frontal** vor die Kamera halten — Abstandswerte: **10 cm**, **50 cm**, **1 m**
2. Jeweils **ENTER** drücken, sobald der Tag erkannt und zentriert ist
3. Die berechnete Brennweite wird in `calibration.json` gespeichert und beim nächsten Start automatisch geladen

## Bedienung

| Taste | Funktion |
|---|---|
| `ESC` | Programm beenden |
| `C` | Kalibrierung neu durchführen |
| `K` | Koordinatenanzeige in der Konsole ein-/ausschalten |

## OSC-Protokoll (VMC)

Pro erkanntem Tag werden zwei Nachrichten gesendet:

```
/VMC/Ext/T        f  <uptime_sekunden>
/VMC/Ext/Tra/Pos  s  "AprilTag_<id>"  f x  f y  f z  f qx  f qy  f qz  f qw
```

Position und Rotation werden per **Exponential Moving Average (EMA)** geglättet (Slerp für Quaternionen).

## Abhängigkeiten

- [OpenCvSharp4.Windows](https://github.com/shimat/opencvsharp) — OpenCV-Binding für .NET
- [Vizcon.OSC](https://www.nuget.org/packages/Vizcon.OSC) — OSC UDP-Sender
