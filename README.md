# 🤖 Excalibur #6738 - Rebuilt 2026 Code Repository  
*By the Excalibur FRC Software Team ⚔️*

---

## 📌 About This Repository

This is the official public codebase for our **FRC 2026 Rebuilt Season** robot — *Galahad*.  
It includes complete logic for:

- ✅ Robot movement  
- ✅ Subsystem control 2026
- ✅ Automation & autonomous modes  
- ✅ Command bindings and more!

---

## 🛡️ Meet Our Robot: *Galahad*

Galahad is our proud warrior for the 2026 season. It will represent us in:

- 🇮🇱 **ISR District #1**  
- 🇮🇱 **ISR District #4**  

### 📷 Robot Snap
![Galahad](https://www.excaliburfrc.com/img/galahad.png)

---

## 📁 Project Structure

Here's an overview of our code layout:

```
src/main/java/frc/
├── excalib2/            ← ExcaLib v2 — our reusable library (Phoenix 6 FOC-first)
│   ├── device/          CANDeviceId, ExcaTalonFX, DeviceConfigs (apply+verify+retry), SignalHub
│   ├── control/         ControlMode, Gains, MotionConstraints (MotionMagic/Expo), CurrentBudget
│   ├── mechanisms/      MechanismConfig + Positional / Velocity / Roller archetypes
│   ├── statemachine/    StateMachine<S> — transition table, guards, enter/while/exit
│   ├── superstructure/  Superstructure<G> goal-coordinator base
│   ├── swerve/          SwerveSubsystem (CTRE swerve), DriveCommands, DriveToPose, MegaTag2
│   ├── auto/            PathPlanner glue (named commands, auto chooser)
│   ├── telemetry/       Telemetry (DogLog+SignalLogger), TunableNumber, FaultReporter
│   ├── sim/             MechanismSim — TalonFXSimState + WPILib physics models
│   └── util/            AllianceFlip, Zones
│
├── excalib/             ← ExcaLib v1 (legacy — kept until removal is approved)
│
└── robot/
    ├── Robot.java / RobotContainer.java / Constants.java / SwerveConfig.java
    ├── subsystems/      intake / shooter / transport — built on the excalib2 archetypes
    ├── superstructure/  Superstructure (RobotState goal machine) + RobotState
    └── util/            ShiftUtil, TurretOffsetGetter, BallCounter, LimelightHelpers
```

📖 **Adding a subsystem?** See [docs/ADDING_A_SUBSYSTEM.md](docs/ADDING_A_SUBSYSTEM.md).
📚 Library background: [docs/AUDIT.md](docs/AUDIT.md) · [docs/RESEARCH.md](docs/RESEARCH.md) · [docs/DESIGN.md](docs/DESIGN.md) · credits in [NOTICE.md](NOTICE.md).

---

## 🔗 Follow Us!

Stay up to date with our team and events:

- 📺 [YouTube](https://youtube.com/ExcaliburFRC)
- 📸 [Instagram](https://www.instagram.com/excalibur_6738/)
- 📘 [Facebook](https://facebook.com/excalibur6738)
- 🧑‍💻 [GitHub](https://github.com/ExcaliburFRC/)
- 🌐 [Website](https://www.excaliburfrc.com/)

---

