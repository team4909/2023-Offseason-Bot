package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CTREHelper;
import frc.lib.LoggedString;
import frc.lib.Logger;
import frc.lib.Logger.CTRESignalMap;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final Pigeon2 m_imu;
  private final Module[] m_modules = new Module[4];
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0));

  private double m_chassisAngleSimRad;
  private final Log m_log;

  public Drivetrain() {
    m_imu = new Pigeon2(DeviceIDs.kIMU, Constants.kCANFDBus);
    m_imu.getConfigurator().apply(new Pigeon2Configuration());
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i] = new Module(i);
    }
    setDefaultCommand(testDrive());
    m_log = new Log();
  }

  @Override
  public void periodic() {
    for (Module module : m_modules) {
      module.periodic();
    }
    m_log.log();
  }

  /**
   * This is done instead of {@link #simulationPeriodic()} because that causes
   * some weird non-deterministic loop time spikes with the ctre sim api
   */
  public void updateModulesSim() {
    double start = Timer.getFPGATimestamp();
    for (Module module : m_modules) {
      module.simulationPeriodic();
    }
    double end = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Time", end - start);
  }

  public void updateGyroSim() {
    Pigeon2SimState simState = m_imu.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simState.setRawYaw(m_chassisAngleSimRad);
  }

  private SwerveModuleState[] calculateSetpointStates(ChassisSpeeds speeds) {
    final double dt = Constants.kPeriodicLoopSeconds;
    Pose2d velocityPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));
    Twist2d moduleStateDelta = new Pose2d().log(velocityPose);
    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
        moduleStateDelta.dx / dt, moduleStateDelta.dy / dt, moduleStateDelta.dtheta / dt);
    m_chassisAngleSimRad = m_chassisAngleSimRad += moduleStateDelta.dtheta;
    return m_kinematics.toSwerveModuleStates(adjustedSpeeds);
  }

  private void setSetpointStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      m_modules[i].set(states[i]);
    }
  }

  private Command testDrive() {
    return this.run(() -> {
      setSetpointStates(calculateSetpointStates(new ChassisSpeeds(1, 0, 0)));
    }).withName("Test Drive");
  }

  private class Log {
    final String subTable = "Drivetrain";
    final CTRESignalMap<Double> imuLoggerMap = CTREHelper.getDeviceLog(m_imu, subTable + "/IMU");
    final LoggedString currentCommand = new LoggedString(subTable, "Current Command");

    void log() {
      Logger.getInstance().logLoggedMap(imuLoggerMap);
      currentCommand.accept(getCurrentCommand() == null ? "null" : getCurrentCommand().getName());
    }
  }
}
