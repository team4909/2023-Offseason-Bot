package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0));
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  // private final SwerveDrivePoseEstimator m_poseEstimator = new
  // SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(),
  // new SwerveModulePosition[4], new Pose2d());

  public Drivetrain() {
    setName("Drivetrain");
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i] = new Module(new ModuleIOFalcon(i), i);
    }
    setDefaultCommand(testDrive());
  }

  @Override
  public void periodic() {
    for (var module : m_modules) {
      module.periodic();
    }
  }

  private SwerveModuleState[] calculateSetpointStates(ChassisSpeeds speeds) {
    final double dt = 0.02;
    Pose2d velocityPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));
    Twist2d moduleStateDelta = new Pose2d().log(velocityPose);
    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
        moduleStateDelta.dx / dt, moduleStateDelta.dy / dt, moduleStateDelta.dtheta / dt);
    return m_kinematics.toSwerveModuleStates(adjustedSpeeds);
  }

  private void setSetpointStates(SwerveModuleState[] states) {
    Logger.getInstance().recordOutput("Drivetrain/Setpoint Module States", states);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < states.length; i++) {
      optimizedStates[i] = m_modules[i].setSetpoint(states[i]);
    }
    Logger.getInstance().recordOutput("Drivetrain/Optimized Setpoint Module States", optimizedStates);
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = m_modules[i].getState();
    }
    Logger.getInstance().recordOutput("Drivetrain/Measured Module States", measuredStates);

  }

  float test = 1;

  private Command testDrive() {
    return this.run(() -> {
      if (test < 4)
        test += 0.05;
      else if (test >= 4)
        test *= -1;

      setSetpointStates(calculateSetpointStates(new ChassisSpeeds(0, 0, test)));
    }).withName("Test Drive");
  }
}
