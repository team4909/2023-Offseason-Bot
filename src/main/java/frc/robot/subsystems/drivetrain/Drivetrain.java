package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;

public final class Drivetrain extends Subsystem {

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, DriveConstants.kWheelbaseMeters / 2.0),
      new Translation2d(-DriveConstants.kTrackwidthMeters / 2.0, -DriveConstants.kWheelbaseMeters / 2.0));
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final ImuIO m_imuIO;
  private final ImuIOInputsAutoLogged m_imuInputs = new ImuIOInputsAutoLogged();
  private DoubleSupplier m_leftX, m_leftY, m_rightX;

  public Drivetrain(ModuleIO[] moduleIOs, ImuIO imuIO) {
    setName("Drivetrain");
    final boolean isReplay = Constants.kCurrentMode.equals(Mode.kReplay);
    for (int i = 0; i < m_modules.length; i++)
      m_modules[i] = new Module(isReplay ? new ModuleIO() {} : new ModuleIOFalcon(i), i);
    m_imuIO = isReplay ? new ImuIO() {} : new ImuIOPigeon2();
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
        Rotation2d.fromDegrees(m_imuInputs.latencyCompensatedYaw), getModulePositions(), new Pose2d());
    setDefaultCommand(testDrive());
  }

  @Override
  public void periodic() {
    for (var module : m_modules) {
      module.periodic();
    }
    m_imuIO.updateInputs(m_imuInputs);
    Logger.getInstance().processInputs("Drivetrain/IMU", m_imuInputs);
    Logger.getInstance().recordOutput("Drivetrain/Yaw Radians", Math.toRadians(m_imuInputs.latencyCompensatedYaw));
    m_poseEstimator.update(Rotation2d.fromDegrees(m_imuInputs.latencyCompensatedYaw), getModulePositions());
  }

  private SwerveModuleState[] calculateSetpointStates(ChassisSpeeds speeds) {
    final double dtSeconds = 0.02;
    return m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromDiscreteSpeeds(speeds, dtSeconds));
  }

  private void setSetpointStates(SwerveModuleState[] states) {
    final SwerveModuleState[] optimizedStates = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < states.length; i++)
      optimizedStates[i] = m_modules[i].setSetpoint(states[i]);
    Logger.getInstance().recordOutput("Drivetrain/Setpoint Module States", optimizedStates);
    final SwerveModuleState[] measuredStates = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++)
      measuredStates[i] = m_modules[i].getState();
    Logger.getInstance().recordOutput("Drivetrain/Measured Module States", measuredStates);
  }

  private SwerveModulePosition[] getModulePositions() {
    final SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++)
      positions[i] = m_modules[i].getPosition();
    return positions;
  }

  public void setJoystickAxes(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
    m_leftX = leftX;
    m_leftY = leftY;
    m_rightX = rightX;
  }

  float test = 1;

  private Command testDrive() {
    return this.run(() -> {
      if (test < 6)
        test += 0.05;
      else if (test >= 4)
        test *= -1;

      setSetpointStates(calculateSetpointStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 12, Rotation2d.fromDegrees(m_imuInputs.latencyCompensatedYaw))));
    }).withName("Test Drive");
  }

  private Command idle() {
    return this.run(() -> {
      for (Module module : m_modules)
        module.stop();
    });

  }

  private Command joystickDrive() {
    return this.run(() -> {
      setSetpointStates(calculateSetpointStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              m_leftX.getAsDouble() * DriveConstants.kMaxLinearSpeedMetersPerSec,
              m_leftY.getAsDouble() * DriveConstants.kMaxLinearSpeedMetersPerSec,
              m_rightX.getAsDouble() * DriveConstants.kMaxAngularSpeedRadPerSec,
              Rotation2d.fromDegrees(m_imuInputs.latencyCompensatedYaw))));
    });
  }

}
