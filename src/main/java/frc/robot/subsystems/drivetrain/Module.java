package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.CTREHelper;
import frc.lib.LoggedDouble;
import frc.lib.LoggedString;
import frc.lib.Logger;
import frc.lib.Logger.CTRESignalMap;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Util;

public class Module {
  private final int m_index;
  private final String m_name;
  private final TalonFX m_driveMotor, m_steerMotor;
  private final CANcoder m_azimuthEncoder;

  private FlywheelSim m_driveSim, m_steerSim;
  private final Log m_log;

  public Module(int index) {
    m_index = index;
    switch (m_index) {
      case 0:
        m_name = "FL";
        m_driveMotor = new TalonFX(DeviceIDs.kFrontLeftModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kFrontLeftModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kFrontLeftModuleAzimuthEncoder, Constants.kCANFDBus);
        break;
      case 1:
        m_name = "FR";
        m_driveMotor = new TalonFX(DeviceIDs.kFrontRightModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kFrontRightModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kFrontRightModuleAzimuthEncoder, Constants.kCANFDBus);
        break;
      case 2:
        m_name = "BL";
        m_driveMotor = new TalonFX(DeviceIDs.kBackLeftModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kBackLeftModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kBackLeftModuleAzimuthEncoder, Constants.kCANFDBus);
        break;
      case 3:
        m_name = "BR";
        m_driveMotor = new TalonFX(DeviceIDs.kBackRightModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kBackRightModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kBackRightModuleAzimuthEncoder, Constants.kCANFDBus);
        break;
      default:
        throw new RuntimeException("Invalid Module Index");
    }
    configMotors();

    if (Constants.kSim) {
      m_driveSim = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(DriveConstants.kDrivekV, DriveConstants.kDrivekA),
          Util.getFalcon500FOC(1), DriveConstants.kDriveRatio);
      // For all intents and purposes, the steer is inertialess
      m_steerSim = new FlywheelSim(Util.getFalcon500FOC(1), DriveConstants.kTurnRatio, 1e-10);
    }
    FaultManager faultManager = FaultManager.getInstance();
    faultManager.addTalon(m_driveMotor);
    faultManager.addTalon(m_steerMotor);
    m_log = new Log();
  }

  public void simulationPeriodic() {
    TalonFXSimState driveSimState = m_driveMotor.getSimState();
    TalonFXSimState steerSimState = m_steerMotor.getSimState();
    CANcoderSimState azimuthSimState = m_azimuthEncoder.getSimState();
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    azimuthSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveSim.setInputVoltage(driveSimState.getMotorVoltage());
    m_steerSim.setInputVoltage(steerSimState.getMotorVoltage());
    m_driveSim.update(Constants.kPeriodicLoopSeconds);
    m_steerSim.update(Constants.kPeriodicLoopSeconds);

    double driveAngularVelocityRPS = m_driveSim.getAngularVelocityRadPerSec()
        / ((1.0 / DriveConstants.kDriveRatio) * DriveConstants.kWheelCircumference);
    driveSimState.addRotorPosition(driveAngularVelocityRPS * Constants.kPeriodicLoopSeconds);
    driveSimState.setRotorVelocity(driveAngularVelocityRPS);
    double steerAngularVelocityRPS = m_steerSim.getAngularVelocityRPM() / 60;
    azimuthSimState.addPosition(steerAngularVelocityRPS * Constants.kPeriodicLoopSeconds);
    azimuthSimState.setVelocity(steerAngularVelocityRPS);
  }

  public void periodic() {
    m_log.log();
    m_steerMotor.set(0.1);
  }

  public void set(SwerveModuleState desiredState) {
    m_log.desiredState.accept(desiredState.toString());
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
        new Rotation2d(MathUtil.angleModulus(getSwerveModuleState().angle.getRadians())));
    m_log.optimizedDesiredState.accept(optimizedDesiredState.toString());
    m_driveMotor.setControl(
        new VelocityVoltage(optimizedDesiredState.speedMetersPerSecond / DriveConstants.kDriveMetersPerRotation));
    m_steerMotor.setControl(new PositionVoltage(Units.degreesToRotations(optimizedDesiredState.angle.getDegrees())));
  }

  public SwerveModuleState getSwerveModuleState() {
    double speed = m_driveMotor.getVelocity().asSupplier().get();
    Rotation2d angle = Rotation2d
        .fromDegrees(Units.rotationsToDegrees(BaseStatusSignal.getLatencyCompensatedValue(
            m_azimuthEncoder.getAbsolutePosition(),
            m_azimuthEncoder.getVelocity())));
    return new SwerveModuleState(speed, angle);
  }

  private void configMotors() {
    CANcoderConfiguration azimuthEncoderConfig = new CANcoderConfiguration();
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig); // Factory Default
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    azimuthEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    azimuthEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig);

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    m_driveMotor.getConfigurator().apply(driveMotorConfig); // Factory Default
    driveMotorConfig.Slot0.kP = DriveConstants.kDrivekP;
    driveMotorConfig.Slot0.kS = DriveConstants.kDrivekS;
    driveMotorConfig.Slot0.kV = DriveConstants.kDrivekV;
    Util.logFalconError(m_driveMotor.getConfigurator().apply(driveMotorConfig), m_driveMotor.getDescription());

    TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
    m_steerMotor.getConfigurator().apply(steerMotorConfig); // Factory Default
    steerMotorConfig.Feedback.FeedbackRemoteSensorID = m_azimuthEncoder.getDeviceID();
    steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotorConfig.Feedback.RotorToSensorRatio = DriveConstants.kTurnRatio;
    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerMotorConfig.Slot0.kP = DriveConstants.kSteerkP;

    m_steerMotor.getConfigurator().apply(steerMotorConfig);
  }

  private class Log {
    final String subTable = "Drivetrain/Module " + m_name;
    final CTRESignalMap<Double> driveMotorLoggerMap = CTREHelper.getDeviceLog(m_driveMotor,
        subTable + "/Drive Motor");
    final CTRESignalMap<Double> steerMotorLoggerMap = CTREHelper.getDeviceLog(m_steerMotor,
        subTable + "/Steer Motor");
    final CTRESignalMap<Double> azimuthEncoderLoggerMap = CTREHelper.getDeviceLog(m_azimuthEncoder,
        subTable + "/Azimuth Encoder");
    final LoggedDouble driveSimCurrentDraw = new LoggedDouble(subTable, "Drive Sim Current Draw");
    final LoggedDouble steerSimCurrentDraw = new LoggedDouble(subTable, "Steer Sim Current Draw");
    final LoggedDouble driveSimAngularVelocityRPM = new LoggedDouble(subTable, "Drive Sim Angular Velocity RPM");
    final LoggedDouble steerSimAngularVelocityRPM = new LoggedDouble(subTable, "Steer Sim Angular Velocity RPM");
    final LoggedString state = new LoggedString(subTable, "State");
    final LoggedString desiredState = new LoggedString(subTable, "Desired State");
    final LoggedString optimizedDesiredState = new LoggedString(subTable, "Optimized Desired State");

    void log() {
      Logger.getInstance().logLoggedMap(driveMotorLoggerMap, steerMotorLoggerMap, azimuthEncoderLoggerMap);
      state.accept(getSwerveModuleState().toString());
      if (Constants.kSim) {
        driveSimCurrentDraw.accept(m_driveSim.getCurrentDrawAmps());
        steerSimCurrentDraw.accept(m_steerSim.getCurrentDrawAmps());
        driveSimAngularVelocityRPM.accept(m_driveSim.getAngularVelocityRPM());
        steerSimAngularVelocityRPM.accept(m_steerSim.getAngularVelocityRPM());
      }
    }
  }
}
