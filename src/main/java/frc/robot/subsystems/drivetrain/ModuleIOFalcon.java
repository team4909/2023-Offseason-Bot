package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.CTREHelper;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Util;

public final class ModuleIOFalcon implements ModuleIO {

  private final int m_index;
  private final double m_magnetOffset;
  private final TalonFX m_driveMotor, m_steerMotor;
  private final CANcoder m_azimuthEncoder;

  private final ArrayList<StatusSignal<Double>> m_driveMotorSignals;
  private final ArrayList<StatusSignal<Double>> m_steerMotorSignals;
  private final ArrayList<StatusSignal<Double>> m_azimuthEncoderSignals;
  private final PositionVoltage m_steerControl;
  private final VelocityVoltage m_driveControl;

  private LinearSystemSim<N1, N1, N1> m_driveSim;
  private FlywheelSim m_steerSim;

  public ModuleIOFalcon(int index) {
    m_index = index;
    switch (m_index) {
      case 0:
        m_driveMotor = new TalonFX(DeviceIDs.kFrontLeftModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kFrontLeftModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kFrontLeftModuleAzimuthEncoder, Constants.kCANFDBus);
        m_magnetOffset = 0.0;
        break;
      case 1:
        m_driveMotor = new TalonFX(DeviceIDs.kFrontRightModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kFrontRightModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kFrontRightModuleAzimuthEncoder, Constants.kCANFDBus);
        m_magnetOffset = 0.0;
        break;
      case 2:
        m_driveMotor = new TalonFX(DeviceIDs.kBackLeftModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kBackLeftModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kBackLeftModuleAzimuthEncoder, Constants.kCANFDBus);
        m_magnetOffset = 0.0;
        break;
      case 3:
        m_driveMotor = new TalonFX(DeviceIDs.kBackRightModuleDriveMotor, Constants.kCANFDBus);
        m_steerMotor = new TalonFX(DeviceIDs.kBackRightModuleSteerMotor, Constants.kCANFDBus);
        m_azimuthEncoder = new CANcoder(DeviceIDs.kBackRightModuleAzimuthEncoder, Constants.kCANFDBus);
        m_magnetOffset = 0.0;
        break;
      default:
        throw new RuntimeException("Invalid Module Index");
    }
    configMotors();
    m_driveMotorSignals = CTREHelper.getRelevantSignals(m_driveMotor);
    m_steerMotorSignals = CTREHelper.getRelevantSignals(m_steerMotor);
    m_azimuthEncoderSignals = CTREHelper.getRelevantSignals(m_azimuthEncoder);
    m_steerControl = new PositionVoltage(0, Constants.kUseFOC, 0, 0, false);
    m_driveControl = new VelocityVoltage(0, Constants.kUseFOC, 0, 0, false);
    if (Constants.kCurrentMode.equals(Mode.kSim)) {
      m_driveSim = new LinearSystemSim<>(
          LinearSystemId.identifyVelocitySystem(DriveConstants.kDrivekV, DriveConstants.kDrivekA));
      // Steer is pretty much inertialess
      m_steerSim = new FlywheelSim(Util.getFalcon500FOCGearbox(1, DriveConstants.kSteerRatio), 1.0, 4e-11);
    }
  }

  public void updateSim() {
    final TalonFXSimState driveSimState = m_driveMotor.getSimState();
    final TalonFXSimState steerSimState = m_steerMotor.getSimState();
    final CANcoderSimState azimuthSimState = m_azimuthEncoder.getSimState();
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    azimuthSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveSim.setInput(driveSimState.getMotorVoltage());
    m_steerSim.setInput(steerSimState.getMotorVoltage());

    m_driveSim.update(0.02);
    m_steerSim.update(0.02);

    Logger.getInstance().recordOutput("Drivetrain/Sim/Module " + m_index + "/Drive Sim Output mps",
        m_driveSim.getOutput(0));
    Logger.getInstance().recordOutput("Drivetrain/Sim/Module " + m_index + "/Drive Motor Output V",
        driveSimState.getMotorVoltage());
    final double driveRotorVelocityRPS = Units
        .radiansToRotations(m_driveSim.getOutput(0) / DriveConstants.kWheelRadiusMeters) * DriveConstants.kDriveRatio;
    driveSimState.addRotorPosition(driveRotorVelocityRPS * 0.02);
    driveSimState.setRotorVelocity(driveRotorVelocityRPS);
    Logger.getInstance().recordOutput("Drivetrain/Sim/Module " + m_index + "/Steer Sim Output rad per s",
        m_steerSim.getAngularVelocityRadPerSec());
    Logger.getInstance().recordOutput("Drivetrain/Sim/Module " + m_index + "/Steer Motor Output V",
        steerSimState.getMotorVoltage());
    // Know that since we are simulating a fused cancoder we cannot simulate the
    // internal rotor's state, just the cancoder
    final double steerVelocityRPS = Units.radiansToRotations(m_steerSim.getAngularVelocityRadPerSec());
    azimuthSimState.addPosition(steerVelocityRPS * 0.02);
    azimuthSimState.setVelocity(steerVelocityRPS);

  }

  private void configMotors() {
    final CANcoderConfiguration azimuthEncoderConfig = new CANcoderConfiguration();
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig); // Factory Default
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    azimuthEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    azimuthEncoderConfig.MagnetSensor.MagnetOffset = m_magnetOffset;
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig);

    final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    m_driveMotor.getConfigurator().apply(driveMotorConfig); // Factory Default
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveMotorConfig.Slot0.kP = DriveConstants.kDrivekP;
    driveMotorConfig.Slot0.kS = DriveConstants.kDrivekS;
    driveMotorConfig.Slot0.kV = DriveConstants.kDrivekF;
    m_driveMotor.getConfigurator().apply(driveMotorConfig);

    final TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
    m_steerMotor.getConfigurator().apply(steerMotorConfig); // Factory Default
    steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    steerMotorConfig.Feedback.FeedbackRemoteSensorID = m_azimuthEncoder.getDeviceID();
    steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotorConfig.Feedback.RotorToSensorRatio = DriveConstants.kSteerRatio;
    steerMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerMotorConfig.Slot0.kP = DriveConstants.kSteerkP;
    m_steerMotor.getConfigurator().apply(steerMotorConfig);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    CTREHelper.updateBulkSignals(m_driveMotorSignals, m_steerMotorSignals, m_azimuthEncoderSignals);
    inputs.drivePositionRotations = m_driveMotor.getPosition().getValue();
    inputs.driveRotorPositionRotations = m_driveMotor.getRotorPosition().getValue();
    inputs.driveVelocityRPS = m_driveMotor.getVelocity().getValue();
    inputs.driveClosedLoopReference = m_driveMotor.getClosedLoopReference().getValue();
    inputs.driveClosedLoopError = m_driveMotor.getClosedLoopError().getValue();
    inputs.driveClosedLoopOutput = m_driveMotor.getClosedLoopOutput().getValue();
    inputs.driveTorqueCurrentAmps = m_driveMotor.getTorqueCurrent().getValue();
    inputs.driveStatorCurrentAmps = m_driveMotor.getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = m_driveMotor.getSupplyCurrent().getValue();
    inputs.driveSupplyVoltage = m_driveMotor.getSupplyVoltage().getValue();
    inputs.driveTempCelsius = m_driveMotor.getDeviceTemp().getValue();

    inputs.steerPositionRotations = m_steerMotor.getPosition().getValue();
    inputs.steerRotorPositionRotations = m_steerMotor.getRotorPosition().getValue();
    inputs.steerVelocityRPS = m_steerMotor.getVelocity().getValue();
    inputs.steerClosedLoopReference = m_steerMotor.getClosedLoopReference().getValue();
    inputs.steerClosedLoopError = m_steerMotor.getClosedLoopError().getValue();
    inputs.steerClosedLoopOutput = m_steerMotor.getClosedLoopOutput().getValue();
    inputs.steerTorqueCurrentAmps = m_steerMotor.getTorqueCurrent().getValue();
    inputs.steerStatorCurrentAmps = m_steerMotor.getStatorCurrent().getValue();
    inputs.steerSupplyCurrentAmps = m_steerMotor.getSupplyCurrent().getValue();
    inputs.steerSupplyVoltage = m_steerMotor.getSupplyVoltage().getValue();
    inputs.steerTempCelsius = m_steerMotor.getDeviceTemp().getValue();

    inputs.azimuthPositionRotations = m_azimuthEncoder.getPosition().getValue();
    inputs.azimuthVelocityRPS = m_azimuthEncoder.getVelocity().getValue();
    inputs.azimuthAbsolutePositionRotations = m_azimuthEncoder.getAbsolutePosition().getValue();
    inputs.azimuthUnfilteredVelocityRPS = m_azimuthEncoder.getUnfilteredVelocity().getValue();
    inputs.azimuthPositionSinceBoot = m_azimuthEncoder.getPositionSinceBoot().getValue();

    inputs.latencyCompensatedDrivePositionRotations = StatusSignal
        .getLatencyCompensatedValue(m_driveMotor.getPosition(), m_driveMotor.getVelocity());
    inputs.latencyCompensatedSteerPositionRotations = StatusSignal
        .getLatencyCompensatedValue(m_steerMotor.getPosition(), m_steerMotor.getVelocity());
  }

  @Override
  public void setSteerRotations(double angleRotations) {
    Logger.getInstance().recordOutput("Module " + m_index + "/Desired Angle Rotations", angleRotations);
    m_steerMotor.setControl(m_steerControl.withPosition(angleRotations));
  }

  @Override
  public void setDriveRPS(double speedRPS) {
    m_driveMotor.setControl(m_driveControl.withVelocity(speedRPS));
  }

  @Override
  public void setSteerVoltage(double volts) {
    m_steerMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveMotor.setVoltage(volts);
  }
}