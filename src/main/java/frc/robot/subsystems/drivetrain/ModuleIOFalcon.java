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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.CTREHelper;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Util;

public final class ModuleIOFalcon implements ModuleIO {

  private final int m_index;
  private final String m_name;
  private final TalonFX m_driveMotor, m_steerMotor;
  private final CANcoder m_azimuthEncoder;

  private final ArrayList<StatusSignal<Double>> m_driveMotorSignals;
  private final ArrayList<StatusSignal<Double>> m_steerMotorSignals;
  private final ArrayList<StatusSignal<Double>> m_azimuthEncoderSignals;

  private FlywheelSim m_driveSim, m_steerSim;

  public ModuleIOFalcon(int index) {
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
    m_driveMotorSignals = CTREHelper.getRelevantSignals(m_driveMotor);
    m_steerMotorSignals = CTREHelper.getRelevantSignals(m_steerMotor);
    m_azimuthEncoderSignals = CTREHelper.getRelevantSignals(m_azimuthEncoder);

    if (Constants.kCurrentMode.equals(Mode.kSim)) {
      m_driveSim = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(DriveConstants.kDrivekV, DriveConstants.kDrivekA),
          Util.getFalcon500FOC(1), DriveConstants.kDriveRatio);
      // For all intents and purposes, the steer is inertialess
      m_steerSim = new FlywheelSim(Util.getFalcon500FOC(1), DriveConstants.kTurnRatio, 1e-10);
    }
  }

  public void updateSim() {
    TalonFXSimState driveSimState = m_driveMotor.getSimState();
    TalonFXSimState steerSimState = m_steerMotor.getSimState();
    CANcoderSimState azimuthSimState = m_azimuthEncoder.getSimState();
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    azimuthSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveSim.setInputVoltage(driveSimState.getMotorVoltage());
    m_steerSim.setInputVoltage(steerSimState.getMotorVoltage());
    m_driveSim.update(0.02);
    m_steerSim.update(0.02);
    double driveAngularVelocityRPS = m_driveSim.getAngularVelocityRadPerSec()
        / ((1.0 / DriveConstants.kDriveRatio) * DriveConstants.kWheelCircumference);
    driveSimState.addRotorPosition(driveAngularVelocityRPS * 0.02);
    driveSimState.setRotorVelocity(driveAngularVelocityRPS);
    double steerAngularVelocityRPS = m_steerSim.getAngularVelocityRPM() / 60;
    azimuthSimState.addPosition(steerAngularVelocityRPS * 0.02);
    azimuthSimState.setVelocity(steerAngularVelocityRPS);
    double angularVelocityRotor = steerAngularVelocityRPS * DriveConstants.kTurnRatio;
    steerSimState.addRotorPosition(angularVelocityRotor * 0.02);
    steerSimState.setRotorVelocity(angularVelocityRotor);
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
    m_driveMotor.getConfigurator().apply(driveMotorConfig);

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

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    CTREHelper.updateBulkSignals(m_driveMotorSignals, m_steerMotorSignals, m_azimuthEncoderSignals);
    inputs.drivePositionRotations = m_driveMotor.getPosition().getValue();
    inputs.driveVelocityRPS = m_driveMotor.getVelocity().getValue();
    inputs.driveClosedLoopReference = m_driveMotor.getClosedLoopReference().getValue();
    inputs.driveClosedLoopError = m_driveMotor.getClosedLoopError().getValue();
    inputs.driveTorqueCurrentAmps = m_driveMotor.getTorqueCurrent().getValue();
    inputs.driveStatorCurrentAmps = m_driveMotor.getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = m_driveMotor.getSupplyCurrent().getValue();
    inputs.driveSupplyVoltage = m_driveMotor.getSupplyVoltage().getValue();
    inputs.driveTempCelsius = m_driveMotor.getDeviceTemp().getValue();

    inputs.steerPositionRotations = m_steerMotor.getPosition().getValue();
    inputs.steerVelocityRPS = m_steerMotor.getVelocity().getValue();
    inputs.steerClosedLoopReference = m_steerMotor.getClosedLoopReference().getValue();
    inputs.steerClosedLoopError = m_steerMotor.getClosedLoopError().getValue();
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

    inputs.latencyCompensatedPositionRotations = StatusSignal
        .getLatencyCompensatedValue(m_azimuthEncoder.getAbsolutePosition(), m_azimuthEncoder.getVelocity());
  }

  @Override
  public void setSteerRotations(double angleRotations) {
    Logger.getInstance().recordOutput("Module " + m_index + "/Desired Angle Rotations", angleRotations);
    m_steerMotor.setControl(new PositionVoltage(angleRotations, Constants.kUseFOC, 0, 0, false));
  }

  @Override
  public void setDriveRPS(double speedRPS) {
    m_driveMotor.setControl(new VelocityVoltage(speedRPS, Constants.kUseFOC, 0, 0, false));
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