package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj.RobotController;
import frc.lib.CTREHelper;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs;

public class ImuIOPigeon2 implements ImuIO {

  private final Pigeon2 m_imu;
  private final ArrayList<StatusSignal<Double>> m_imuSignals;

  public ImuIOPigeon2() {
    m_imu = new Pigeon2(DeviceIDs.kImu, Constants.kCANFDBus);
    Pigeon2Configuration imuConfig = new Pigeon2Configuration();
    m_imu.getConfigurator().apply(imuConfig);
    m_imuSignals = CTREHelper.getRelevantSignals(m_imu);
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    CTREHelper.updateBulkSignals(m_imuSignals);
    inputs.yawPositionDeg = m_imu.getYaw().getValue();
    inputs.pitchPositionDeg = m_imu.getPitch().getValue();
    inputs.rollPositionDeg = m_imu.getRoll().getValue();
    inputs.yawVelocityDegPerSec = m_imu.getAngularVelocityZ().getValue();
    inputs.pitchVelocityDegPerSec = m_imu.getAngularVelocityX().getValue();
    inputs.rollVelocityDegPerSec = m_imu.getAngularVelocityY().getValue();
    inputs.supplyVoltage = m_imu.getSupplyVoltage().getValue();
    inputs.upTimeSeconds = m_imu.getUpTime().getValue();
    inputs.tempCelsius = m_imu.getTemperature().getValue();

    inputs.latencyCompensatedYaw = StatusSignal.getLatencyCompensatedValue(m_imu.getYaw(), m_imu.getAngularVelocityZ());
  }

  public void updateSim(double dThetaRad) {
    Pigeon2SimState imuSimState = m_imu.getSimState();
    imuSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double dThetaDeg = Math.toDegrees(dThetaRad);
    imuSimState.addYaw(dThetaDeg);
  }
}
