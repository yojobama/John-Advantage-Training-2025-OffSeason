package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

;

public class PhysicalTransferIO implements TransferIO {
    private final POMSparkMax m_TransferMotor;
    private final SparkMaxConfig m_TransferMotorConfig = new SparkMaxConfig();
    private final POMDigitalInput m_InputThingy;

    public PhysicalTransferIO() {
        m_TransferMotor = new POMSparkMax(Constants.s_TRANSFER_CONSTANTS.kTransferMotorID, MotorType.kBrushless);
        m_TransferMotorConfig.idleMode(IdleMode.kCoast);
        m_TransferMotor.configure(m_TransferMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_InputThingy = new POMDigitalInput(Constants.s_TRANSFER_CONSTANTS.kTransferSensorID);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.velocity = m_TransferMotor.getEncoder().getVelocity();
        inputs.voltage = (m_TransferMotor.getAppliedOutput() * m_TransferMotor.getBusVoltage());
        inputs.transferSensorInput = m_TransferMotor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void setSpeed(double speed) {
        m_TransferMotor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        m_TransferMotor.setVoltage(voltage);
    }

    public void stopMotor() {
        m_TransferMotor.stopMotor();
    }

    @Override
    public boolean isCoralIn() {
        return m_InputThingy.get();
    }
}