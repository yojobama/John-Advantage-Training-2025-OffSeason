package frc.robot.subsystems.Transfer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.POM_lib.Motors.POMSparkMax;
import java.util.function.BooleanSupplier;;

public class PhysicalTransferIO implements TransferIO {
    private final POMSparkMax motor;
    private RelativeEncoder encoder; // I don't know if this bloody thing is needed.
    private final SparkMaxConfig config = new SparkMaxConfig();

    public PhysicalTransferIO() {
        motor = new POMSparkMax(Constants.s_TRANSFER_CONSTANTS.kTransferMotorID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.velocity = motor.getEncoder().getVelocity();
        inputs.voltage = (motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.transferSensorInput = motor.getReverseLimitSwitch().isPressed();

        resetEncoder();
        // Logger.recordOutput("Transfer/Spark FW Switch",
        // motor.getForwardLimitSwitch().isPressed());
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public boolean isCoralIn() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}