package frc.robot.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

public class TransferSubsystem {
    TransferIO m_TransferIO;

    public TransferSubsystem(TransferIO transferIO) {
        m_TransferIO = transferIO;
    }

    // a command to run the transfer in intake until the limit switch is triggered
    public Command runIntakeUntilLimitSwitch() {
        return Commands.runOnce(() -> m_TransferIO.setSpeed(0.0))
                .until(() -> m_TransferIO.isCoralIn())
                .andThen(() -> m_TransferIO.stopMotor());
    }

    // a command to set the bloody voltage
    public Command setBloodyVoltage(double voltage, BooleanSupplier limitSwitch) {
        return Commands.runOnce(() -> m_TransferIO.setVoltage(voltage))
                .until(limitSwitch)
                .andThen(() -> m_TransferIO.stopMotor());
    }
}
