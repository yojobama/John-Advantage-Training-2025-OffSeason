package frc.robot.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

public class TransferSubsystem extends SubsystemBase {
    TransferIO m_TransferIO;

    public TransferSubsystem(TransferIO transferIO) {
        m_TransferIO = transferIO;
    }

    public TransferIO getTransferIO() {
        return m_TransferIO;
    }

    public Command intakeUntilCoralDetected()
    {
        return Commands.runEnd(() -> getTransferIO().setSpeed(Constants.s_TRANSFER_CONSTANTS.kTransferSpeed),
                        () -> getTransferIO().setSpeed(0.0),
                        this)
                .until(() -> m_TransferIO.isCoralIn());
    }
}
