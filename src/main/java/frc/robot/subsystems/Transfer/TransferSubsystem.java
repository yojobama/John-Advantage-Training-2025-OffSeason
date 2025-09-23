package frc.robot.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TransferSubsystem {
    ITransferIO m_TransferIO;

    public TransferSubsystem(ITransferIO transferIO) {
        m_TransferIO = transferIO;
    }

    // a command to run the transfer in intake until the limit switch is triggered
    public Command runIntakeUntilLimitSwitch() {
        //    return null;
        return Commands.runOnce(m_TransferIO.setTransferOpenLoop(0.0 /* add some constant that doest that  */)).until().andThen();
    }
}
