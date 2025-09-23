package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Transfer.TransferSubsystem;

public class TransferCommands {

    public static Command setVoltage(TransferSubsystem subsystem, double voltage)  {
        return Commands.runEnd(() -> subsystem.getTransferIO().setVoltage(voltage)
                , () -> subsystem.getTransferIO().stopMotor()
                , subsystem);
    }

    public static Command DriveTransferForOneSecond(TransferSubsystem subsystem) {
        return Commands.runOnce(() -> subsystem.getTransferIO().setSpeed(0.5), subsystem)
                .andThen(() -> Commands.waitSeconds(1.0)).andThen(() -> subsystem.getTransferIO().setSpeed(0.5), subsystem);
    }
}
