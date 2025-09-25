package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Lift.LiftSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.s_LIFT_CONSTANTS.*;

public class LiftCommands {
    public static Command goToPosition(LiftSubsystem lift, double position) {
        return new FunctionalCommand(() -> {
            System.out.println("going to position: " + position);
            lift.getIO().stopMotor();
            lift.getIO().resetPID(position);
        },
                () -> lift.getIO().setGoal(position),
                interrupted -> {
                    lift.getIO().stopMotor();
                    if (interrupted) {
                        System.out.println("interupted go to: " + position);
                    }
                },
                lift.getIO().atGoal(), lift);
    }

    public static Command stopElevator(LiftSubsystem elevator) {
        return Commands.runOnce(() -> elevator.getIO().stopMotor(), elevator);
    }

    public static Command onlyFeedForward(LiftSubsystem elevator, double velocity) {
        return Commands.run(() -> elevator.getIO().setFeedForward(velocity), elevator);
    }

    public static Command closeUntilSwitch(LiftSubsystem elevator) {
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(CLOSE_ELEVATOR_SPEED), elevator)
                .until(elevator.getIO()::isPressed);
    }

    public static Command goToPositionWithoutPid(LiftSubsystem elevator, double position) {
        return Commands
                .run(() -> elevator.getIO()
                                .setVoltageWithResistGravity(Math.copySign(4, position - elevator.getIO().getPosition())),
                        elevator)
                .until(() -> (Math.abs(elevator.getIO().getPosition() - position) < 1));
    }

    public static Command setSpeed(LiftSubsystem elevator, double speed) {
        return Commands.run(() -> elevator.getIO().setSpeed(speed), elevator);
    }

    public static Command closeElevator(LiftSubsystem elevator) {
        return goToPosition(elevator, 0)
                .andThen(closeUntilSwitch(elevator)).andThen(elevator.getIO()::stopMotor);
    }

    public static Command L2(LiftSubsystem elevator) {
        return goToPosition(elevator, L2_POSITION);
    }

    public static Command L3(LiftSubsystem elevator) {
        return goToPosition(elevator, L3_POSITION);
    }

    public static Command closeElevatorManual(LiftSubsystem elevator, DoubleSupplier voltage) {
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(voltage.getAsDouble()), elevator);
    }

    public static Command openElevatorManual(LiftSubsystem elevator, DoubleSupplier voltage) {
        return Commands.run(() -> elevator.getIO().setVoltage(voltage.getAsDouble()), elevator);
    }

}
