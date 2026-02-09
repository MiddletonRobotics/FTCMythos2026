package org.firstinspires.ftc.teamcode.command_factories;

import androidx.core.math.MathUtils;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.ConditionalCommand;
import org.firstinspires.ftc.library.command.ParallelCommandGroup;
import org.firstinspires.ftc.library.command.ParallelDeadlineGroup;
import org.firstinspires.ftc.library.command.WaitCommand;
import org.firstinspires.ftc.library.command.WaitUntilCommand;
import org.firstinspires.ftc.library.math.MathUtility;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.concurrent.locks.Condition;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SuperstructureFactory {
    public static Command rapidFireCommand(Intake intake, Transfer transfer, Shooter shooter, LED led, BooleanSupplier ignoreFlywheelSetpointReached) {
        return Commands.sequence(
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0),
                Commands.either(
                        Commands.sequence(
                                new WaitUntilCommand(shooter::flywheelAtSetpoint).interruptOn(ignoreFlywheelSetpointReached),
                                new ConditionalCommand(
                                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition).alongWith(LEDFactory.setAdvancedStandardBlinkingCommand(led, LEDConstants.ColorValue.ORANGE, LEDConstants.ColorValue.VIOLET, () -> 130)),
                                        Commands.none(),
                                        transfer::isBlockerEngaged
                                ),
                                LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.GREEN, () -> 170),
                                rapidFireSequence(intake, transfer)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() > 0
                ),
                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                LEDFactory.setConstantColorCommand(led, LEDConstants.ColorValue.ORANGE)
        );
    }

    private static Command rapidFireSequence(Intake intake, Transfer transfer) {
        return Commands.sequence(
                shootSingleBall(intake, transfer),
                Commands.either(
                        Commands.sequence(
                                indexNextBall(intake, transfer),
                                shootSingleBall(intake, transfer)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() >= 1
                ),
                Commands.either(
                        Commands.sequence(
                                indexNextBall(intake, transfer),
                                shootSingleBall(intake, transfer)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() >= 1
                )
        );
    }

    public static Command controlledShootCommand(Intake intake, Transfer transfer, Shooter shooter, LED led, BooleanSupplier ignoreFlywheelSetpointReached) {
        return Commands.sequence(
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0),
                Commands.either(
                        Commands.sequence(
                                new WaitUntilCommand(shooter::flywheelAtSetpoint).interruptOn(ignoreFlywheelSetpointReached),
                                new ConditionalCommand(
                                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition).alongWith(LEDFactory.setAdvancedStandardBlinkingCommand(led, LEDConstants.ColorValue.ORANGE, LEDConstants.ColorValue.VIOLET, () -> 130)),
                                        Commands.none(),
                                        transfer::isBlockerEngaged
                                ),
                                LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.GREEN, () -> 170),
                                controlledFireSequence(intake, transfer, shooter)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() > 0
                ),
                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                LEDFactory.setConstantColorCommand(led, LEDConstants.ColorValue.ORANGE)
        );
    }

    private static Command controlledFireSequence(Intake intake, Transfer transfer, Shooter shooter) {
        return Commands.sequence(
                shootSingleBall(intake, transfer),
                Commands.either(
                        Commands.sequence(
                                indexNextBall(intake, transfer),
                                new WaitUntilCommand(shooter::flywheelAtSetpoint),
                                shootSingleBall(intake, transfer)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() >= 1
                ),
                Commands.either(
                        Commands.sequence(
                                indexNextBall(intake, transfer),
                                new WaitUntilCommand(shooter::flywheelAtSetpoint),
                                shootSingleBall(intake, transfer)
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() >= 1
                )
        );
    }

    /**
     * Improved shootSingleBall with better timing and state verification
     */
    private static Command shootSingleBall(Intake intake, Transfer transfer) {
        return Commands.sequence(
                Commands.either(
                        Commands.sequence(
                                // If ball not at third beam, index it there first
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.5),
                                Commands.waitUntil(transfer::isThirdBeamBroken).withTimeout(1000),
                                Commands.waitMillis(100), // Settling time
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0)
                        ),
                        Commands.none(),
                        () -> !transfer.isThirdBeamBroken()  // Only run if ball not already there
                ),
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1.0),
                new ConditionalCommand(
                        TransferFactory.runKickerCycle(transfer),
                        Commands.none(),
                        transfer::doesTransferContainSingleBall
                ),
                Commands.waitUntil(() -> !transfer.isThirdBeamBroken()).withTimeout(1000),
                Commands.waitMillis(200),
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0),
                Commands.waitMillis(100)
        );
    }

    /**
     * Improved indexNextBall with better state verification and timeout handling
     */
    private static Command indexNextBall(Intake intake, Transfer transfer) {
        return Commands.sequence(
                Commands.either(
                        Commands.sequence(
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 1.0),
                                Commands.waitUntil(() -> !transfer.isThirdBeamBroken()).withTimeout(500),
                                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0),
                                Commands.waitMillis(100)
                        ),
                        Commands.none(),
                        transfer::isThirdBeamBroken
                ),
                Commands.waitMillis(150),
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.8),
                Commands.waitUntil(transfer::isThirdBeamBroken).withTimeout(1500),
                Commands.waitMillis(200),
                Commands.either(
                        Commands.sequence(
                                Commands.waitUntil(transfer::isThirdBeamBroken).withTimeout(300),
                                Commands.waitMillis(100)
                        ),
                        Commands.none(),
                        () -> !transfer.isThirdBeamBroken()
                ),
                IntakeFactory.setUnevenOpenLoopSetpointCommand(intake, () -> 0.0),
                Commands.waitMillis(100)
        );
    }

    public static Command auomaticallyParkAndLiftCommand(Drivetrain drivetrain, Lift lift) {
        return Commands.runEnd(
                () -> {
                    PathBuilder otfToParkingPath = drivetrain.getPathBuilder();

                    otfToParkingPath.addPath(
                        new BezierLine(
                            drivetrain.getDrivetrainParkingPose(),
                            DrivetrainConstants.kTopRightParkingPoseBlue
                        )
                    );
                    otfToParkingPath.setLinearHeadingInterpolation(drivetrain.getPose().getAsPedroPose().getHeading(), DrivetrainConstants.kTopRightParkingPoseBlue.getHeading());

                    drivetrain.followTrajectory(otfToParkingPath.build(), true);

                    if(drivetrain.isDrivetrainAtSetpoint() && drivetrain.getVelocity() == 0) {
                        lift.setPosition(1000);
                    }
                },
                () -> lift.setPower(0.0),
                drivetrain, lift
        ).until(lift::isAtSetpoint);
    }
}
