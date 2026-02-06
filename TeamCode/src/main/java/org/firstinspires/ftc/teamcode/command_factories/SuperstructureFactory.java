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
import java.util.function.DoubleSupplier;

public class SuperstructureFactory {
    @Deprecated
    public static Command smartVelocityRampCommand(Intake intake, Transfer transfer, Shooter shooter, LED led, DoubleSupplier shooterRPM, DoubleSupplier hoodPosition) {
        return Commands.sequence( // TODO: Add a check to see if we have balls, if not then just return (maybe add it to the blink array)
                new ConditionalCommand(
                        Commands.none(),
                        new ParallelCommandGroup(
                                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition),
                                LEDFactory.setAdvancedStandardBlinkingCommand(led, LEDConstants.ColorValue.ORANGE, LEDConstants.ColorValue.VIOLET, () -> 50)
                        ),
                        transfer::isBlockerEngaged
                ),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0),
                new ParallelCommandGroup(
                        ShooterFactory.velocitySetpointCommand(shooter, shooterRPM),
                        ShooterFactory.hoodPositionCommand(shooter, hoodPosition),
                        LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.YELLOW, () -> 100)
                ),
                LEDFactory.setStandardBlinkingCommand(led, LEDConstants.ColorValue.GREEN, () -> 50)
        );
    }

    public static Command smartShootingCommand(Intake intake, Transfer transfer, LED led) {
        return Commands.sequence(
                Commands.either(
                        Commands.sequence(
                                new ParallelCommandGroup(
                                        TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerAllowPosition),
                                        LEDFactory.setAdvancedStandardBlinkingCommand(led, LEDConstants.ColorValue.ORANGE, LEDConstants.ColorValue.VIOLET, () -> 50)
                                ),
                                shootOneBallSensoredCommand(intake, transfer),
                                Commands.either(
                                        Commands.sequence(
                                                indexBallsSensoredCommand(intake, transfer),
                                                shootOneBallSensoredCommand(intake, transfer)
                                        ),
                                        Commands.none(),
                                        () -> transfer.getCurrentNumberOfBalls() > 2  // Check at sequence start
                                ),
                                Commands.either(
                                        Commands.sequence(
                                                indexBallsSensoredCommand(intake, transfer),
                                                shootOneBallSensoredCommand(intake, transfer)
                                        ),
                                        Commands.none(),
                                        () -> transfer.getCurrentNumberOfBalls() > 1  // Check at sequence start
                                )
                        ),
                        Commands.none(),
                        () -> transfer.getCurrentNumberOfBalls() > 0
                ),
                TransferFactory.engageBlocker(transfer, () -> TransferConstants.blockerIdlePosition)
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

    private static Command shootOneBallSensoredCommand(Intake intake, Transfer transfer) {
        return Commands.sequence(
                IntakeFactory.openLoopSetpointCommand(intake, () -> 1.0),
                Commands.waitMillis(100),
                Commands.waitUntil(transfer::isThirdBeamBroken).withTimeout(750),
                new ConditionalCommand(
                        TransferFactory.runKickerCycle(transfer),
                        Commands.none(),
                        transfer::doesTransferContainSingleBall
                ),
                Commands.waitUntil(() -> !transfer.isThirdBeamBroken()).withTimeout(500),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.0)
        );
    }

    private static Command indexBallsSensoredCommand(Intake intake, Transfer transfer) {
        return Commands.sequence(
                Commands.waitMillis(250),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.3),
                Commands.waitUntil(transfer::isThirdBeamBroken).withTimeout(2000),
                IntakeFactory.openLoopSetpointCommand(intake, () -> 0.0),
                Commands.waitMillis(100)
        );
    }
}
