package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.library.command.type.InstantCommand;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;

public class Superstructure extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Turret turret;
    private final Shooter shooter;
    private final TelemetryManager telemetryManager;

    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        INTAKING_FROM_GROUND,
        CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_ARTIFACT_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_ARTIFACT_AUTO,
        INTAKING_FROM_GROUND,
        CLIMB
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;

    public Superstructure(Drivetrain drivetrain, Intake intake, Turret turret, Shooter shooter, TelemetryManager telemetryManager) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.turret = turret;
        this.shooter = shooter;
        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
        telemetryManager.addData("Superstructure/WantedSuperState", wantedSuperState);
        telemetryManager.addData("Superstructure/CurrentSuperState", currentSuperState);
        telemetryManager.addData("Superstructure/PreviousSuperState", previousSuperState);

        currentSuperState = handleStateTransitions();
        applyStates();
    }

    private CurrentSuperState handleStateTransitions() {
        previousSuperState = currentSuperState;
        switch (wantedSuperState) {
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case HOME:
                currentSuperState = CurrentSuperState.HOME;
                break;
            case DEFAULT_STATE:
                if (intake.currentlyHoldingBalls()) {
                    if (GlobalConstants.opModeType.equals(GlobalConstants.OpModeType.AUTONOMOUS)) {
                        currentSuperState = CurrentSuperState.HOLDING_ARTIFACT_AUTO;
                    } else {
                        currentSuperState = CurrentSuperState.HOLDING_ARTIFACT_TELEOP;
                    }
                } else {
                    if (GlobalConstants.opModeType.equals(GlobalConstants.OpModeType.AUTONOMOUS)) {
                        currentSuperState = CurrentSuperState.NO_PIECE_AUTO;
                    } else {
                        currentSuperState = CurrentSuperState.NO_PIECE_TELEOP;
                    }
                }
                break;
            case INTAKING_FROM_GROUND:
                currentSuperState = CurrentSuperState.INTAKING_FROM_GROUND;
        }

        return currentSuperState;
    }

    private void applyStates() {
        switch (currentSuperState) {
            case HOME:
                home();
            case INTAKING_FROM_GROUND:
                intakeArtifactFromGround();
        }
    }

    public void home() {
        drivetrain.setWantedState(Drivetrain.WantedState.TELEOP_DRIVE);
    }

    public void intakeArtifactFromGround() {
        drivetrain.setWantedState(Drivetrain.WantedState.TELEOP_DRIVE);
        intake.setIntakeTargetRPM(400);
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        return setStateCommand(superState, false);
    }

    public Command setStateCommand(WantedSuperState superState, boolean runIfClimberDeployed) {
        Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
        if (!runIfClimberDeployed) {
            commandToReturn = commandToReturn.onlyIf(() -> currentSuperState != CurrentSuperState.CLIMB);
        }
        return commandToReturn;
    }

    public Command configureButtonBinding(WantedSuperState hasBallsCondition, WantedSuperState noBallsCondition) {
        return Commands.either(
                setStateCommand(hasBallsCondition),
                setStateCommand(noBallsCondition),
                intake::currentlyHoldingBalls
        );
    }
}
