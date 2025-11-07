package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
        DEFAULT_STATE
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_ARTIFACT_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_ARTIFACT_AUTO,
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
        }

        return currentSuperState;
    }

    private void applyStates() {

    }
}
