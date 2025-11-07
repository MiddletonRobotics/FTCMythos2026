package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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

        }
    }

    private void applyStates() {

    }
}
