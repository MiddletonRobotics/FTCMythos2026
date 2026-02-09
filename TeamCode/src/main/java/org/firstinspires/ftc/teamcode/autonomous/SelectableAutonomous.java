package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.RunCommand;
import org.firstinspires.ftc.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.library.command.WaitUntilCommand;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.library.utilities.Timing;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commands.AimTowardShootingRegion;
import org.firstinspires.ftc.teamcode.commands.MaintainShooterNumericals;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.SavedConfiguration;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "SelectableAutonomous", group = "Auto", preselectTeleOp = "RobotController")
public class SelectableAutonomous extends CommandOpMode {
    public enum AutoSelectState {
        CONFIRM_CONTROLLERS,
        SELECTING,
        LOCKED,
        SCHEDULED
    }

    private AutoSelectState state = AutoSelectState.CONFIRM_CONTROLLERS;
    private final Timing.Stopwatch loopTimer = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
    private final Timing.Rate telemetryRate = new Timing.Rate(100);

    private Location selectedLocation = Location.CLOSE;
    private Auto selectedAuto = Auto.IDLE;
    private GlobalConstants.AllianceColor selectedAlliance = GlobalConstants.AllianceColor.BLUE;

    private int selectionIndex = 0;
    private static final int TOTAL_CATEGORIES = 3;

    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;
    private Vision vision;
    private Lift lift;
    private LED led;

    private AutoChooser autoChooser;

    private boolean lastTriangle;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    private boolean confirmedDriverController = false;
    private boolean confirmOperatorController = false;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @IgnoreConfigurable
    static LightsManager lightsManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        lightsManager = PanelsLights.INSTANCE.getLights();

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        transfer = new Transfer(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        vision = new Vision(hardwareMap, telemetryManager);
        led = new LED(hardwareMap, telemetryManager, lightsManager);
        lift = new Lift(hardwareMap, telemetryManager);

        autoChooser = new AutoChooser(drivetrain, intake, transfer, turret, shooter, vision, led);

        telemetryManager.addLine("Press DPAD-LEFT on BOTH controllers to start selection.");
        telemetryManager.update(telemetry);

        transfer.onInitialization(true, true);
        led.setSolid(LEDConstants.ColorValue.YELLOW);
    }

    @Override
    public void initialize_loop() {
        boolean triangle = gamepad1.triangle;

        switch (state) {
            case CONFIRM_CONTROLLERS:
                handleControllerConfirmation();
                break;
            case SELECTING:
                handleSelection(triangle);
                break;
            case LOCKED:
                showReady();
                break;
            case SCHEDULED:
                telemetryManager.addLine("Selections Locked! Please validate setup. Starting Auto Initialization...");
                telemetryManager.addData("Location", selectedLocation);
                telemetryManager.addData("Auto Type", selectedAuto);
                telemetryManager.addData("Alliance", selectedAlliance);
                telemetryManager.update(telemetry);
                break;
        }

        telemetryManager.update(telemetry);

        lastTriangle = triangle;
        lift.onInitialization();
    }

    private void handleControllerConfirmation() {
        if (confirmedDriverController && confirmOperatorController) {
            state = AutoSelectState.SELECTING;
        }

        if(gamepad1.dpad_left && !confirmedDriverController) {
            confirmedDriverController = true;
        } else if(gamepad2.dpad_left && !confirmOperatorController) {
            confirmOperatorController = true;
        }
    }

    private void handleSelection(boolean triangle) {
        readInputs();
        drawUI();

        led.setSolid(selectedAlliance == GlobalConstants.AllianceColor.BLUE ? LEDConstants.ColorValue.BLUE : LEDConstants.ColorValue.RED);

        if (triangle && !lastTriangle) {
            state = AutoSelectState.LOCKED;
            onLocked();
        }
    }

    private void onLocked() {
        led.setDefaultComplexBlink(
            selectedAlliance == GlobalConstants.AllianceColor.BLUE ? LEDConstants.ColorValue.BLUE : LEDConstants.ColorValue.RED,
            LEDConstants.ColorValue.ORANGE,
            230
        );

        scheduleRoutine();
        state = AutoSelectState.SCHEDULED;
    }

    private void scheduleRoutine() {
        GlobalConstants.opModeType = GlobalConstants.OpModeType.AUTONOMOUS;
        GlobalConstants.allianceColor = selectedAlliance;

        Pair<Pose, Pair<Pose, Command>> routine = autoChooser.getDesiredProgram(selectedLocation, selectedAuto);

        if (routine == null) {
            telemetryManager.addLine("ERROR: No auto routine found!");
            telemetryManager.update(telemetry);
            return;
        }

        //TODO: Change the turret position to always track the goal
        schedule(
            new RunCommand(drivetrain::update),
            new SequentialCommandGroup(
                new WaitUntilCommand(this::opModeIsActive),
                routine.getSecond().getSecond()
            )
        );

        turret.setDefaultCommand(new AimTowardShootingRegion(
                turret,
                drivetrain::getPose,
                GlobalConstants::getCurrentAllianceColor,
                turret::isTurretAutoTrackingEnabled
        ));

        shooter.setDefaultCommand(new MaintainShooterNumericals(
                shooter,
                () -> shooter.calculateFlywheelSpeeds(drivetrain.getDistanceToPose3D(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? GlobalConstants.kBlueGoalPose : GlobalConstants.kRedGoalPose, 38, 12)),
                () -> shooter.calculateHoodPosition(drivetrain.getDistanceToPose3D(GlobalConstants.getCurrentAllianceColor() == GlobalConstants.AllianceColor.BLUE ? GlobalConstants.kBlueGoalPose : GlobalConstants.kRedGoalPose, 38, 12)),
                () -> true
        ));

        drivetrain.setStartingPose(routine.getFirst());
        transfer.onInitialization(true, true);
        drivetrain.update();

        SavedConfiguration.selectedLocation = selectedLocation;
        SavedConfiguration.selectedAuto = selectedAuto;
        SavedConfiguration.selectedAlliance = selectedAlliance;
        SavedConfiguration.pathEndPose = routine.getSecond().getFirst();
    }

    private void drawUI() {
        telemetryManager.addLine("=== Autonomous Selection ===");

        telemetryManager.addLine((selectionIndex == 0 ? "> " : " ") + "Starting Location: " + selectedLocation);
        telemetryManager.addLine((selectionIndex == 1 ? "> " : " ") + "Auto Type: " + selectedAuto);
        telemetryManager.addLine((selectionIndex == 2 ? "> " : " ") + "Alliance Color: " + selectedAlliance);

        telemetryManager.addLine("");
        telemetryManager.addLine("DPAD ←/→ change value");
        telemetryManager.addLine("DPAD ↑/↓ change category");
        telemetryManager.addLine("Triangle to confirm");
    }

    private void readInputs() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !lastUp) selectionIndex = (selectionIndex - 1 + TOTAL_CATEGORIES) % TOTAL_CATEGORIES;
        if (down && !lastDown) selectionIndex = (selectionIndex + 1) % TOTAL_CATEGORIES;

        switch (selectionIndex) {
            case 0:
                if (right && !lastRight) selectedLocation = cycleRight(selectedLocation, Location.values());
                if (left && !lastLeft) selectedLocation = cycleLeft(selectedLocation, Location.values());
                break;
            case 1:
                if (right && !lastRight) selectedAuto = cycleRight(selectedAuto, Auto.values());
                if (left && !lastLeft) selectedAuto = cycleLeft(selectedAuto, Auto.values());
                break;
            case 2:
                if (right && !lastRight) selectedAlliance = cycleRight(selectedAlliance, GlobalConstants.AllianceColor.values());
                if (left && !lastLeft) selectedAlliance = cycleLeft(selectedAlliance, GlobalConstants.AllianceColor.values());
                break;
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }

    private <T> T cycleRight(T current, T[] values) {
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) return values[(i + 1) % values.length];
        }

        return current;
    }


    private <T> T cycleLeft(T current, T[] values) {
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) return values[(i - 1 + values.length) % values.length];
        }

        return current;
    }

    // TODO: Migrate the logs to automatically post with the subsystem
    private void showReady() {
        telemetryManager.addLine("Please validate the current setup. If the setup is incorrect, please stop and restart the OpMode.");
        telemetryManager.addLine("");

        telemetryManager.addData(DrivetrainConstants.kSubsystemName + " Pose X", drivetrain.getPose().getX());
        telemetryManager.addData(DrivetrainConstants.kSubsystemName + " Pose Y", drivetrain.getPose().getY());
        telemetryManager.addData(DrivetrainConstants.kSubsystemName + " Pose θ", drivetrain.getPose().getRotation().getRadians());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        long loopMs = loopTimer.deltaTime();
        if (telemetryRate.atTime()) {
            telemetry.addData("Loop ms", loopMs);
            telemetry.addData("Hz", 1000.0 / loopMs);
        }

        SavedConfiguration.finalDrivetrainPose = drivetrain.getPose().getAsPedroPose();
        SavedConfiguration.finalDrivetrainVelocity = drivetrain.getVelocity();
        SavedConfiguration.savedTurretPosition = turret.getCurrentPosition();
    }

    @Override
    public void end() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        state = AutoSelectState.CONFIRM_CONTROLLERS;
        lastTriangle = false;

        led.setSolid(LEDConstants.ColorValue.OFF);
    }
}