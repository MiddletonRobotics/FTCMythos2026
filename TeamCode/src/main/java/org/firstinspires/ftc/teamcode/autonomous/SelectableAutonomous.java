package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
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
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name="SelectableAutonomous", group="Auto", preselectTeleOp="RobotController")
public class SelectableAutonomous extends CommandOpMode {
    private Location selectedLocation = Location.CLOSE;
    private Auto selectedAuto = Auto.IDLE;
    private GlobalConstants.AllianceColor selectedAlliance = GlobalConstants.AllianceColor.BLUE;

    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;
    private Vision vision;
    private LED led;

    private AutoChooser autoChooser;

    private int selectionIndex = 0;
    private static final int totalCategories = 3;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastTriangle;
    private boolean isLockedIn = false;
    private boolean hasBeenScheduled = false;

    private boolean confirmedControllers = false;
    private boolean confirmedDriverController = false;
    private boolean confirmOperatorController = false;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        transfer = new Transfer(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        vision = new Vision(hardwareMap, telemetryManager);
        led = new LED(hardwareMap, telemetryManager);

        autoChooser = new AutoChooser(drivetrain, intake, transfer, turret, shooter, vision, led);

        telemetryManager.addLine("Enable both gamepad by pressing DPAD-LEFT.");
        telemetryManager.update(telemetry);

        led.enableBlinking(100, LEDConstants.ColorValue.YELLOW);
    }

    @Override
    public void initialize_loop() {
        boolean triangle = gamepad1.triangle;

        if(confirmedControllers) {
            if (triangle && !lastTriangle && !hasBeenScheduled) {
                isLockedIn = true;
                hasBeenScheduled = true;
                scheduleRoutine();
            } else if(!isLockedIn) {
                readInputs();
                drawUI();
            } else {
                showReady();

                telemetryManager.addLine("");
                telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose X", drivetrain.getPose().getX());
                telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose Y", drivetrain.getPose().getY());
                telemetryManager.addData(DrivetrainConstants.kSubsystemName + "Pose θ", drivetrain.getPose().getRotation().getDegrees());
                telemetryManager.addData(TurretConstants.kSubsystemName + "Position", turret.getCurrentPosition());
                telemetryManager.update(telemetry);
            }

            if (!isLockedIn) {
                if (selectedAlliance == GlobalConstants.AllianceColor.RED) {
                    led.enableSolidColor(LEDConstants.ColorValue.RED);
                } else {
                    led.enableSolidColor(LEDConstants.ColorValue.BLUE);
                }
            }
        } else {
            if(gamepad1.dpad_left) {
                confirmedDriverController = true;
            } else if(gamepad2.dpad_left) {
                confirmOperatorController = true;
            } else if(confirmedDriverController && confirmOperatorController) {
                confirmedControllers = true;
            }
        }

        if(turret.isHomingTriggered()) {
            turret.resetPosition();
        }

        lastTriangle = triangle;
        led.update();
    }

    /** Handle gamepad navigation */
    private void readInputs() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if(up && !lastUp) {
            selectionIndex = (selectionIndex - 1 + totalCategories) % totalCategories;
        }

        if(down && !lastDown) {
            selectionIndex = (selectionIndex + 1) % totalCategories;
        }

        switch (selectionIndex) {
            case 0: // Location
                if(right && !lastRight)
                    selectedLocation = cycleRight(selectedLocation, Location.values());
                if(left && !lastLeft)
                    selectedLocation = cycleLeft(selectedLocation, Location.values());
                break;

            case 1: // Auto Type
                if(right && !lastRight)
                    selectedAuto = cycleRight(selectedAuto, Auto.values());
                if(left && !lastLeft)
                    selectedAuto = cycleLeft(selectedAuto, Auto.values());
                break;

            case 2: // Alliance
                if(right && !lastRight)
                    selectedAlliance = cycleRight(selectedAlliance, GlobalConstants.AllianceColor.values());
                if(left && !lastLeft)
                    selectedAlliance = cycleLeft(selectedAlliance, GlobalConstants.AllianceColor.values());
                break;
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }

    /** Draw the selection menu on Driver Hub */
    private void drawUI() {
        telemetryManager.addLine("=== Autonomous Selection ===");

        telemetryManager.addLine((selectionIndex == 0 ? "> " : "  ") + "Starting Location: " + selectedLocation);
        telemetryManager.addLine((selectionIndex == 1 ? "> " : "  ") + "Auto Type:        " + selectedAuto);
        telemetryManager.addLine((selectionIndex == 2 ? "> " : "  ") + "Alliance Color:   " + selectedAlliance);

        telemetryManager.addLine("");
        telemetryManager.addLine("DPAD ←/→ to change value");
        telemetryManager.addLine("DPAD ↑/↓ to change category");
        telemetryManager.addLine("Triangle to confirm");

        telemetryManager.update(telemetry);
    }

    public void scheduleRoutine() {
        GlobalConstants.opModeType = GlobalConstants.OpModeType.AUTONOMOUS;
        GlobalConstants.allianceColor = selectedAlliance;

        Pair<Pose, Command> routine = autoChooser.getDesiredProgram(selectedLocation, selectedAuto);

        if(routine == null) {
            telemetryManager.addLine("ERROR: No auto routine found for selection!");
            telemetryManager.update(telemetry);
        }

        assert routine != null;

        schedule(
            new RunCommand(drivetrain::update),
            new RunCommand(led::update),
            new RunCommand(() -> telemetryManager.update(telemetry)),
            new RunCommand(() -> turret.setPosition(turret.computeAngle(drivetrain.getPose(), DrivetrainConstants.decideToFlipPose(selectedAlliance, TurretConstants.aimPoseBlue), 0, 0))),
            new SequentialCommandGroup(
                new WaitUntilCommand(this::opModeIsActive),
                routine.getSecond()
            )
        );

        drivetrain.setStartingPose(routine.getFirst());
        drivetrain.update();

        telemetryManager.addLine("Selections Locked! Please validate setup. Starting Auto Initialization...");
        telemetryManager.addData("Location", selectedLocation);
        telemetryManager.addData("Auto Type", selectedAuto);
        telemetryManager.addData("Alliance", selectedAlliance);
    }

    public void showReady() {
        if (selectedAlliance == GlobalConstants.AllianceColor.RED) {
            led.enableBlinking(230, LEDConstants.ColorValue.RED);
        } else {
            led.enableBlinking(230, LEDConstants.ColorValue.BLUE);
        }
    }

    /** Helper to cycle right through an enum */
    private <T> T cycleRight(T current, T[] values) {
        int index = -1;
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) index = i;
        }

        return values[(index + 1) % values.length];
    }

    /** Helper to cycle left through an enum */
    private <T> T cycleLeft(T current, T[] values) {
        int index = -1;
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) index = i;
        }

        return values[(index - 1 + values.length) % values.length];
    }

    @Override
    public void end() {
        hasBeenScheduled = false;
        isLockedIn = false;
        lastTriangle = false;

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}