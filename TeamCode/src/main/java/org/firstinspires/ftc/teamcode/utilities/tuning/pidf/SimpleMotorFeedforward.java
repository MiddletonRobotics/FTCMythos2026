package org.firstinspires.ftc.teamcode.utilities.tuning.pidf;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Disabled
@TeleOp(name="SimpleMotorFeedforward")
public class SimpleMotorFeedforward extends OpMode {
    private MotorEx motor;
    private MotorEx.Encoder encoder;

    private double power;
    private double saveSlot1;

    private boolean aButtonPreviousState = false;
    private boolean xButtonPreviousState, yButtonPreviousState = false;
    private boolean rightBumperButtonPreviousState = false;
    private boolean isFinishedTuning = false;
    private boolean startButtonPressed = false;

    private Telemetry telemetryA;

    @Override
    public void init() {
        motor = new MotorEx(hardwareMap, "turretMotor");
        encoder = new MotorEx(hardwareMap, "turretMotor").encoder;

        motor.setInverted(false);
        encoder.reset();
        power = 0.0;
    }

    @Override
    public void init_loop() {
        if(startButtonPressed) {
            telemetryA.addLine("Square (X): Increase power by 0.01");
            telemetryA.addLine("Triangle (Y): Decrease power by 0.01");
            telemetryA.addLine("Cross (A): Continue / output final results (needs save slots full)");
            telemetryA.addLine("Right Bumper: Save slot 1 (-1 represents no data saved)");
            telemetryA.update();
        } else {
            telemetryA.addLine("This will run the motor specified initially at 0 power.");
            telemetryA.addLine("Make sure the motor is able to rotate multiple times without damaging the system.");
            telemetryA.addLine("Increase power until the motor barely moves, then save it.");
            telemetryA.addLine("Press the start button to bring up the controls");
            telemetryA.update();

            if(gamepad1.start) {
                startButtonPressed = true;
            }
        }
    }

    @Override
    public void loop() {
        if (gamepad1.square && !xButtonPreviousState) {
            if (power == 1) {
                power -= 0;
            } else {
                power += 0.01;
            }
        } else if (gamepad1.triangle && !yButtonPreviousState) {
            if (power == 0) {
                power -= 0;
            } else {
                power -= 0.01;
            }
        }

        if(gamepad1.right_bumper && !rightBumperButtonPreviousState) {
            saveSlot1 = power;
        }

        if(gamepad1.a && !aButtonPreviousState || isFinishedTuning) {
            if(saveSlot1 != -1) {
                double kS = power;

                telemetryA.addData("Calculated kS Value: ", kS);
                telemetryA.update();

                isFinishedTuning = true;
            }
        } else {
            motor.set(power);

            telemetryA.addData("Motor Power: ", power);
            telemetryA.addData("Current Position: ", encoder.getPosition());
            telemetryA.addData("Current Velocity: ", encoder.getCorrectedVelocity());
            telemetryA.addLine("\n");
            telemetryA.addData("Save Slot 1: ", saveSlot1);
            telemetryA.update();
        }

        aButtonPreviousState = gamepad1.a;
        xButtonPreviousState = gamepad1.square;
        yButtonPreviousState = gamepad1.triangle;
        rightBumperButtonPreviousState = gamepad1.right_bumper;
    }
}
