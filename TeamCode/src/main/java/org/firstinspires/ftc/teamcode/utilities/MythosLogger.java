package org.firstinspires.ftc.teamcode.utilities;


import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;


public class MythosLogger {
    String logFileName;
    private FileHandler logFileHandler;

    public ElapsedTime timer;

    private Logger logger;

    public GamepadEx driver;

    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;
    private LED led;
    private Vision vision;

    public MythosLogger(String fileName)  {
        logFileName = fileName;
        timer = new ElapsedTime();

    }

    public boolean init() {
        if (logger != null) {
            return true;
        }

        try {
            String fullLogPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logFileName;
            logFileHandler = new FileHandler(fullLogPath, false);

            //Simple Formatter
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);

            logger = Logger.getLogger(MythosLogger.class.getName());
            logger.addHandler(logFileHandler);
            logger.setLevel(Level.INFO);

            logger = Logger.getLogger(MythosLogger.class.getName());
            // Add the FileHandler to the logger.
            logger.addHandler(logFileHandler);

            logger.setLevel(Level.INFO);

        } catch (SecurityException | IOException e) {
            e.printStackTrace();
            return false;
        }

        return true;
    }

    public void deinit() {
        logger.info("Deinitialize the logger");
        logger = null;
        logFileHandler.close();
        logFileHandler = null;
    }

    public void log() {
        String str = "";

        // log format
        // [Name:Type:Value];
        // E.g.
        // Timestamp:int64:value;Pose:Tuple:{a,b,c};
        str += "Loop Time:Double:" + timer.milliseconds();

        str += ";Heading:Double:" + drivetrain.getPose().getHeading();
        str += ";Robot Pose:Pose2D;" + drivetrain.getPose();
        str += ";Turret Position:Double;" + turret.getCurrentPosition();
        str += ";Flywheel Velocity:Double:" + shooter.getVelocity();

        logger.info(str);
    }


}