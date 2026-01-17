package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class AutoChooser {
    private final List<AutoRoutine> routines = new ArrayList<>();
    private final AutoFactory autoFactory;

    public AutoChooser(Drivetrain drivetrain, Intake intake, Transfer transfer, Turret turret, Shooter shooter, Vision vision, LED led) {
        autoFactory = new AutoFactory(drivetrain, intake, transfer, turret, shooter, vision, led);

        routines.add(new AutoRoutine(Location.CLOSE, Auto.IDLE, () -> autoFactory.initializeIdle(GlobalConstants.allianceColor, Location.CLOSE.getPose())));
        routines.add(new AutoRoutine(Location.FAR, Auto.IDLE, () -> autoFactory.initializeIdle(GlobalConstants.allianceColor, Location.FAR.getPose())));
        routines.add(new AutoRoutine(Location.CLOSE, Auto.LEAVE, () -> autoFactory.initializeCloseLeave(GlobalConstants.allianceColor, Location.CLOSE.getPose())));
        routines.add(new AutoRoutine(Location.FAR, Auto.LEAVE, () -> autoFactory.initializeFarLeave(GlobalConstants.allianceColor, Location.FAR.getPose())));
        routines.add(new AutoRoutine(Location.FAR, Auto.SIX_BALL, () -> autoFactory.initializeFarSixBall(GlobalConstants.allianceColor, Location.FAR.getPose())));
        routines.add(new AutoRoutine(Location.CLOSE, Auto.NINE_BALL, () -> autoFactory.initializeCloseNineBall(GlobalConstants.allianceColor, Location.CLOSE.getPose())));
        routines.add(new AutoRoutine(Location.CLOSE, Auto.NINE_BALL_PICKUP, () -> autoFactory.initializeCloseNineBall(GlobalConstants.allianceColor, Location.CLOSE.getPose())));
    }

    public Pair<Pose, Pair<Pose, Command>> getDesiredProgram(Location location, Auto type) {
        for (AutoRoutine r : routines) {
            if (r.location == location && r.autoType == type) {
                return r.routine.get();
            }
        }

        return null;
    }
}
