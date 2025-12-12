package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class AutoChooser {
    private final List<AutoRoutine> routines = new ArrayList<>();
    private final AutoFactory autoFactory;

    public AutoChooser(Drivetrain drivetrain, Intake intake, Transfer transfer, Shooter shooter) {
        autoFactory = new AutoFactory(GlobalConstants.allianceColor, drivetrain, intake, transfer, shooter);

        routines.add(new AutoRoutine(Location.CLOSE, Auto.IDLE, () -> autoFactory.initializeIdle(Location.CLOSE.getPose())));
        routines.add(new AutoRoutine(Location.FAR, Auto.IDLE, () -> autoFactory.initializeIdle(Location.FAR.getPose())));
        routines.add(new AutoRoutine(Location.FAR, Auto.SIX_BALL, () -> autoFactory.initializeSixBallFar(Location.FAR.getPose())));
    }

    public Pair<Pose, Command> getDesiredProgram(Location loc, Auto type) {
        for (AutoRoutine r : routines) {
            if (r.location == loc && r.autoType == type) {
                return r.routine.get();
            }
        }

        return null;
    }
}
