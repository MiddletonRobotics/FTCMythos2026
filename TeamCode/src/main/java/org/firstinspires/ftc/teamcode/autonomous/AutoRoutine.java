package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;

import java.util.function.Supplier;

public class AutoRoutine {
    public final Location location;
    public final Auto autoType;
    public final Supplier<Pair<Pose, Command>> routine;

    public AutoRoutine(final Location location, final Auto autoType, final Supplier<Pair<Pose, Command>> routine) {
        this.location = location;
        this.autoType = autoType;
        this.routine = routine;
    }
}
