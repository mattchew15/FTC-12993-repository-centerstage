package org.firstinspires.ftc.teamcode.system.paths.P2P;

import java.util.ArrayList;

public class TrajectoryBuilder {

    private final ArrayList<TrajectorySegment> segments = new ArrayList<>();

    public TrajectoryBuilder(TrajectorySegment firstSegment){
        segments.add(firstSegment);
    }

    public TrajectoryBuilder addSegment(TrajectorySegment segment){
        segments.add(segment);
        return this;
    }

    public Trajectory build(){
        return new Trajectory(segments);
    }
}