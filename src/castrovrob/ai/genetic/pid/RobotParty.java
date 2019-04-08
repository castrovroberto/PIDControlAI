package castrovrob.ai.genetic.pid;

import castrovrob.dyn4j.framework.SimulationBody;
import castrovrob.dyn4j.framework.SimulationFrame;
import org.dyn4j.dynamics.World;
import org.dyn4j.geometry.Geometry;

import java.util.ArrayList;
import java.util.List;

public class RobotParty extends SimulationFrame{

    private List<PIDRobot> myRobots;

    protected void initializeWorld(){
        this.world.setGravity(World.EARTH_GRAVITY);
        this.createGround();
    }

    private RobotParty(){
        super("Genetic Algorithm PID", 40.0);
    }

    private void createGround(){
        SimulationBody ground = new SimulationBody();
        ground.addFixture(Geometry.createRectangle(15.0, 1.0),1.0, 1.0, 1.0);
        ground.translate(0.0, -3.5);

        this.world.addBody(ground);
    }

    private void createRobots(){
        this.myRobots = new ArrayList<>();
        this.myRobots.add(new PIDRobot(this.world));
        this.myRobots.get(0).run();
    }

    public static void main(String[] args){
        RobotParty simulation = new RobotParty();
        simulation.run();
        simulation.createRobots();
    }
}
