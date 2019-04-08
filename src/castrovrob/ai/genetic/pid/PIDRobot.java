package castrovrob.ai.genetic.pid;

import castrovrob.dyn4j.framework.SimulationBody;
import org.dyn4j.dynamics.World;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

public class PIDRobot extends SimulationBody implements Runnable{
    private boolean isRunning = true;

    private SimulationBody circle;
    private SimulationBody stick;
    private RevoluteJoint circleToStick;

    private double previousError = 0.0;

    private final int PROPORTIONAL_CONST = 5;
    private final int INTEGRAL_CONST = 1;
    private final int DERIVATIVE_CONST = 5;

    public PIDRobot(World mWorld){
        circle = new SimulationBody();
        circle.addFixture(Geometry.createCircle(1.0),1.0,1.0,1.0);
        circle.setMass(MassType.NORMAL);
        circle.translate(0.0,0.0);
        circle.setAngularDamping(10.0);

        stick = new SimulationBody();
        stick.addFixture(Geometry.createRectangle(0.1, 3.75));
        stick.setMass(MassType.NORMAL);
        stick.translate(0.0, 1.875);

        circleToStick = new RevoluteJoint(circle, stick, new Vector2(0.0, 0.0));
        circleToStick.setLimitEnabled(false);
        circleToStick.setLimits(Math.toRadians(0.0), Math.toRadians(0.0));
        circleToStick.setReferenceAngle(Math.toRadians(0.0));
        circleToStick.setMotorEnabled(true);
        circleToStick.setMotorSpeed(Math.toRadians(0.0));
        circleToStick.setMaximumMotorTorque(0.0);

        circleToStick.setCollisionAllowed(true);

        mWorld.addBody(circle);
        mWorld.addBody(stick);
        mWorld.addJoint(circleToStick);
    }

    public void PIDControl(){
        double offset = 0.0;
        double normalValue = Math.toDegrees(stick.getChangeInOrientation());

        double proportionalError = 0.0;
        proportionalError = normalValue - offset;

        double integralError = 0.0;
        integralError = ((integralError + proportionalError) * 2 / 3);

        double derivativeError = 0.0;
        derivativeError = proportionalError - this.previousError;

        double pidValue = PROPORTIONAL_CONST * proportionalError + INTEGRAL_CONST * integralError + DERIVATIVE_CONST * derivativeError;
        this.previousError = proportionalError;

        //this.circleToStick.setMaximumMotorTorque(pidValue);
        //this.circleToStick.setMotorSpeed(pidValue);

        //this.circle.applyTorque(pidValue);
        this.circle.setAngularVelocity(pidValue);
    }

    public void disposeRobot(){
        this.isRunning = false;
    }

    @Override
    public void run(){
        try {
            while (isRunning) {
                this.PIDControl();
                Thread.sleep(100);
            }
        }catch(InterruptedException interruptedException){
            interruptedException.printStackTrace();
        }
    }

}
