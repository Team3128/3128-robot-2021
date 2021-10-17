import static org.junit.Assert.assertEquals;

import com.revrobotics.CANEncoder;

import org.junit.Before;
import org.junit.Test;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.grogu.subsystems.Hopper;
import org.team3128.grogu.subsystems.Constants;
import org.team3128.grogu.subsystems.FalconDrive;
import org.team3128.grogu.subsystems.Shooter;
import org.team3128.grogu.subsystems.Sidekick;

import edu.wpi.first.hal.HAL;

public class ShooterTest {

    public FalconDrive drive;
    public Hopper hopper;
    public Shooter shooter;
    public Sidekick sidekick;
    public static LazyTalonFX LEFT_SHOOTER;
    public static LazyTalonFX RIGHT_SHOOTER;
    public static CANEncoder SHOOTER_ENCODER;

    // private StateTracker stateTracker = StateTracker.getInstance()

    @Before // this method will run before each test
    public void setup() {
      assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
      //LEFT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
      //RIGHT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);

      shooter = Shooter.getInstance();
    }

    @Test
    public void isReadyTest() {
        assertEquals(true, shooter.isReady());
    }
}
