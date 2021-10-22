import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.junit.Before;
import org.junit.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.grogu.subsystems.Shooter;

public class ShooterTest {

    @Mock
    public static LazyTalonFX LEFT_SHOOTER;

    @Mock
    public static LazyTalonFX RIGHT_SHOOTER;

    @InjectMocks
    public Shooter shooter;

    // private StateTracker stateTracker = StateTracker.getInstance()

    @Before // this method will run before each test
    public void setup() {
      //assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
      //LEFT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
      //RIGHT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
      MockitoAnnotations.initMocks(this);
      shooter = Mockito.spy(Shooter.getInstance());
    }

    @Test
    public void isReadyTest() {
        assertEquals(true, shooter.isReady());
    }

    @Test
    public void testUseOutputRandom() {
      double batteryVoltage = Math.random()*13;
      double setpoint = Math.random()*13;
      double feedForward = (0.00147 * setpoint)  - 0.2;
      double percentage = Math.random()*100;
      Mockito.doReturn(feedForward).when(shooter).shooterFeedForward(setpoint);
      double pidOutput = (percentage*batteryVoltage)/(100) - feedForward;

      shooter.useOutput(pidOutput, setpoint);
      Mockito.verify(LEFT_SHOOTER).set(ControlMode.PercentOutput, percentage);
      Mockito.verify(RIGHT_SHOOTER).set(ControlMode.PercentOutput, -percentage);
    }

    @Test
    public void testUseOutputZero() {
      double batteryVoltage = Math.random()*13;
      double setpoint = Math.random()*13;
      double feedForward = (0.00147 * setpoint)  - 0.2;
      double percentage = 0;
      Mockito.doReturn(feedForward).when(shooter).shooterFeedForward(setpoint);
      double pidOutput = (percentage*batteryVoltage)/(100) - feedForward;

      shooter.useOutput(pidOutput, setpoint);
      Mockito.verify(LEFT_SHOOTER).set(ControlMode.PercentOutput, percentage);
      Mockito.verify(RIGHT_SHOOTER).set(ControlMode.PercentOutput, -percentage);
    }

    @Test
    public void testUseOutputExtraneous() {
      double batteryVoltage = Math.random()*13;
      double setpoint = Math.random()*13;
      double feedForward = (0.00147 * setpoint)  - 0.2;
      double percentage = 105;
      Mockito.doReturn(feedForward).when(shooter).shooterFeedForward(setpoint);
      double pidOutput = (percentage*batteryVoltage)/(100) - feedForward;

      shooter.useOutput(pidOutput, setpoint);
      Mockito.verify(LEFT_SHOOTER).set(ControlMode.PercentOutput, percentage);
      Mockito.verify(RIGHT_SHOOTER).set(ControlMode.PercentOutput, -percentage);
    }
}
