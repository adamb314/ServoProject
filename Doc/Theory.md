Theory
------

## PID vs Cascade control

The control system of a regular hobby servo looks something like this:

![loop1](readmeResources/ControlLoop1.png)

This control scheme is good at holding and moving fast between position, but not so good at following a smooth predefined motion. Which is what you want when you build a robot.

Industrial servo controllers use a cascade based control scheme instead, which looks something like this:

![loop2](readmeResources/ControlLoop2.png)

This is the control scheme used in this project.

The main benefit of this approach is that it also takes the velocity and torque into account. This allows for a much tighter motion tracking.

## Backlash control

When dealing with cheap gearboxes, backlash is always a problem. The hacked servos in this project uses two encoders to compensate for backlash. One AS5048a magnetic encoder or a potentiometer on the output shaft and one custom optical encoder inside the DC-motor. The benefit of having the extra encoder inside the motor is getting higher resolution and a main control loop which is backlash free. If one would only use the AS5048a for controlling, the backlash would enter the control loop as a time delay. This limits the control loop performance.

The backlash compensation is done by moving the main control loops reference-position so that the output encoder reaches the correct position.

## Calculating the control parameters
This project uses pole placement to calculate the control parameters for the cascade controller. To get the equations for the poles we need the find the closed transfer functions for the velocity and position control loops.

### Transfer function of open system to closed system
Given the discrete transfer function for the open system:

$$ G_0(z)=\frac{p(z)}{q(z)} $$

where $p(z)$ and $q(z)$ are pure polynomials, we can derive the closed systems transfer function as follows:

$$ Y = G_0(z) (R - Y) \iff Y = \frac{G_0(z)}{1 + G_0(z)} R = \frac{p(z)}{p(z) + q(z)}R $$

Which give us the closed system transfer function

$$ G_c(z) = \frac{p(z)}{p(z) + q(z)} $$

### Position control loop

The position control loops open transfer function consists of a proportional gain controller and an integrator (we approximate the inner velocity loop as a pure integrator):

$$ G_0(z)=G_{posControl}(z) G_{velLoop}(z)= (L_0) (\frac{dt}{z - 1}) $$

This gives us $p(z) = L_0 dt$ and $q(z) = z - 1$ which means that the pole polynomial is:

$$ p(z) + q(z) = z - 1 + L_0 dt $$

with the root:

$$ z - 1 + L_0 dt = 0 \iff z_{posPole} = 1 - L_0 dt $$

which means that the position control loop gain $L_0$ can be calculated with:

$$ L_0 =\frac{1 - z_{posPole}}{dt} $$

### Velocity control loop

The velocity control loops open transfer function consists of a PI controller and the motors transfer function (from control signal to velocity) which we approximate as a first order damped system:

$$ G_0(z)=G_{velControl}(z) G_{motor}(z)= (L_1 + L_2 \frac{1}{z - 1})(\frac{b}{z - a}) $$

This gives us $p(z) = b (L_1 (z - 1) + L_2)$ and $q(z) = (z - 1)(z - a)$ which means that the pole polynomial is:

$$ p(z) + q(z) = b (L_1 (z - 1) + L_2) + (z - 1)(z - a) = (a - b L_1 + b L_2) + (b L_1 -a - 1) z + z^2 $$

Substituting $L_1^{'} = b L_1$ and $L_2^{'} = b L_2$ gives:

$$ (a - L_1^{'} + L_2^{'}) + (L_1^{'} -a - 1) z + z^2 $$

with the roots:

$$ z_{velPole} = 1/2 (±\sqrt{{L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'}) - 4 L_2^{'}} + a - L_1^{'} + 1) $$

Given that we want the closed loop system to be stable and at least as fast as the open loop ($z_{openLoopPole} = a$) we can deduce the following:

$$ 0 < 1/2 (a - L_1^{'} + 1) \le a \le 1  \iff $$

$$ 0 < a - L_1^{'} + 1 \le 2a, \space 0 < a \le 1 \Rightarrow $$

$$ 0 \le 1 - a \le L_1^{'} < a + 1 \le 2 $$

For a damped system we need real roots $\Rightarrow$

$$ {L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'}) - 4 L_2^{'} \ge 0 $$

since $0 < a \le 1$ and $L_1^{'} > 0 \Rightarrow$

$$ {L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'}) - 4 L_2^{'} \ge 
 {L_1^{'}}^2 - 4 L_2^{'} \ge 0 $$


Substituting back to $L_1$ and $L_2$ results in:

$$ b (b L_1^2 - 4 b L_2) \ge 0 \iff b L_1^2 \ge 4 L_2 $$

If $L_1$ and $L_2$ are calculated so that

$$ {L_1^{'}}^2 - 4 L_2^{'} = 0 $$

then a increase of the load inertial on the motor ($b^{'} < b$) will result in an undamped system. To prevent this we can introduce an inertial margin $c_{im}$ such that $b^{'} = b / c_{im} \le b$. We want the damped system inequality to hit the limit if $b$ changes to $b^{'}$. This yields the following:

$$ (a - 1)^2 - 2 a L_1^{'} / c_{im} + (L_1^{'} / c_{im})^2 + 2 L_1^{'} / c_{im} - 4 L_2^{'} / c_{im} = 0 \iff $$

$$ L_2^{'} = 1/4 (c_{im} (a - 1)^2 - 2 a L_1^{'} + {L_1^{'}}^2 / c_{im} + 2 L_1^{'}) = $$

$$ = L_2^{'} = 1/4 ({L_1^{'}}^2 / c_{im} + (a - 1) (c_{im} (a - 1) - 2 L_1^{'})) $$

The slowest $z_{velPole}$ pole is the one closest to the value one, together with

$$ 0 < z_{velPole} \le a $$

we can now solve for $L_1^{'}$ with:

$$ z_{velPole} = 1/2 (\sqrt{{L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'}) - 4 L_2^{'}} + a - L_1^{'} + 1) $$

substituting in $L_2^{'}$

$$ z_{velPole} = 1/2 (\sqrt{{L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'}) - {L_1^{'}}^2 / c_{im} - (1 - a) (c_{im} (1 - a) + 2 L_1^{'})} + a - L_1^{'} + 1) = $$

$$ = z_{velPole} = 1/2 (\sqrt{(1 - 1 / c_{im}) {L_1^{'}}^2 + (1 - a) ((1 - a) + 2 L_1^{'} - c_{im} (1 - a) - 2 L_1^{'})} + a - L_1^{'} + 1) = $$

$$ = z_{velPole} = 1/2 (\sqrt{(1 - 1 / c_{im}) {L_1^{'}}^2 + (1 - a)^2 (1- c_{im})} + a - L_1^{'} + 1) \Rightarrow $$

$$ L_1^{'} = c_{im} (a - 2 z_{velPole} + 1) + 2 \sqrt{c_{im} (c_{im} - 1) (1 - z_{velPole}) (a - z_{velPole})} $$

Resulting in the final equations:

$$ L_1^{'} = c_{im} (a - 2 z_{velPole} + 1) + 2 \sqrt{c_{im} (c_{im} - 1) (1 - z_{velPole}) (a - z_{velPole})} $$

$$ L_2^{'} = 1/4 ({L_1^{'}}^2 / c_{im} + (a - 1) (c_{im} (a - 1) - 2 L_1^{'})) $$

$$ L_1 = L_1^{'} / b $$

$$ L_2 = L_2^{'} / b $$
