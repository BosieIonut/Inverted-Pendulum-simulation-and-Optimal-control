# Inverted-Pendulum-simulation-and-Optimal-control
I simulated an inverted pendulum and controled it with an linear quadratic regulator
![ezgif com-video-to-gif-converter](https://github.com/BosieIonut/Inverted-Pendulum-simulation-and-Optimal-control/assets/33691449/94579b83-7fe4-4cf6-98ae-2c57f9a636be)

<br>
<br>

The simulation was made using SFML library in C++.

The diferential equations for the cart are:<br>
![image](https://github.com/BosieIonut/Inverted-Pendulum-simulation-and-Optimal-control/assets/33691449/0ff085ce-68f3-4da9-b4ed-9767878cebff)
<br>
![image](https://github.com/BosieIonut/Inverted-Pendulum-simulation-and-Optimal-control/assets/33691449/4e0d6469-eb22-41eb-b1db-7684edf11a9d)

For the simulation I solved those diferential equiations using EULER method.

![image](https://github.com/BosieIonut/Inverted-Pendulum-simulation-and-Optimal-control/assets/33691449/fe6fefda-8aa9-43e7-bcc6-acbadcfc9fd4)

![image](https://github.com/BosieIonut/Inverted-Pendulum-simulation-and-Optimal-control/assets/33691449/e24e1605-23ff-47d4-bf24-8caef063426d)

I liniarized them to obtain an liniar model in the up position.
The LQR weights were computed in matlab.


