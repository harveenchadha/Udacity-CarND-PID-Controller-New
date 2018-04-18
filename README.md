# PID Controller

In this Project the main challenge was to drive a car around the track using a Propotional-Integral-Derivative (PID) Controller. In this project the basic task is to optimize the parameters i.e. the PID coefficients in order to calculate the steering angle that ensures the car does not goes off-track.

# Results
[The final project video can be found here.](https://www.youtube.com/watch?v=J6QR-bMdd2g)

Components of PID-
..* P (Propotional Component) - This component tells that the car will steer propotional to cross track error(CTE). CTE is the deviation of car from the referrence trajectory(which is exactly in center of track in this project). If your car is to left of the center obviously you will want to steer it back to center by moving right. But in this process the car overshoots when it moves to right and then it tries to move left and so on.. the process continues. This component tells that exactly the steering value is how much propotional to CTE. **I estimated the P value to a low value to reduce the direct propotionality.**

..* D (Differential Component)- This component accounts for the rate of change of CTE. This means if the derivative is quickly changing, the car will move quickly towards center as in case of a curve where steering values are normally higher. If the value of this component is less then the oscillations are too much. **To reduce the oscillations I kept the value of this component to be a bit higher**

..* I (Integral Component)- This component accounts for the sum of all CTE's at a point. If the value of I component is too high, then the car oscillates too much and does not tends to pick up speed. **I choose the value of I to be really small.**

# Final Parameters

P - 0.15

I - 0.0005

D - 2.0

I found out this parameters by Manual Tuning. I tried out various combinations of coefficients. In the starting, I set every parameter to zero except Kd(Constant for Propotional Term). I got an oscilating behaviour. 
[Video for P component Only](./Videos/P_Controller.mp4)

Then I set the D component with Hit and Trial in such way that the oscillations are now controlled and the car steers correctly on curves. 
[Video for PD component Only](./Videos/P_D_Controller.mp4)

## Experiment with Throttle Values

I tried out increasing the throttle to 0.5. The results can be found below.
[Video for PID Controller with throttle 0.5](./Videos/P_I_D_Throttle_5.mp4)
