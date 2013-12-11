
function next = car(speed, steer, next, stlim, vmax, L, dt)        

   speed = min(vmax, max(-vmax, speed));   %limites
   steer = max(-stlim, min(stlim, steer)); %limites
   
   %modelo cinemático car-like
   %see: Kinematic parameter calibration of a car-like mobile robot to improve odometry accuracy.
   dth = speed * dt/L * tan(steer);
   next(1) = next(1) + speed * dt*cos(next(3) + dth/2);
   next(2) = next(2) + speed * dt*sin(next(3) + dth/2);
   next(3) = next(3) + dth;
   
end
  