
function next = car(speed, steer, next, stlim, vmax, L, dt)        

   speed = min(vmax, max(-vmax, speed));   %limites
   steer = max(-stlim, min(stlim, steer)); %limites
   
   %modelo cinemático car-like
   next(1) = next(1) + speed*dt*cos(next(3));
   next(2) = next(2) + speed*dt*sin(next(3));
   next(3) = next(3) + speed*dt/L*tan(steer);
   
end
  