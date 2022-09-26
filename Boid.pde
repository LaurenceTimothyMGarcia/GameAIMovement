/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   
   // "private" variables
   float current_accel = 1;
   float current_rotational_accel;
   float slowdown_accel = -2;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt)
   {
     if (target != null)
     {  
        // TODO: Implement seek here
        // set base rotational speed
        current_rotational_accel = rotational_acceleration;
        // get absolute direction of target position
        float targetRotation = atan2(target.y - kinematic.getPosition().y, target.x - kinematic.getPosition().x);
        // get direction of target position relative to Boid's front; now (hopefully) leftwards movement is a positive angle and rightwards is negative
        float directionRotation = normalize_angle_left_right(targetRotation - kinematic.getHeading());
        // orient direction of acceleration properly (I.E. if moving leftwards set the acceleration to be towards left
        if (directionRotation < 0)
        {
          current_rotational_accel *= -1;
        }
        
        print("target: " + targetRotation + "\n");
        print("direction: " + directionRotation + "\n");
        print("acceleration: " + acceleration + "\n");
        
        // now the boid's angular acceleration is towards the target angle; when the target angle is close, the boid should instead start decelerating
        if (target.y - kinematic.getPosition().y < 10 && target.x - kinematic.getPosition().x < 10)
        {
          acceleration = slowdown_accel;
        }
        else //Makes this go back and forth
        {
          acceleration = current_accel;
        }
        
        // please ignore the magic numbers (that value is 20 degrees in radians); also this conditional doesn't work LMAO
        if (abs(directionRotation) < 0.349066) {
          current_rotational_accel *= -1;
        }
        
        kinematic.increaseSpeed(acceleration, current_rotational_accel);
     }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   }
   
   void draw()
   {
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   void seek(PVector target)
   {
      this.target = target;
      
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      this.target = waypoints.get(0);
      
   }
}
