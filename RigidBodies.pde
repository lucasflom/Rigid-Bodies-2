//Rigid Body Dynamics
//CSCI 5611 Physical Simulation [Exercise]
// Stephen J. Guy <sjguy@umn.edu>

//To use: Click to apply a force to the box at the point where the mouse is clicked.
//        Box will turn red while the click force is being applied.  
//Challenge:
//  3. Add a second box to the scene! Break this into a few steps
//       3) Use the resources mentioned in resolveCollision() to update that function for object-object collisions
//     If you get this working, you'll have the basics of an impressive 2D physics engine
//  4. Other ideas: Allow other shapes besides boxes, add friction, add gravity

void setup(){
  size(600,400);
  s[0] = box;
  s[1] = new Square(new Vec2(400, 200), 25, 100);
}

//Set inital conditions
float w = 50;
float h = 200;
float box_bounce = 0.8; //Coef. of restitution

Vec2 middle = new Vec2(200,200);        //Current position of center of mass

float arrow_angle = 0.0;
Square box = new Square(middle, w, h);
int things = 2;
Square[] s = new Square[things];
//----------
// Physics Functions
void apply_force(Vec2 force, Vec2 applied_position, Square obj){
  obj.total_force.add(force);
  Vec2 displacement = applied_position.minus(obj.center);
  obj.total_torque = cross(displacement, force);
}

void update_physics(float dt){
  for (int i = 0; i < things; i++){
    Square obj = s[i];
    //Update center of mass
    obj.momentum.add(obj.total_force.times(dt));     //Linear Momentum = Force * time
    Vec2 box_vel = obj.momentum.times(1.0/obj.mass); //Velocity = Momentum / mass
    obj.center.add(box_vel.times(dt));           //Position += Vel * time
    //Update rotation
    obj.angular_momentum += obj.total_torque * dt;
    float ang_velocity = obj.angular_momentum/obj.rot_inertia;
    obj.angle += ang_velocity * dt;
    
    //Reset forces and torques
    obj.total_force = new Vec2(0,0); //Set forces to 0 after they've been applied
    obj.total_torque = 0.0; //Set torques to 0 after the forces have been applied
  }
  
}


class ColideInfo{
  public boolean hit = false;
  public Vec2 hitPoint = new Vec2(0,0);
  public Vec2 objectNormal =  new Vec2(0,0);
}


void updateCornerPositions(Square obj){
  Vec2 right = new Vec2(cos(obj.angle),sin(obj.angle)).times(obj.w/2);
  Vec2 up = new Vec2(-sin(obj.angle),cos(obj.angle)).times(-obj.h/2);
  obj.p1 = obj.center.plus(right).plus(up);
  obj.p2 = obj.center.plus(right).minus(up);
  obj.p3 = obj.center.minus(right).plus(up);
  obj.p4 = obj.center.minus(right).minus(up);
}

void boxCollisionTest(float dt){
  for (int i = 0; i < things; i++){
    Square obj = s[i];
    updateCornerPositions(obj);
    Vec2[] points = new Vec2[]{obj.p1, obj.p2, obj.p3, obj.p4};
    for (int j = 0; j < things; j++) {
      if (j != i){
        Square other = s[j];
        updateCornerPositions(other);
        for (int k = 0; k < 4; k++){
          Vec2 point = points[k];
          if (other.isInside(point)) { // Means there was a collision if the point is within any of the bounds of the box
            // Do collision things
            // The normal should be perpendicular to the side of other that was hit
            // Find the point velocity of object A
            Vec2 r = point.minus(obj.center);
            Vec2 r_perp = perpendicular(r);
            Vec2 object_vel = obj.momentum.times(1/obj.mass);
            float object_angular_speed = obj.angular_momentum/obj.rot_inertia;
            Vec2 point_vel_obj = object_vel.plus(r_perp.times(object_angular_speed));
            // Calculate the others point velocity
            Vec2 otherR = point.minus(other.center);
            Vec2 otherR_perp = perpendicular(otherR);
            Vec2 other_vel = other.momentum.times(1/other.mass);
            float other_angular_speed = other.angular_momentum/other.rot_inertia;
            Vec2 point_vel_other = other_vel.plus(otherR_perp.times(other_angular_speed));
            Vec2 rel_vel = point_vel_obj.minus(point_vel_other);                   // vAB
            // Find the hit normal
            // Find the two points that make up the edge of collision
            Vec2 p1 = new Vec2(0,0);
            Vec2 p2 = new Vec2(0,0);
            // Checks p1 to p2
            float d1 = other.p1.distanceTo(point);
            float d2 = other.p2.distanceTo(point);
            float buffer = 0.1;
            float length = other.p1.distanceTo(other.p2);
            if (d1+d2 >= (length - buffer) && d1+d2 <= (length + buffer)) { p1 = other.p1; p2 = other.p2;}
            // Checks p1 to p3
            d1 = other.p1.distanceTo(point);
            d2 = other.p3.distanceTo(point);
            length = other.p1.distanceTo(other.p3);
            if (d1+d2 >= (length - buffer) && d1+d2 <= (length + buffer)) { p1 = other.p1; p2 = other.p3;}
            // Check2 p2 to p4
            d1 = other.p2.distanceTo(point);
            d2 = other.p4.distanceTo(point);
            length = other.p2.distanceTo(other.p4);
            if (d1+d2 >= (length - buffer) && d1+d2 <= (length + buffer)) { p1 = other.p2; p2 = other.p4;}
            // Checks p3 to p4
            d1 = other.p3.distanceTo(point);
            d2 = other.p4.distanceTo(point);
            length = other.p3.distanceTo(other.p4);
            if (d1+d2 >= (length - buffer) && d1+d2 <= (length + buffer)) { p1 = other.p3; p2 = other.p4;}
            Vec2 hit_edge = p1.minus(p2);
            // Then take the perpendicular of that to get the hit normal
            Vec2 hit_normal = perpendicular(hit_edge);
            // Time to find the impulse
            float l = -(1 + box_bounce)*dot(rel_vel, hit_normal);
            l /= dot(hit_normal, hit_normal) * (1/obj.mass + 1/other.mass) + pow(dot(r_perp, hit_normal),2)/obj.rot_inertia + pow(dot(otherR_perp, hit_normal), 2)/other.rot_inertia;

            // Now use the impulse to figure out what happens
            Vec2 impulse = hit_normal.times(l);
            obj.momentum.add(impulse);
            obj.angular_momentum += (dot(r_perp, impulse)*(1/obj.rot_inertia));

            impulse = hit_normal.times(-l);
            other.momentum.add(impulse);
            other.angular_momentum += (dot(otherR_perp, impulse)*(1/other.rot_inertia));

            update_physics(1.01*dt); // Way to move the box out of the way, change it
            // float overlap = 0.5 * other.center.distanceTo(hit_edge) - other.center.distanceTo(point);
            // println(overlap);
            // other.center.subtract(hit_normal.normalized().times(overlap));
            // obj.center.add(hit_normal.normalized().times(overlap));
            return;
          }
        }
      }
    }
  }
}

ColideInfo collisionTest(Square obj){
  updateCornerPositions(obj); //Compute the 4 corners: p1,p2,p3,p4
  //We only check if the corners collide
  Vec2 p1,p2,p3,p4;
  ColideInfo info = new ColideInfo();
  p1 = obj.p1;
  p2 = obj.p2;
  p3 = obj.p3;
  p4 = obj.p4;
  if (p1.x > width){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p2.x > width){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p3.x > width){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  if (p4.x > width){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(-1,0);
  }
  //Test against the left wall
  if (p1.x < 0){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(1,0);
  }
  if (p2.x < 0){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(1,0);
  }
  if (p3.x < 0){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(1,0);
  }
  if (p4.x < 0){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(1,0);
  }
  // Collides with the bottom wall
  if (p1.y > height){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p2.y > height){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p3.y > height){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  if (p4.y > height){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(0,-1);
  }
  // Collides with the top wall
  if (p1.y < 0){
    info.hitPoint = p1;
    info.hit = true;
    info.objectNormal = new Vec2(0,1);
  }
  if (p2.y < 0){
    info.hitPoint = p2;
    info.hit = true;
    info.objectNormal = new Vec2(0,1);
  }
  if (p3.y < 0){
    info.hitPoint = p3;
    info.hit = true;
    info.objectNormal = new Vec2(0,1);
  }
  if (p4.y < 0){
    info.hitPoint = p4;
    info.hit = true;
    info.objectNormal = new Vec2(0,1);
  }
  
  return info;
}

//Updates momentum & angular_momentum based on collision using an impulse based method
//This method assumes you hit an immovable obstacle which simplifies the math
// see Eqn 8-18 of here: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
// or Eqn 9 here: http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
//for obstacle-obstacle collisions.
void resolveCollision(Vec2 hit_point, Vec2 hit_normal, float dt, Square obj){
  Vec2 r = hit_point.minus(obj.center);
  Vec2 r_perp = perpendicular(r);
  Vec2 object_vel = obj.momentum.times(1/obj.mass);
  float object_angular_speed = obj.angular_momentum/obj.rot_inertia;
  Vec2 point_vel = object_vel.plus(r_perp.times(object_angular_speed));
  // println(point_vel,object_vel);
  float j = -(1+box_bounce)*dot(point_vel,hit_normal);
  j /= (1/obj.mass + pow(dot(r_perp,hit_normal),2)/obj.rot_inertia);
  Vec2 impulse = hit_normal.times(j);
  obj.momentum.add(impulse);
  // println(momentum);
  obj.angular_momentum += dot(r_perp,impulse);
  update_physics(1.01*dt); //A small hack, better is just to move the object out of collision directly
}

void draw(){
  float dt = 1/frameRate;
  update_physics(dt);
  

  background(200); //Grey background

  // Make the boxes time
  for (int i = 0; i < things; i++){
    Square obj = s[i];
    boolean clicked_box = mousePressed && point_in_box(new Vec2(mouseX, mouseY),obj.center,obj.w,obj.h,obj.angle);
  
    if (clicked_box) {
      Vec2 force = new Vec2(1,0).times(100);
      if (arrow_angle == 90) force = new Vec2(0,1).times(100);
      if (arrow_angle == 180) force = new Vec2(-1,0).times(100);
      if (arrow_angle == 270) force = new Vec2(0,-1).times(100);
      Vec2 hit_point = new Vec2(mouseX, mouseY);
      apply_force(force, hit_point, obj);
    }

    ColideInfo info = collisionTest(obj); 
  
  
    Boolean hit_something = info.hit; 
    if (hit_something){
      Vec2 hit_point = info.hitPoint;
      Vec2 hit_normal = info.objectNormal;
      resolveCollision(hit_point,hit_normal,dt,obj);
    }
    
    boxCollisionTest(dt);

    fill(255);
    if (clicked_box){
      fill(255,200,200);
    }
    pushMatrix();
    translate(obj.center.x,obj.center.y);
    rotate(obj.angle);
    rect(-obj.w/2, -obj.h/2, obj.w, obj.h);
    popMatrix();
    
    fill(0);
    circle(obj.center.x, obj.center.y, 6.0);
    
    circle(obj.p1.x, obj.p1.y, 4.0);
    circle(obj.p2.x, obj.p2.y, 4.0);
    circle(obj.p3.x, obj.p3.y, 4.0);
    circle(obj.p4.x, obj.p4.y, 4.0);
  }
  
  
  
  
  // Vec2 box_vel = momentum.times(1/mass);
  // float box_speed = box_vel.length();
  // float box_agular_velocity = angular_momentum/rot_inertia;
  // float linear_kinetic_energy = .5*mass*box_speed*box_speed;
  // float rotational_kinetic_energy = .5*rot_inertia*box_agular_velocity*box_agular_velocity;
  // float total_kinetic_energy = linear_kinetic_energy+rotational_kinetic_energy;
  // println("Box Vel:",box_vel,"ang vel:",box_agular_velocity,"linear KE:",linear_kinetic_energy,"rotation KE:",rotational_kinetic_energy,"Total KE:",total_kinetic_energy);
  
  
  drawArrow(mouseX, mouseY, 100, arrow_angle);

}


void keyPressed(){
  if (key == 'r'){
    println("Resetting the simulation");
    for (int i = 0; i < things; i++){
      s[i].center = new Vec2(200 + (i * 200),200);
      s[i].angle = 0.0;
      s[i].momentum = new Vec2(0.0, 0.0);
      s[i].angular_momentum = 0.0;
    }
    
    return;
  }if (key == CODED){
    if (keyCode == LEFT){
        arrow_angle = 180;
      } if (keyCode == RIGHT){
        arrow_angle = 0;
      } if (keyCode == UP) {
        arrow_angle = 270;
      } if (keyCode == DOWN) {
        arrow_angle = 90;
      }
    }
}

//Returns true iff the point 'point' is inside the box
boolean point_in_box(Vec2 point, Vec2 box_center, float box_w, float box_h, float box_angle){
  Vec2 relative_pos = point.minus(box_center);
  Vec2 box_right = new Vec2(cos(box_angle),sin(box_angle));
  Vec2 box_up = new Vec2(sin(box_angle),cos(box_angle));
  float point_right = dot(relative_pos,box_right);
  float point_up = dot(relative_pos,box_up);
  if ((abs(point_right) < box_w/2) && (abs(point_up) < box_h/2))
    return true;
  return false;
}

void drawArrow(int cx, int cy, int len, float angle){
  pushMatrix();
  translate(cx, cy);
  rotate(radians(angle));
  line(-len,0,0, 0);
  line(0, 0,  - 8, -8);
  line(0, 0,  - 8, 8);
  popMatrix();
}
