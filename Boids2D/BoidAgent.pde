class BoidAgent{
 PVector position;
 PVector velocity;
 PVector acceleration;
 PShape model;
 float modelHeight = 100;
 float x1,x2,y1,y2,z1,z2;
 
 BoidAgent(float x1, float x2, float y1, float y2, float z1, float z2){
   this.x1 = x1; this.x2 = x2;
   this.y1 = y1; this.y2 = y2;
   this.z1 = z1; this.z2 = z2;
   this.position = new PVector(random(x1+10,x2-10), y2-modelHeight/2, random(z1+10,z2-10));
   this.velocity = new PVector(random(-1,1),0,random(-1,1));
   this.acceleration = new PVector(0,0,0);
 }
 
 void setModel(PShape model){
   this.model = model;
 }
 
 void alignWithMovement(){
   //PVector direction = velocity.copy().normalize();
   //float rho = direction.mag();
   //float phi = acos(direction.z / rho);
   //float theta = atan2(direction.y, direction.x);
   //rotateY(phi);
   //rotateZ(theta);
   
   //rotateX(atan2(velocity.y, velocity.z));
   //rotateY(atan2(velocity.x, velocity.z));
   
   float theta = acos(velocity.z / velocity.mag());
   PVector normal = velocity.cross(new PVector(0,0,1)).normalize();
   rotateAboutAxes(normal.x, normal.y, normal.z, theta);
 }
 
 void rotateAboutAxes(float x,float y,float z,float angle) {
  float yaw, pitch, roll;
  float s=sin(angle);
  float c=cos(angle); //<>//
  float t=1-c;
  if ((x*y*t + z*s) > 0.998) { // north pole singularity detected
    yaw = 2*atan2(x*sin(angle/2), cos(angle/2));
    pitch = PI/2;
    roll = 0;
    return;
  }
  if ((x*y*t + z*s) < -0.998) { // south pole singularity detected
    yaw = -2*atan2(x*sin(angle/2), cos(angle/2));
    pitch = -PI/2;
    roll = 0;
    return;
  }
  yaw = atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
  pitch = asin(x * y * t + z * s) ;
  roll = atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
  rotateY(yaw);
  rotateX(pitch);
  rotateZ(roll);
}
 
 void draw(){
   pushMatrix();
   translate(this.position.x, this.position.y, this.position.z);
   //strokeWeight(2);
   //stroke(0,255,0);
   //line(0,0,0, velocity.x*10, velocity.y*10, velocity.z*10);
   //alignWithMovement();
   //stroke(255,0,0);
   //line(0,0,0, 0,0,20);
   //float angleDiff = velocity.dot(0,0,1); //<>//
   //println(angleDiff);
   if(model == null){
     sphere(2);
   }else{
     scale(2);
     shape(model);
   }
   popMatrix();
 }
 
 void computeForces(){
   
 }
  
 void update(float dt){
   this.computeForces();
   this.acceleration.mult(dt);
   this.velocity.add( this.acceleration);
   this.position.add( this.velocity);
   
   // Boundary conditions. Reflect from boundary.
   if(position.x < x1){ position.x = x1+1; velocity.x *= -1;}
   if(position.x > x2){ position.x = x2-1; velocity.x *= -1;}
   
   if(position.y < y1){ position.y = y1+1; velocity.y *= -1;}
   if(position.y > y2){ position.y = y2-1; velocity.y *= -1;}
   
   if(position.z < z1){ position.z = z1+1; velocity.z *= -1;}
   if(position.z > z2){ position.z = z2-1; velocity.z *= -1;}
 }
}
