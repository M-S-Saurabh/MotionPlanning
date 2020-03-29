import queasycam.*;

QueasyCam cam;

BoidSystem fishes;

boolean paused = false;

float X1=500, X2=1000;
float Y1=250, Y2=300;
float Z1=-250, Z2=250;

void setup(){
  size(1000,1000,P3D);
  cam = new QueasyCam(this);
  cam.speed = 5; 
  cam.sensitivity = 0.5;
  lights();
  
  fishes = new BoidSystem(X1,X2,Y1,Y2,Z1,Z2);
}

void drawOrigin(){
  pushMatrix();
  stroke(255);
  fill(255);
  sphere(3);
  popMatrix();
}

void drawWorldBounds(){
  noFill();
  stroke(255);
  strokeWeight(2);
  beginShape(QUAD_STRIP);
  vertex(X1, Y1, Z1);
  vertex(X1, Y2, Z1); //
  vertex(X2, Y1, Z1);
  vertex(X2, Y2, Z1);//
  vertex(X2, Y1, Z2);
  vertex(X2, Y2, Z2);//
  vertex(X1, Y1, Z2);
  vertex(X1, Y2, Z2);//
  endShape();
  line(X1, Y1, Z1, X1, Y1, Z2);
  line(X1, Y2, Z1, X1, Y2, Z2);
}

void drawPlane(){
  pushMatrix();
  
  translate((X1+X2)/2,(Y1+Y2)/2,(Z1+Z2)/2);
  box(X2-X1, Y2-Y1, Z2-Z1);
  popMatrix();
}

void draw(){
  background(0);
  //drawOrigin();
  drawWorldBounds();
  fishes.draw();
  if(!paused){
    update(0.1);
  }
}

void keyPressed(){
  if(key == 'p'){
    paused = !paused;
  }
}

void update(float dt){
  fishes.update(dt);
}
