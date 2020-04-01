class Obstacle{
  PVector position;
  PVector velocity;
  PShape model;
  float radius = 20.0;
  color colour = color(165, 151, 129);
  
  Obstacle(PVector position, float radius, PShape model){
    this.position = position;
    this.radius = radius;
    this.velocity = new PVector(0,0,0);
    this.model = model;
  }
  
  void move(float dt){
  }
  
  void draw(float centerX, float centerY, float centerZ){
    pushMatrix();
    fill(colour);
    stroke(colour);
    translate(centerX+position.x, centerY+position.y, centerZ+position.z);
    //sphere(radius);
    rotateX(PI);
    float s = 0.75 * this.radius;
    scale(s);
    model.setFill(colour);
    shape(model);
    popMatrix();
  }
}
