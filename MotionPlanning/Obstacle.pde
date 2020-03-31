class Obstacle{
  PVector position;
  PVector velocity;
  float radius = 20.0;
  color colour = color(165, 151, 129);
  
  Obstacle(PVector position, float radius){
    this.position = position;
    this.radius = radius;
    this.velocity = new PVector(0,0,0);
  }
  
  void move(float dt){
  }
  
  void draw(float centerX, float centerY, float centerZ){
    pushMatrix();
    fill(colour);
    stroke(colour);
    translate(centerX+position.x, centerY+position.y, centerZ+position.z);
    sphere(radius);
    popMatrix();
  }
}
