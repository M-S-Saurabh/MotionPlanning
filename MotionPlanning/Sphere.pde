class Sphere{
  PVector center;
  float radius;
  
  Sphere(PVector center, float radius){
    this.center = center.copy();
    this.radius = radius;
  }
  
  boolean isInvalid(PVector point){
    return (PVector.dist(point, this.center) <= this.radius);
  }
}
