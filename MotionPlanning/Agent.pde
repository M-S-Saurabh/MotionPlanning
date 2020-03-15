class Agent{
  PVector position;
  float speed = 5;
  float radius = 5.0;
  color colour = color(220, 71, 48);
  
  Agent(PVector position){
    this.position = position;
  }
  
  void move(GraphNode start, GraphNode end, float dt){
    PVector startLocation = start.location;
    PVector endLocation = end.location;
    if(agent.position.dist(endLocation) < (agent.radius+0.5)){ 
      agent.position = endLocation.copy();
      currentNode++;
      return;
    }
    PVector direction = PVector.sub(endLocation, startLocation).normalize();
    direction.mult(agent.speed*dt);
    agent.position.add(direction);
  }
}
