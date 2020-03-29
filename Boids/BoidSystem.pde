class BoidSystem{
  ArrayList<BoidAgent> particles;
  PShape model = loadShape("Fish1.obj");
  int numParticles = 100;
  
  BoidSystem(float x1, float x2, float y1, float y2, float z1, float z2){
    this.particles = new ArrayList();
    for(int i=0; i<numParticles; i++){
      BoidAgent agent = new BoidAgent(x1, x2, y1, y2, z1, z2);
      agent.setModel(model);
      this.particles.add(agent);
    }
  }
  
  void draw(){
    for(BoidAgent agent : particles){
      agent.draw();
    }
  }
  
  void update(float dt){
    for(int i=particles.size()-1; i>=0; i--){
      BoidAgent p = particles.get(i);
      p.update(dt);
    }
  }
}
