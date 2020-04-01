int nMilestones = 200;
int minNeighbors = 5;

void constructCSpace(){
  invalidPositions = new ArrayList();
  for(Obstacle obstacle : obstacles){
    invalidPositions.add(new Sphere(obstacle.position, obstacle.radius+ Agent.radius));
  }
}

void sampleConfigurations(){
  milestones = new ArrayList();
  while(milestones.size() < nMilestones){
    PVector sample = new PVector(random(-planeW/2+5, planeW/2-5), 0, random(-planeH/2+5,planeH/2-5));
    boolean invalid = false; 
    for(Sphere s : invalidPositions){
      invalid = invalid || s.isInvalid(sample);
      if(invalid){break;}
    }
    if(!invalid){milestones.add(sample);}
  }
}

void printRoadGraph(){
  // print graph as adjacency list
  for(int i=roadmap.size()-1; i>=0; i--){
    GraphNode node = roadmap.get(i);
    print("P"+node.id+":");
    for(int j=node.neighbors.size()-1; j>=0; j--){
      GraphNode neighbor = node.neighbors.get(j);
      print(neighbor.id+" ");
    }
    print("\n");
  }
}

void constructSparseGraph(){
  roadmap =  new ArrayList();
  // Make nodes out for each milestone location.
  for(PVector milestone : milestones){
    GraphNode node = new GraphNode(milestone);
    roadmap.add(node);
  }
  // Adding startPoint and endPoint
  for(Agent agent : agents){
    roadmap.add(agent.start);
    roadmap.add(agent.goal);
  }
  
  ArrayList<ArrayList<Pair>> distance = new ArrayList();
  for(int i=0; i<roadmap.size(); i++){
    GraphNode node = roadmap.get(i);
    distance.add(new ArrayList());
    for(int j=0; j<roadmap.size(); j++){
      GraphNode neighbor = roadmap.get(j);
      float dist = node.location.dist(neighbor.location);
      Pair pair = new Pair(j, dist);
      distance.get(i).add(pair);
    }
  }
  for(int i=0; i<roadmap.size();i++){
    Collections.sort(distance.get(i));
  }
  for(int i=0; i<roadmap.size(); i++){
    GraphNode node = roadmap.get(i);
    int edgeCount = 0;
    for(int j=1; j<roadmap.size(); j++){
      Pair p = distance.get(i).get(j);
      GraphNode neighbor = roadmap.get(p.index);
      boolean addEdge = true;
      for(Obstacle obstacle : obstacles){
        if( lineCircleIntersect(node.location, neighbor.location, obstacle.position, obstacle.radius) ){
          addEdge = false;
          break;
        }
      }
      if(addEdge){
        node.addNeighbor(neighbor);
        edgeCount++;
      }
      if(edgeCount >= minNeighbors){ break;}
    }
  }
  //printRoadGraph();
}

// Taken from CCD code posted by Dr.Guy on canvas.
boolean lineCircleIntersect(PVector start, PVector end, PVector center, float radius){
  PVector toEnd = PVector.sub(end, start);
  float lineSegmentLength = toEnd.mag();
  toEnd.normalize(); // V
  PVector toCenter = PVector.sub(center, start); // W
  
  float a = 1; // ||V||^2 :since V is normalized
  float b = -2 * toEnd.dot(toCenter); // 2 * (P0-C) 
  float c = toCenter.magSq() - sq(radius); // ||P0-C||^2 - r^2
  float D = sq(b) - 4*a*c;
  
  boolean colliding = false;
  if(D>=0){
    float t = (-b - sqrt(D))/(2*a);
    if (t > 0 && t < lineSegmentLength){
      colliding = true;
    }
  }
  return colliding;
}
