
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
    for(int j=1; j<minNeighbors; j++){
      Pair p = distance.get(i).get(j);
      GraphNode neighbor = roadmap.get(p.index);
      node.addNeighbor(neighbor);
    }
  }
  //printRoadGraph();
}

boolean checkCollision(PVector center, float radius, PVector start, PVector end){
  PVector line = PVector.sub(end, start).normalize();
  PVector centerToStart = PVector.sub(start, center);
  PVector perpendicular = PVector.sub(centerToStart, PVector.mult(line, centerToStart.dot(line)));
  float perpendicularLength = perpendicular.mag();
  if(perpendicularLength >= radius){return false;}
  PVector perpendicularBase = PVector.add(perpendicular, center);
  float distToStart = perpendicularBase.dist(start);
  float distToEnd = perpendicularBase.dist(end);
  float lineMag = start.dist(end);
  return abs(distToStart + distToEnd - lineMag) < 0.001;
}
