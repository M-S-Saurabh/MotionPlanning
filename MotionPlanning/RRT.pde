int NUM_SAMPLES = 1000;

ArrayList<GraphNode> findRRT(PVector from, GraphNode to, boolean saveTree){
  TreeNode start = new TreeNode(from, null);
  ArrayList<TreeNode> tree = new ArrayList();
  tree.add(start);
  int nSample = 0;
  while(nSample < NUM_SAMPLES){
    PVector sample = null;
    if(nSample % 20 == 0){
      // Periodically select target location.
      sample = to.location.copy();
    }else{
      sample = new PVector(random(-planeH/2, planeH/2), 0, random(-planeW/2, planeW/2));
    }
    TreeNode parent = findClosest(sample, tree);
    TreeNode newNode = addToTree(parent, sample, obstacles, Agent.radius);
    tree.add(newNode);
    if(newNode.location.dist(to.location) == 0){
      if(saveTree){globalTree = tree;}
      return getPath(newNode);
    }
    nSample++;
  }
  if(saveTree){globalTree = tree;}
  return new ArrayList();
}

ArrayList<GraphNode> findRRT3D(PVector from, GraphNode to, boolean saveTree){
  TreeNode start = new TreeNode(from, null);
  ArrayList<TreeNode> tree = new ArrayList();
  tree.add(start);
  int nSample = 0;
  TreeNode closest = null;
  while(nSample < NUM_SAMPLES){
    PVector sample = null;
    if(nSample % 20 == 0){
      // Periodically select target location.
      sample = to.location.copy();
    }else{
      sample = new PVector(random(-planeH/2, planeH/2), random(-worldH, 0), random(-planeW/2, planeW/2));
    }
    TreeNode parent = findClosest(sample, tree);
    TreeNode newNode = addToTree(parent, sample, obstacles, Bird.radius);
    tree.add(newNode);
    if(newNode.location.dist(to.location) == 0){
      if(saveTree){globalTree = tree;}
      return getPath(newNode);
    }
    if(nSample % 20 == 0){closest = newNode;}
    nSample++;
  }
  if(saveTree){globalTree = tree;}
  return getPath(closest);
}

ArrayList<GraphNode> getPath(TreeNode node){
  ArrayList<GraphNode> list = new ArrayList();
  while(node.parent != null){
    list.add( (GraphNode) node);
    node = node.parent;
  }
  Collections.reverse(list);
  return list;
}

TreeNode findClosest(PVector sample, ArrayList<TreeNode> tree){
  float minDist = 1000;
  TreeNode result = null;
  for(TreeNode node : tree){
    float dist = node.location.dist(sample);
    if(dist < minDist){
      minDist = dist;
      result = node;
    }
  }
  return result;
}

TreeNode addToTree(TreeNode parent, PVector newLocation, ArrayList<Obstacle> obstacles, float excludeRadius){
  PVector tempLocation = newLocation.copy();
  for(Obstacle obstacle : obstacles){
    PVector closest = closestCollision(parent.location, tempLocation, obstacle.position, obstacle.radius+ excludeRadius);
    if(closest == null){ continue;}
    tempLocation = closest.copy();
  }
  
  // Add the closest location to tree.
  TreeNode newNode = new TreeNode(tempLocation, parent);
  parent.children.add(newNode);
  return newNode;
}

PVector closestCollision(PVector start, PVector end, PVector center, float radius){
  PVector toEnd = PVector.sub(end, start);
  float lineSegmentLength = toEnd.mag();
  toEnd.normalize(); // V
  PVector toCenter = PVector.sub(center, start); // W
  
  float a = 1; // ||V||^2 :since V is normalized
  float b = -2 * toEnd.dot(toCenter); // 2 * (P0-C) 
  float c = toCenter.magSq() - sq(radius); // ||P0-C||^2 - r^2
  float D = sq(b) - 4*a*c;
  
  PVector colliding = null;
  if(D>=0){
    float t = (-b - sqrt(D))/(2*a);
    if (t > 0 && t < lineSegmentLength){
      colliding = PVector.add(start, PVector.mult(toEnd,t- 0.1*lineSegmentLength));
    }
  }
  return colliding;
}
