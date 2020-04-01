PVector oldCameraPosition;
float oldCameraTilt;
float oldCameraPan;

void setTopCamera(){
  oldCameraPosition = cam.position.copy();
  oldCameraTilt = cam.tilt;
  oldCameraPan = cam.pan;
  
  cam.position = new PVector(centerX, centerY-400, centerZ);
  cam.tilt = PI/2;
  cam.pan = 0;
}

void resetCamera(){
  cam.position = oldCameraPosition.copy();
  cam.tilt = oldCameraTilt;
  cam.pan = oldCameraPan;
  cam.controllable = true;
}

PVector mapMouse(){
  if(mouseY > 75 && mouseY < 940 && mouseX > 70 && mouseX < 930){
    return new PVector( map(mouseY,75,940,planeH/2,-planeH/2), 0, map(mouseX,70,930,-planeW/2,planeW/2));
  }else{
    return null;
  }
}
