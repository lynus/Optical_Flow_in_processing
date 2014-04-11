import processing.video.*;
Capture cam;
int w=640,h=480;
int[][] cur;
int[][] last;
double[][] flow;
PImage img_flow;
int[][] dx = new int[h][w];
int[][] dy = new int[h][w];
int[][] dt = new int[h][w];
class vt {

  double vx;
  double vy;
  vt(){
    vx=0.0;
    vy=0.0;
  }
};
void setup()
{
  size(640,480);
  String[] cameras = Capture.list();
  img_flow = new PImage(width,height);
  if (cameras.length == 0) {
    println("no cameras");
    exit();
  }
  cam = new Capture(this,cameras[0]);
  cam.start();
  frameRate(15);
//  cur = createImage(width,height,ARGB);
//  last = createImage(width,height,ARGB);
  while (!cam.available()){}
  cam.read();
  last = togray(cam);
  println("width "+cam.width+"  height "+cam.height);
}

int[][] togray(PImage I)
{
  I.loadPixels();
  int w = I.width;
  int h = I.height;
  int[][] ret = new int[height][width];
  for (int i=0; i<h; i++) {
    for (int j=0; j<w; j++) {
      color t= I.pixels[j+i*width];
      ret[i][j] = int(red(t)*0.299+green(t)*0.587+blue(t)*0.114);
    }
  }
  return ret;
}
PImage matrix2pimage(int[][] m)
{
  PImage ret = new PImage(width,height);
  for(int i=0; i<height; i++) {
    for (int j=0; j<width; j++) {
      color c=color(m[i][j],m[i][j],m[i][j]);
      ret.pixels[j+i*width] = c;
    }
  }
  ret.updatePixels();
  return ret;
  
}
void draw()
{
  if (!cam.available()) return;
  cam.read();
  //image(matrix2pimage(togray(cam)),0,0);
  cur = togray(cam);
  diffx();
//  diffy();
//  difft();
  cal_flow(25);
  last = cur;
  //image(matrix2pimage(dy),0,0);
  image(img_flow,0,0);
}

void diffx()
{
  for(int i=0; i<height;i++) {
    for(int j=0; j<width;j++) {
      if (j == width-1)
        dx[i][j]=dx[i][j-1];
      else if (i == height-1)
        dy[i][j]=dy[i-1][j];
      else {
        dx[i][j]= last[i][j+1]-last[i][j];
        dy[i][j]= last[i+1][j]-last[i][j];
        dt[i][j]= cur[i][j]-last[i][j];
      }
    }
  }
}


void diffy()
{
  for(int i=0; i<height;i++) {
    for(int j=0; j<width;j++) {
      if (i == height-1)
        dy[i][j]=dy[i-1][j];
      else
        dy[i][j]= last[i+1][j]-last[i][j];
    }
  }
}


void difft()
{
  for(int i=0; i<height;i++) {
    for(int j=0; j<width;j++) {
      dt[i][j]=cur[i][j]-last[i][j];
    }
  }
}

void cal_flow(int step)
{
  color c ;
  for (int i=0; i<=height-step;i+=step) {
    for (int j=0; j<=width-step; j+=step) {
      vt t=_cal_flow(i,j,step);
      println("vx: "+t.vx+" vy: "+t.vy);
      float v = sqrt((float)(t.vx*t.vx+t.vy*t.vy));
      if (v > 0.3) {
        int k =(int)( 128 + 128*t.vx);
        c = color(k,k,k);
      }else
      c = color(128,128,128);  
      for (int ii=0; ii<step; ii++)
        for (int jj=0; jj<step; jj++)
          img_flow.pixels[(i+ii)*width+jj+j] = c;
    }
  }
  img_flow.updatePixels();
}

vt _cal_flow(int i,int j,int step)
{
  //K-L algorithm
  //check out at http://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method
  long t;
  vt ret = new vt();
  long[][] mxt=multi(dx,dt,i,j,step);
  long[][] myt=multi(dy,dt,i,j,step);
  long[][] mxy=multi(dx,dy,i,j,step);
  long sx=sum(multi(dx,dx,i,j,step),  step);
  long sy=sum(multi(dy,dy,i,j,step),  step);
  long sxy=sum(mxy, step);
  long sxt= -sum(mxt, step);
  long syt= -sum(myt, step);
  //println("mxt: "+mxt+" myt: "+myt+" mxy"+mxy);
  //println("i: "+i+" j: "+j);
  //println("sx: "+sx+" sy: "+sy+" sxy: "+sxy+" sxt: "+sxt+" syt: "+syt);
  if (sx*sy-sxy*sxy != 0)
     t = (sx*sy-sxy*sxy);
  else
     t = 1;
  double a11 =(double) sy/t, a12 =(double)  (-sxy)/t, a21=a12, a22=(double)sx/t;
  //println("t: "+t+" sx*sy-sxy*sxy:"+(sx*sy-sxy*sxy));
  println("a11: "+a11+"       a12: "+a12+"       a21: "+a21+"      a22: "+a22);
  ret.vx=  a11*sxt+a12*syt;
  ret.vy=  a21*sxt+a22*syt;
  return ret;
}

long[][] multi(int[][] m1,int[][]m2, int i ,int j , int step)
{
  long[][] ret=new long[step][step];
  for(int ii=0; ii<step ;ii++)
    for(int jj=0; jj<step; jj++)
      ret[ii][jj]=(long)m1[i+ii][j+jj]*m2[i+ii][j+jj];
  return ret;
}

long sum(long[][] m, int step)
{
  long ret=0;
  for(int i=0; i<step ;i++)
    for(int j=0; j<step; j++)
      ret += m[i][j];
  return ret;
}
