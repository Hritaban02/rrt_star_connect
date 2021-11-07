#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <bits/stdc++.h>

using namespace cv;
using namespace std;

int connected=0;
int state=1;
int sub_state=0;
float step_size=5.0;
float circle_radius_1=20.0;
float circle_radius_2=5.0;

struct point{
  int x,y;
};

point path_points_start[500];
int pathcount_start=0;
point path_points_end[500];
int pathcount_end=0;

struct node{
  point loc;
  node* parent;
  float cost;
};

node start_nodes[10000];
int nodecount_start=0;
node end_nodes[10000];
int nodecount_end=0;

float dist(point p1, point p2){
  int x1=p1.x, y1=p1.y, x2=p2.x, y2=p2.y;

  float distance=sqrt(pow(x1-x2,2)+pow(y1-y2,2));
  return distance;
}

int nearest_node(node rand){
  float min=999.0;
  int index;

  if(state==1){
    for(int i=0; i<nodecount_start; i++){
      if(dist(rand.loc, start_nodes[i].loc)<min){
        min=dist(rand.loc, start_nodes[i].loc);
        index=i;
      }
    }
  }
  else{
    for(int i=0; i<nodecount_end; i++){
      if(dist(rand.loc, end_nodes[i].loc)<min){
        min=dist(rand.loc, end_nodes[i].loc);
        index=i;
      }
    }
  }
  return index;
}

point step_func(point near, point rand, float size_step){
  int dx=rand.x-near.x, dy=rand.y-near.y;
  float d=sqrt((dx*dx)+(dy*dy));

  point step;

  step.x=(int)(near.x+((size_step)*((float)(dx/d))));
  step.y=(int)(near.y+((size_step)*((float)(dy/d))));

  return step;
}

bool isValid(Mat obst, point near, point step){
  if(obst.at<uchar>(step.y,step.x)==255){
    return false;
  }

  int x1=near.x,x2=step.x;
  if(near.x>step.x){
    x1=step.x;
    x2=near.x;
  }
  if(x1!=x2){
    float m=(float)(step.y-near.y)/(step.x-near.x);
    float c=(float)(near.y)-(near.x*m);
    int j;
    for(int i=x1; i<=x2; i++){
      j=(int)((m*i)+c);
      if(obst.at<uchar>(j,i)==255){
        return false;
      }
    }
  }

  int y1=near.y,y2=step.y;
  if(near.y>step.y){
    y1=step.y;
    y2=near.y;
  }
  if(y1!=y2){
    float m=(float)(step.x-near.x)/(step.y-near.y);
    float c=(float)(near.x)-(near.y*m);
    int j;
    for(int i=y1; i<=y2; i++){
      j=(int)((m*i)+c);
      if(obst.at<uchar>(i,j)==255){
        return false;
      }
    }
  }

  return true;
}

void minimal_cost(Mat obst,node* step){
  float new_cost;
  float min_cost=step->cost;
  int index;
  if(state==1){
    for(int i=0; i<nodecount_start; i++){
      if(dist(start_nodes[i].loc,step->loc)<circle_radius_1&&isValid(obst,start_nodes[i].loc,step->loc)){
        new_cost=dist(start_nodes[i].loc,step->loc)+start_nodes[i].cost;
        if(new_cost<min_cost){
          min_cost=new_cost;
          index=i;
        }
      }
    }
    if(min_cost<step->cost){
      step->parent=&start_nodes[index];
      step->cost=min_cost;
    }
  }
  else{
    for(int i=0; i<nodecount_end; i++){
      if(dist(end_nodes[i].loc,step->loc)<circle_radius_1&&isValid(obst,end_nodes[i].loc,step->loc)){
        new_cost=dist(end_nodes[i].loc,step->loc)+end_nodes[i].cost;
        if(new_cost<min_cost){
          min_cost=new_cost;
          index=i;
        }
      }
    }
    if(min_cost<step->cost){
      step->parent=&end_nodes[index];
      step->cost=min_cost;
    }
  }
}

void rewiring(Mat &image, Mat obst,node* step){
  float new_cost;
  if(state==1){
    for(int i=0; i<nodecount_start; i++){
      if(dist(start_nodes[i].loc,step->loc)<circle_radius_2){
        new_cost=dist(start_nodes[i].loc,step->loc)+(step->cost);
        if(new_cost<start_nodes[i].cost&&isValid(obst,start_nodes[i].loc,step->loc)){
          start_nodes[i].cost=new_cost;
          start_nodes[i].parent=step;
          line(image,Point(start_nodes[i].loc.x,start_nodes[i].loc.y),Point((start_nodes[i].parent->loc).x,(start_nodes[i].parent->loc).y),Scalar(0,255,0),2,8);
        }
      }
    }
  }
  else{
    for(int i=0; i<nodecount_end; i++){
      if(dist(end_nodes[i].loc,step->loc)<circle_radius_2){
        new_cost=dist(end_nodes[i].loc,step->loc)+(step->cost);
        if(new_cost<end_nodes[i].cost&&isValid(obst,end_nodes[i].loc,step->loc)){
          end_nodes[i].cost=new_cost;
          end_nodes[i].parent=step;
          line(image,Point(end_nodes[i].loc.x,end_nodes[i].loc.y),Point((end_nodes[i].parent->loc).x,(end_nodes[i].parent->loc).y),Scalar(0,255,0),2,8);
        }
      }
    }
  }
}

void path(Mat &image, int index_1, int index_2){
  node n1,n2;
  n2=start_nodes[index_1-1];
  n1=*(n2.parent);
  while(n1.parent!=NULL){
    line(image, Point(n1.loc.x,n1.loc.y), Point(n2.loc.x,n2.loc.y), Scalar(255,0,0),2,8);
    n2=n1;
    n1=*(n2.parent);
  }
  n2=end_nodes[index_2-1];
  n1=*(n2.parent);
  while(n1.parent!=NULL){
    line(image, Point(n1.loc.x,n1.loc.y), Point(n2.loc.x,n2.loc.y), Scalar(255,0,0),2,8);
    n2=n1;
    n1=*(n2.parent);
  }
}

void path_points(int index_1, int index_2){

  pathcount_start=0;
  pathcount_end=0;
  node n1,n2;
  float d=20.0;

  n2=start_nodes[index_1-1];
  n1=*(n2.parent);
  path_points_start[pathcount_start++]=n2.loc;
  while(n1.parent!=NULL){
    if(dist(n1.loc,path_points_start[pathcount_start-1])<d){
      n1=*(n1.parent);
    }
    else{
      n2=n1;
      n1=*(n2.parent);
      path_points_start[pathcount_start++]=n2.loc;
    }
  }

  n2=end_nodes[index_2-1];
  n1=*(n2.parent);
  path_points_end[pathcount_end++]=n2.loc;
  while(n1.parent!=NULL){
    if(dist(n1.loc,path_points_end[pathcount_end-1])<d){
      n1=*(n1.parent);
    }
    else{
      n2=n1;
      n1=*(n2.parent);
      path_points_end[pathcount_end++]=n2.loc;
    }
  }


}

void rrt_connect(point p_start, point p_end, Mat &img, Mat obst){

  connected=0;
  state=1;
  sub_state=0;
  nodecount_start=0;
  nodecount_end=0;

  node start_node,end_node,rand_node;
  int index;

  node temp;
  node* step_node=&temp;

  node sub_temp;
  node* sub_step_node=&sub_temp;

  srand(time(NULL));

  start_node.loc.x=p_start.x;
  start_node.loc.y=p_start.y;
  start_node.parent=NULL;
  start_node.cost=0;
  end_node.loc.x=p_end.x;
  end_node.loc.y=p_end.y;
  end_node.parent=NULL;
  end_node.cost=0;

  start_nodes[nodecount_start]=start_node;
  nodecount_start++;
  end_nodes[nodecount_end]=end_node;
  nodecount_end++;

  while(connected!=1){

    sub_state=0;

    if(state==1){

      rand_node.loc.x=rand()%obst.cols;
      rand_node.loc.y=rand()%obst.rows;

      index=nearest_node(rand_node);

      if(dist(start_nodes[index].loc,rand_node.loc)<step_size){
        continue;
      }
      else{
        (step_node->loc)=step_func(start_nodes[index].loc,rand_node.loc,step_size);
      }

      if(isValid(obst,start_nodes[index].loc,step_node->loc)==false){
        continue;
      }
      else{
        step_node->parent=&start_nodes[index];
        step_node->cost=start_nodes[index].cost+step_size;
        minimal_cost(obst,step_node);
        //rewiring(img,obst,step_node);
        start_nodes[nodecount_start]=*step_node;
        nodecount_start++;
      }

      line(img,Point((step_node->loc).x,(step_node->loc).y),Point((step_node->parent->loc).x,(step_node->parent->loc).y),Scalar(0,255,255),1,8);
      imshow("IMAGE",img);
      waitKey(50);

      state=2;

      while(sub_state!=1){

        index=nearest_node(*step_node);

        if(dist(end_nodes[index].loc,step_node->loc)<20.0&&isValid(obst,step_node->loc,end_nodes[index].loc)){
            line(img, Point((step_node->loc).x,(step_node->loc).y), Point(end_nodes[index].loc.x,end_nodes[index].loc.y), Scalar(255,0,0),2,8);
            connected=1;
            sub_state=1;
            std::cout << "DONE" << '\n';
            path(img, nodecount_start, index+1);
            path_points(nodecount_start, index+1);
            continue;
        }
        else{
          (sub_step_node->loc)=step_func(end_nodes[index].loc,step_node->loc,step_size);
        }

        if(isValid(obst,end_nodes[index].loc,sub_step_node->loc)==false){
          sub_state=1;
          continue;
        }
        else{
          sub_step_node->parent=&end_nodes[index];
          sub_step_node->cost=end_nodes[index].cost+step_size;
          minimal_cost(obst,sub_step_node);
          //rewiring(img,obst,sub_step_node);
          end_nodes[nodecount_end]=*sub_step_node;
          nodecount_end++;
        }

        line(img,Point((sub_step_node->loc).x,(sub_step_node->loc).y),Point((sub_step_node->parent->loc).x,(sub_step_node->parent->loc).y),Scalar(0,255,255),1,8);
        imshow("IMAGE",img);
        waitKey(50);
      }

    }

    if(state==2){

      rand_node.loc.x=rand()%obst.cols;
      rand_node.loc.y=rand()%obst.rows;

      index=nearest_node(rand_node);

      if(dist(end_nodes[index].loc,rand_node.loc)<step_size){
        continue;
      }
      else{
        (step_node->loc)=step_func(end_nodes[index].loc,rand_node.loc,step_size);
      }

      if(isValid(obst,end_nodes[index].loc,step_node->loc)==false){
        continue;
      }
      else{
        step_node->parent=&end_nodes[index];
        step_node->cost=end_nodes[index].cost+step_size;
        minimal_cost(obst,step_node);
        //rewiring(img,obst,step_node);
        end_nodes[nodecount_end]=*step_node;
        nodecount_end++;
      }

      line(img,Point((step_node->loc).x,(step_node->loc).y),Point((step_node->parent->loc).x,(step_node->parent->loc).y),Scalar(0,255,255),1,8);
      imshow("IMAGE",img);
      waitKey(50);

      state=1;

      while(sub_state!=1){

        index=nearest_node(*step_node);

        if(dist(start_nodes[index].loc,step_node->loc)<10.0&&isValid(obst,step_node->loc,start_nodes[index].loc)){
            line(img, Point((step_node->loc).x,(step_node->loc).y), Point(start_nodes[index].loc.x,start_nodes[index].loc.y), Scalar(255,0,0),2,8);
            connected=1;
            sub_state=1;
            std::cout << "DONE" << '\n';
            path(img, index+1, nodecount_end);
            path_points(index+1, nodecount_end);
            continue;
        }
        else{
          (sub_step_node->loc)=step_func(start_nodes[index].loc,step_node->loc,step_size);
        }

        if(isValid(obst,start_nodes[index].loc,sub_step_node->loc)==false){
          sub_state=1;
          continue;
        }
        else{
          sub_step_node->parent=&start_nodes[index];
          sub_step_node->cost=start_nodes[index].cost+step_size;
          minimal_cost(obst,sub_step_node);
          //rewiring(img,obst,sub_step_node);
          start_nodes[nodecount_start]=*sub_step_node;
          nodecount_start++;
        }

        line(img,Point((sub_step_node->loc).x,(sub_step_node->loc).y),Point((sub_step_node->parent->loc).x,(sub_step_node->parent->loc).y),Scalar(0,255,255),1,8);
        imshow("IMAGE",img);
        waitKey(50);
      }

    }

  }

  imshow("IMAGE",img);
  waitKey(0);

}

int main(int argc, char const *argv[]) {

    /*-----------------------------------IP---------------------------------*/

    namedWindow("IMAGE",WINDOW_AUTOSIZE);
    Mat img=imread("rrt_connect_2.png",1);
    Mat obst=imread("rrt_connect_2.png",0);

    Mat start(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
    Mat end(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    for(int i=0;i<img.rows;i++){
      for(int j=0;j<img.cols;j++){
        if(img.at<Vec3b>(i,j)[1]>100&&img.at<Vec3b>(i,j)[2]<100){
          start.at<Vec3b>(i,j)[1]=img.at<Vec3b>(i,j)[1];
        }
        if(img.at<Vec3b>(i,j)[2]>100&&img.at<Vec3b>(i,j)[1]<100){
          end.at<Vec3b>(i,j)[2]=img.at<Vec3b>(i,j)[2];
        }
      }
    }

    Mat gray_start;
    cvtColor(start,gray_start,COLOR_BGR2GRAY);
    Mat gray_end;
    cvtColor(end,gray_end,COLOR_BGR2GRAY);

    medianBlur(gray_start,gray_start,5);
    medianBlur(gray_end,gray_end,5);

    vector<Vec3f> circle_start;
    vector<Vec3f> circle_end;

    Canny( gray_start, gray_start, 100, 100*3, 3 );
    Canny( gray_end, gray_end, 100, 100*3, 3 );

    vector<vector<Point>> contours_start,contours_end;
		vector<Vec4i> hierarchy_start,hierarchy_end;
    findContours(gray_start,contours_start,hierarchy_start,RETR_LIST,CHAIN_APPROX_SIMPLE,Point(0,0));
    findContours(gray_end,contours_end,hierarchy_end,RETR_LIST,CHAIN_APPROX_SIMPLE,Point(0,0));

    drawContours(gray_start,contours_start,-1,255,1,8,hierarchy_start,2,Point(0,0));
    drawContours(gray_end,contours_end,-1,255,1,8,hierarchy_end,2,Point(0,0));

    Moments m_start=moments(gray_start,true);
		Point p_start(m_start.m10/m_start.m00, m_start.m01/m_start.m00);
		circle(img,p_start,5,Scalar(255,255,255),-1);
    circle(obst,p_start,20,Scalar(0),-1);

    Moments m_end=moments(gray_end,true);
		Point p_end(m_end.m10/m_end.m00, m_end.m01/m_end.m00);
		circle(img,p_end,5,Scalar(255,255,255),-1);
    circle(obst,p_end,20,Scalar(0),-1);

    for(int i=0;i<obst.rows;i++){
      for(int j=0;j<obst.cols;j++){
        if(obst.at<uchar>(i,j)>100){
          obst.at<uchar>(i,j)=255;
        }
        else{
          obst.at<uchar>(i,j)=0;
        }
      }
    }

    int size=3;
    Mat element = getStructuringElement(MORPH_RECT,Size( 2*size + 1, 2*size+1 ),Point(size,size));
    for(int i=0;i<3;i++)
      dilate(obst,obst,element);

    /*----------------------------------RRT----------------------------------*/
    point s,e;
    s.x=p_start.x;
    s.y=p_start.y;
    e.x=p_end.x;
    e.y=p_end.y;
    cout<<s.x*11.0/600.0<<" "<<(600.0-s.y)*11.0/600.0<<" "<<e.x*11.0/600.0<<" "<<(600.0-e.y)*11.0/600.0<<"\n";
    rrt_connect(s,e,img,obst);

    return 0;

}
