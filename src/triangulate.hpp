#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;


Mat IterativeLinearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1 );


Mat LinearLSTriangulation(Point3d u,  Matx34d P,Point3d u1, Matx34d P1)	
								    
{
	
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	//	cout << "u " << u <<", u1 " << u1 << endl;
	//	Matx<double,6,4> A; //this is for the AX=0 case, and with linear dependence..
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u.x*P(1)-u.y*P(0);
	//	A(3) = u1.x*P1(2)-P1(0);
	//	A(4) = u1.y*P1(2)-P1(1);
	//	A(5) = u1.x*P(1)-u1.y*P1(0);
	//	Matx43d A; //not working for some reason...
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u1.x*P1(2)-P1(0);
	//	A(3) = u1.y*P1(2)-P1(1);
	Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),		
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	
	return X;
}






void triangulatePoints(const vector<Point2f>& pt_set1, const vector<Point2f>& pt_set2,const Mat& Kinv,const Matx34d& P,const Matx34d& P1, Mat& triag_points, vector<Point2f>& correspImg1Pt)
{
    vector<double> depths;
    vector<Point3d> pointcloud;
    pointcloud.clear();
    correspImg1Pt.clear();
    double t = getTickCount();
    unsigned int pts_size = pt_set1.size();
    //cout << "Triangulating..." << pts_size <<  endl ;

    cout << "Recieved total point " << pts_size << "for triangulation" << endl ;

    for (int i=0; i<pts_size; i++) {


    	Point2f kp = pt_set1[i];
        Point3f u(kp.x,kp.y,1.0);

        Mat u_mat(u);
        Mat um = Kinv * u_mat;
	
    	Point3d u_new(um.at<float>(0,0),um.at<float>(0,1),um.at<float>(0,2));
        Point2f kp1 = pt_set2[i];
        Point3f u1(kp1.x,kp1.y,1.0);
       
        cv::Mat u1_mat(u1);
        Mat um1 = Kinv * u1_mat;
        Point3d u1_new(um1.at<float>(0,0),um1.at<float>(0,1),um1.at<float>(0,2));
        Mat X = IterativeLinearLSTriangulation(u_new,P,u1_new,P1);

//      if(X.at<double>(2,0) > 6 || X.at<double>(2,0) < 0) continue;
 

         triag_points.at<float>(0,i)=X.at<double>(0,0);
         triag_points.at<float>(1,i)=X.at<double>(1,0);
         triag_points.at<float>(2,i)=X.at<double>(2,0);
         triag_points.at<float>(3,i)=1;

         pointcloud.push_back(Point3d(X.at<double>(0,0),X.at<double>(1,0),X.at<double>(2,0)));
         correspImg1Pt.push_back(pt_set1[i]);
         depths.push_back(X.at<double>(2,0));
        

	}

    	cout << "Depth found for Total  point " << depths.size() <<  endl ;

	//	cout << "triangulate points" << triag_points.at<float>(3,0) << endl ;


	t = ((double)getTickCount() - t)/getTickFrequency();
        
	cout << "Done. ("<<pointcloud.size()<<"points, " << t << ")"<< endl; 

	double minVal,maxVal;
    minMaxLoc(depths, &minVal, &maxVal);


	Mat tmp(240,320,CV_8UC3,Scalar(0,0,0)); //cvtColor(img_1_orig, tmp, CV_BGR2HSV);
	for (unsigned int i=0; i<pointcloud.size(); i++) {
		double _d = MAX(MIN((pointcloud[i].z-minVal)/(maxVal-minVal),1.0),0.0);
		circle(tmp, correspImg1Pt[i], 1, Scalar(255 * (1.0-(_d)),255,255), CV_FILLED);
	}
        cvtColor(tmp, tmp, CV_HSV2BGR);
        cv::namedWindow("depths",cv::WINDOW_NORMAL);
        cv::startWindowThread();
	
        imshow("depths", tmp);


}
/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat IterativeLinearLSTriangulation(Point3d u, Matx34d P,Point3d u1, Matx34d P1)

 {
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1); 

    for (int i=0; i<100; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
         
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
        
	double EPSILON=0.5; 
        //breaking point
        if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,DECOMP_SVD);

	//cout << "X_" << X_ << endl ;
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
//	cout << "size of X" << X.size()<< "X" << X(0,0)  << " X(0) "<<X(0) << X(1,0)  << " X(1) " << X(1) << X(2,0) << " X(2) "<< X(2) << " "<< X(3,0)<< endl;   
 }
    return X;
}
