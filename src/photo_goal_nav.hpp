#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "triangulate.hpp"
#include "TooN/TooN.h"
#include "TooN/so3.h"
#include "TooN/se3.h"

/**

 * \brief Compute and draw the epipolar lines in two images
 *      associated to each other by a fundamental matrix
 *
 * \param title     Title of the window to display
 * \param F         Fundamental matrix
 * \param img1      First image
 * \param img2      Second image
 * \param points1   Set of points in the first image
 * \param points2   Set of points in the second image matching to the first set
 * \param inlierDistance      Points with a high distance to the epipolar lines are
 *                not displayed. If it is negative, all points are displayed
 **/

using namespace std;
using namespace cv;

typedef cv::Vec<float, 4> Vec4f;

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
  //Line is given as a*x + b*y + c = 0
  return std::fabs(line(0)*point.x + line(1)*point.y + line(2)) / std::sqrt(line(0)*line(0)+line(1)*line(1));
}

static cv::Mat ConstructP(cv::Mat R, cv::Mat T, cv::Mat K);

static cv::Mat rigid_transform(cv::Mat R, cv::Mat T, cv::Mat points4f);

static cv::Mat convert_channel_mat(cv::Mat channel_mat);

static cv::Mat computeNewCamCenter(cv::Mat R, cv::Mat T)
{
return - R.t() * T;
}

template <typename T1, typename T2>
static void drawEpipolarLines( std::string& title, cv::Matx<T1,3,3> F,
                 cv::Mat image1,  cv::Mat image2,
                 std::vector<cv::Point_<T2> > points1,
                 std::vector<cv::Point_<T2> > points2,
                 float inlierDistance = -1)
{

	cv::Mat img1=image1.clone();
    cv::Mat img2=image2.clone();

   CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());



  cv::Mat outImg(img1.rows, img1.cols*2, CV_32F);
  cv::Rect rect1(0,0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U)
  {
    std::cout << "here1" << std::endl ;
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  }
  else
  {
//    std::cout << "Image type:: " << img1.type() << "::well" << outImg.type() <<  std::endl ;
  //  img1.copyTo(outImg(rect1));
   // img2.copyTo(outImg(rect2));
   // std::cout << "will" << std::endl ;
  }
  std::vector<cv::Vec<T2,3> > epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);
 
  CV_Assert(points1.size() == points2.size() &&
        points2.size() == epilines1.size() &&
        epilines1.size() == epilines2.size());
 
  cv::RNG rng(0);
  for(size_t i=0; i<points1.size(); i++)
  {
    if(inlierDistance > 0)
    {
      if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
        distancePointLine(points2[i], epilines1[i]) > inlierDistance)
      {
        //The point match is no inlier
        continue;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
     */
    cv::Scalar color(rng(256),rng(256),rng(256));
 
//    cv::line(outImg(rect2), cv::Point(0,-epilines1[i][2]/epilines1[i][1]),cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),color);
    cv::line(img2, cv::Point(0,-epilines1[i][2]/epilines1[i][1]),cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),color);
   
    // std::cout << "now" << std::endl ; 
    // cv::Mat testImage= outImg(rect1);
  //   std::cout << "now2" << std::endl ;

//     std::cout << "test image type" << testImage.type() << "Inout image type" << img1.type() << std::endl ;

   //  cv::circle(testImage, points1[i], 3, color, -1, CV_AA);
     cv::circle(img1, points1[i], 3, color, -1, CV_AA);
     cv::circle(img2,points2[i],3,color,-1,CV_AA);	
/*    cv::line(outImg(rect1),
      cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
      cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
      color);
    cv::circle(outImg(rect2), points2[i], 3, color, -1, CV_AA);
*/ 
 }
  	 cv::namedWindow(title, cv::WINDOW_NORMAL);
     cv::startWindowThread(); 
     cv::imshow(title, img1);


     std::string title1 = "Epilines2" ;      
     cv::namedWindow(title1, cv::WINDOW_NORMAL);
     cv::startWindowThread(); 
     cv::imshow(title1,img2);

//  cv::imshow(title, img2);

  cv::waitKey(1);
}


int find_minimum_depth_index(int* depth_array,int size){
 int minimum = depth_array[0];
 int index=0 ;
    for ( int c = 0 ; c < size ; c++ ) 
    {
        if ( depth_array[c] < minimum ) 
        {
           minimum = depth_array[c];
           index = c;
        }
    } 

return index ;

 }


int find_minimum_depth_index(float* depth_array,int size){
	cout << "Received Depths" << depth_array[0] << endl ;
 float minimum = depth_array[0];
 int index=0 ;
    for ( int c = 0 ; c < size ; c++ ) 
    {
        if ( depth_array[c] < minimum ) 
        {
           minimum = depth_array[c];
           index = c;
        }
    } 

return index ;

} 

int find_maximum_depth_index(float* depth_array,int size){
 float maximum = depth_array[0];
 int index=0 ;
    for ( int c = 0 ; c < size ; c++ )
    {
        if ( depth_array[c] > maximum )
        {
           maximum = depth_array[c];
           index = c;
        }
    }

return index ;

}



TooN::SO3<float> toToonSO3(cv::Mat rot){
	TooN::Matrix<3,3> mat;
	mat(0,0) =rot.at<float>(0, 0);
	mat(0,1) = rot.at<float>(0, 1);
	mat(0,2) = rot.at<float>(0, 2) ;

	mat(1,0) = rot.at<float>(1, 0);
	mat(1,1) = rot.at<float>(1, 1);
	mat(1,2) = rot.at<float>(1, 2);

	mat(2,0) = rot.at<float>(2, 0);
	mat(2,1) =rot.at<float>(2, 1);
	mat(2,2) =rot.at<float>(2, 2);

	TooN::SO3<float> res = mat;
	return res;
}

void cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose)
{
    pose = cv::Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)cv::norm(H.col(0));  
    float norm2 = (float)cv::norm(H.col(1));  
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    cv::Mat p1 = H.col(0);       // Pointer to first column of H
    cv::Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

    p1 = H.col(1);           // Pointer to second column of H
    p2 = pose.col(1);        // Pointer to second column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

    p1 = pose.col(0);
    p2 = pose.col(1);

    cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    cv::Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

    pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}

static void validateRandT(cv::Mat K1,cv::Mat K2,std::vector<float> distCoeff,cv::Mat candidateR1,cv::Mat candidateR2,cv::Mat candidateT1,cv::Mat candidateT2,cv::Mat img1,  cv::Mat img2,
        std::vector<cv::Point2f> points1,
        std::vector<cv::Point2f> points2,cv::Mat &validR, cv::Mat &validT ){

	for(int j=0; j < points1.size() ; j++){
		std::cout<<points1[j]<<"-->"<<points2[j]<<std::endl;
	}
	std::cout<<"Points1\n"<<points1<<std::endl;
	std::cout<<"Points2\n"<<points2<<std::endl;

	std::vector<cv::Point2f> undistort_points2;
	std::vector<cv::Point2f> undistort_points1;
	std::vector<float> zerodist;
	
	cv::undistortPoints(points1,undistort_points1,K1,zerodist);
	cv::undistortPoints(points2,undistort_points2,K1,zerodist);
	//points1 = undistort_points1;
	//points2 = undistort_points2; 
	std::cout<<"Vaildating R and T \n";
	cv::Mat T0(3,1,CV_32F);
	T0.at<float>(0,0)=0;
	T0.at<float>(0,1)=0;	
	T0.at<float>(0,2)=0;


	cv::Mat P1 = ConstructP(cv::Mat::eye(3,3,CV_32FC1),T0,K1);
	std::cout<<"P1 \n"<<P1<<std::endl;
	cv::Mat P2_1 = ConstructP(candidateR1,candidateT1,K2);
	
	std::cout<<"P2_1 \n"<<P2_1<<std::endl;
	cv::Mat P2_2 = ConstructP(candidateR1,candidateT2,K2);
	cv::Mat P2_3 = ConstructP(candidateR2,candidateT1,K2);
	cv::Mat P2_4 = ConstructP(candidateR2,candidateT2,K2);

	int N =points1.size();
	std::vector<cv::Point2f> considered_points1(points1.begin(),points1.begin()+N);
	std::vector<cv::Point2f> considered_points2(points2.begin(),points2.begin()+N);
	std::vector<cv::Vec4f> points4d;
	points4d.resize(N);
	cv::Mat pnts3D1(1,N,CV_32FC4);
	cv::Mat pnts3D2(1,N,CV_32FC4);
	cv::Mat pnts3D3(1,N,CV_32FC4);
	cv::Mat pnts3D4(1,N,CV_32FC4);
	//cv::Mat cam0pnts(1,N,CV_64FC2);
	//cv::Mat cam1pnts(1,N,CV_64FC2);

	std::cout<<"ConsideredPoints"<<std::endl;
	int case1_neg =0;
	int case2_neg =0;
	int case3_neg =0;
        int case4_neg =0;
	


        //std::vector<cv::Point2f> considered_points1;
        //std::vector<cv::Point2f> considered_points2;

	//considered_points1.push_back(undistort_points1[i]);
	//considered_points2.push_back(undistort_points2[i]);


	//std::cout<<considered_points1[0]<<"-->"<<considered_points2[0]<<std::endl;
	//std::cout<<std::endl;
	
	//std::cout<<"ConsideredPoints2"<<std::endl;
	//std::cout<<considered_points1[i];}


	cv::triangulatePoints(P1,P2_1,considered_points1,considered_points2,pnts3D1);
	cv::triangulatePoints(P1,P2_2,considered_points1,considered_points2,pnts3D2);
	cv::triangulatePoints(P1,P2_3,considered_points1,considered_points2,pnts3D3);
	cv::triangulatePoints(P1,P2_4,considered_points1,considered_points2,pnts3D4);
	
	cv::Mat trans3D1;
	cv::Mat trans3D2;
	cv::Mat trans3D3;
	cv::Mat trans3D4;
	cv::Mat c3D1 = convert_channel_mat(pnts3D1);
	cv::Mat c3D2 = convert_channel_mat(pnts3D2);
	cv::Mat c3D3 = convert_channel_mat(pnts3D3);
	cv::Mat c3D4 = convert_channel_mat(pnts3D4);
	
	trans3D1 = rigid_transform(candidateR1,candidateT1,c3D1);
	trans3D2 = rigid_transform(candidateR1,candidateT2,c3D2);
	trans3D3 = rigid_transform(candidateR2,candidateT1,c3D3);
	trans3D4 = rigid_transform(candidateR2,candidateT2,c3D4);
	std::cout<<"\nPoints\n";
        std::cout<<c3D1<<std::endl;
	std::cout<<"\n Transformed Points\n";
	std::cout<<trans3D1<<std::endl;

	for(int i=0;i<points1.size();i++){
	std::cout<<"3D Points1\n";//<<pnts3D1<<std::endl;
	//std::cout<<"w:"<<pnts3D1.at<Vec4f>(0,0)[3];
	float w1 = pnts3D1.at<Vec4f>(0,i)[3];
	float x1 = pnts3D1.at<Vec4f>(0,i)[0]/w1;
	float y1 = pnts3D1.at<Vec4f>(0,i)[1]/w1;
 	float z1 = pnts3D1.at<Vec4f>(0,i)[2]/w1;
	float trans_z1 = trans3D1.at<float>(2,i);

	std::cout<<"["<<x1<<","<<y1<<","<<z1<<"]"<<std::endl;
	std::cout<<"Trans1 Depth :"<<trans_z1;

	//std::cout<<"Point1  :"<<pnts3D1.at<Vec4f>(0,i);
	//std::cout<<"Point1 T" <<trans3D1.at<float>
	
//	std::cout<<"3D Points2\n";//<<pnts3D2<<std::endl;
	float w2 = pnts3D2.at<Vec4f>(0,i)[3];
        float x2 = pnts3D2.at<Vec4f>(0,i)[0]/w2;
        float y2 = pnts3D2.at<Vec4f>(0,i)[1]/w2;
        float z2 = pnts3D2.at<Vec4f>(0,i)[2]/w2;
	float trans_z2 = trans3D2.at<float>(2,i);
	std::cout<<"["<<x2<<","<<y2<<","<<z2<<"]"<<std::endl;
	
//	std::cout<<"3D Points3\n";//<<pnts3D3<<std::endl;
	float w3 = pnts3D3.at<Vec4f>(0,i)[3];
        float x3 = pnts3D3.at<Vec4f>(0,i)[0]/w3;
        float y3 = pnts3D3.at<Vec4f>(0,i)[1]/w3;
        float z3 = pnts3D3.at<Vec4f>(0,i)[2]/w3;
	float trans_z3 = trans3D3.at<float>(2,i);
        std::cout<<"["<<x3<<","<<y3<<","<<z3<<"]"<<std::endl;
	
//	std::cout<<"3D Points4\n";//<<pnts3D4<<std::endl;

	float w4 = pnts3D4.at<Vec4f>(0,i)[3];
        float x4 = pnts3D4.at<Vec4f>(0,i)[0]/w4;
        float y4 = pnts3D4.at<Vec4f>(0,i)[1]/w4;
        float z4 = pnts3D4.at<Vec4f>(0,i)[2]/w4;
	float trans_z4 = trans3D4.at<float>(2,i);
        std::cout<<"["<<x4<<","<<y4<<","<<z4<<"]"<<std::endl;
	
	//for(int j=0;j<points4d.size();j++){
		//std::cout<<"Point no: "<<j<<" ["<<points4d[j].at<float>(0)<<", "<<points4d[j].at<float>(0,1)<<", "<<points4d[j].at<float>(0,2)<<", "<<points4d[j].at<float>(0,3)<<"] \n";
	//	std::cout<<"Point"<<points4d[j]<<std::endl;
	

	if(z1<0||trans_z1<0){
		case1_neg++;
	}
     if(z2<0 || trans_z2<0){
                case2_neg++;
        }

     if(z3<0 ||trans_z3<0){
                case3_neg++;
        }

     if(z4<0 || trans_z4<0 ){
                case4_neg++;
        }

}
//}
std::cout<<"Num neg depths 1: "<<case1_neg<<std::endl;	
std::cout<<"Num neg depths 2: "<<case2_neg<<std::endl;	
std::cout<<"Num neg depths 3: "<<case3_neg<<std::endl;	
std::cout<<"Num neg depths 4: "<<case4_neg<<std::endl;


int depths[4];
depths[0]=case1_neg ;
depths[1]=case2_neg;
depths[2]=case3_neg ;
depths[3]=case4_neg ;

int index=find_minimum_depth_index(depths,4);
	
	cout << "\nusing depth :: " << index ;
	
	if(index == 0){
		validR=candidateR1;
		validT=candidateT1;

		cout << "valid R" << validR ;
	}else if (index == 1){

		validR=candidateR1;
		validT=candidateT2;

		cout << "valid R" << validR ;
	}else if (index ==2){

		validR=candidateR2;
		validT=candidateT1;

		cout << "valid R" << validR ;
	} else if(index== 3)
	{

        	validR=candidateR2;
        	validT=candidateT2 ;	
		cout << "valid R" << validR ;
	}


	
}


static void validateRandT2(cv::Mat K1,cv::Mat K2,std::vector<float> distCoeff,cv::Mat candidateR1,cv::Mat candidateR2,cv::Mat candidateT1,cv::Mat candidateT2,cv::Mat img1,  cv::Mat img2,
        std::vector<cv::Point2f> points1,
        std::vector<cv::Point2f> points2,cv::Mat &validR, cv::Mat &validT ){

	for(int j=0; j < points1.size() ; j++){
		std::cout<<points1[j]<<"-->"<<points2[j]<<std::endl;
	}
	std::cout<<"Points1\n"<<points1<<std::endl;
	std::cout<<"Points2\n"<<points2<<std::endl;

	std::vector<cv::Point2f> undistort_points2;
	std::vector<cv::Point2f> undistort_points1;
	std::vector<float> zerodist;

	cv::undistortPoints(points1,undistort_points1,K1,zerodist);
	cv::undistortPoints(points2,undistort_points2,K1,zerodist);
	//points1 = undistort_points1;
	//points2 = undistort_points2;
	std::cout<<"Vaildating R and T \n";
	cv::Mat T0(3,1,CV_32F);
	T0.at<float>(0,0)=0;
	T0.at<float>(0,1)=0;
	T0.at<float>(0,2)=0;


	Mat identity_m=cv::Mat::eye(3,3,CV_32FC1);


	cv::Mat P1 = ConstructP(cv::Mat::eye(3,3,CV_32FC1),T0,identity_m);
	std::cout<<"P1 \n"<<P1<<std::endl;
	cv::Mat P2_1 = ConstructP(candidateR1,candidateT1,identity_m);

	std::cout<<"P2_1 \n"<<P2_1<<std::endl;
	cv::Mat P2_2 = ConstructP(candidateR1,candidateT2,identity_m);
	cv::Mat P2_3 = ConstructP(candidateR2,candidateT1,identity_m);
	cv::Mat P2_4 = ConstructP(candidateR2,candidateT2,identity_m);

	int N =points1.size();
	std::vector<cv::Point2f> considered_points1(points1.begin(),points1.begin()+N);
	std::vector<cv::Point2f> considered_points2(points2.begin(),points2.begin()+N);
	std::vector<cv::Vec4f> points4d;
	points4d.resize(N);
	cv::Mat pnts3D1(1,N,CV_32FC4);
	cv::Mat pnts3D2(1,N,CV_32FC4);
	cv::Mat pnts3D3(1,N,CV_32FC4);
	cv::Mat pnts3D4(1,N,CV_32FC4);
	//cv::Mat cam0pnts(1,N,CV_64FC2);
	//cv::Mat cam1pnts(1,N,CV_64FC2);

	std::cout<<"ConsideredPoints"<<std::endl;
	int case1_neg =0;
	int case2_neg =0;
	int case3_neg =0;
        int case4_neg =0;



        //std::vector<cv::Point2f> considered_points1;
        //std::vector<cv::Point2f> considered_points2;

	//considered_points1.push_back(undistort_points1[i]);
	//considered_points2.push_back(undistort_points2[i]);


	//std::cout<<considered_points1[0]<<"-->"<<considered_points2[0]<<std::endl;
	//std::cout<<std::endl;

	//std::cout<<"ConsideredPoints2"<<std::endl;
	//std::cout<<considered_points1[i];}


        //std::vector<cv::Point2f> considered_points1;
        //std::vector<cv::Point2f> considered_points2;

        //considered_points1.push_back(undistort_points1[i]);
        //considered_points2.push_back(undistort_points2[i]);

        int no_of_considered_points=considered_points1.size();
//      for(int j=0; j < no_of_considered_points ; j++){
  //              std::cout<<considered_points1[j]<<"-->"<<considered_points2[j]<<std::endl;
    //    }

        Mat c3D1(4,no_of_considered_points,CV_32F);
        vector<Point2f> correspImg1Pt1;
        cout << "triangulation1" << endl ;
        triangulatePoints(considered_points1,considered_points2, K1.inv(),  P1, P2_1,c3D1, correspImg1Pt1);
        cout << c3D1 << endl ;
        cout << "triangulation2" << endl ;
        Mat c3D2(4,no_of_considered_points,CV_32F);
        vector<Point2f> correspImg1Pt2;
        triangulatePoints(considered_points1,considered_points2, K1.inv(),  P1, P2_2,c3D2, correspImg1Pt2);

        cout << "triangulation3" << endl ;
        Mat c3D3(4,no_of_considered_points,CV_32F);
        vector<Point2f> correspImg1Pt3;
        triangulatePoints(considered_points1,considered_points2, K1.inv(),  P1, P2_3,c3D3, correspImg1Pt3);

       cout << "triangulation4" << endl ;

        Mat c3D4(4,no_of_considered_points,CV_32F);
        vector<Point2f> correspImg1Pt4;
        triangulatePoints(considered_points1,considered_points2, K1.inv(),  P1, P2_4,c3D4, correspImg1Pt4);

        cout << "Triangulation done!!" << endl ;
               cv::Mat trans3D1;
               cv::Mat trans3D2;
               cv::Mat trans3D3;
               cv::Mat trans3D4;

       //      cv::Mat c3D1_test = convert_channel_mat(pnts3D1);
       //      cout << "Size of c3D1_test" << c3D1_test.size() << endl ;
       //      cout << "Size of c3D1" << c3D1.size() << endl ;
       //      cv::Mat c3D2 = convert_channel_mat(pnts3D2);
       //      cv::Mat c3D3 = convert_channel_mat(pnts3D3);
       //      cv::Mat c3D4 = convert_channel_mat(pnts3D4);

               trans3D1 = rigid_transform(candidateR1,candidateT1,c3D1);
               trans3D2 = rigid_transform(candidateR1,candidateT2,c3D2);
               trans3D3 = rigid_transform(candidateR2,candidateT1,c3D3);
               trans3D4 = rigid_transform(candidateR2,candidateT2,c3D4);
       //      std::cout<<"\nPoints\n";
         //      std::cout<<c3D1<<std::endl;
       //      std::cout<<"\n Transformed Points\n";
       //      std::cout<<trans3D1<<std::endl;

              for(int i=0;i<points1.size();i++){

               float w1 = c3D1.at<float>(3,i);
               float x1 = c3D1.at<float>(0,i)/w1;
               float y1 = c3D1.at<float>(1,i)/w1;
               float z1 = c3D1.at<float>(2,i)/w1;
               float trans_z1 = trans3D1.at<float>(2,i);

       //      std::cout<<"["<<x1<<","<<y1<<","<<z1<<"]"<<std::endl;
       //      std::cout<<"Trans1 Depth :"<<trans_z1;


               float w2 = c3D2.at<float>(3,i);

                float x2 = c3D2.at<float>(0,i)/w2;
               float y2 = c3D2.at<float>(1,i)/w2;
               float z2 = c3D2.at<float>(2,i)/w2;
               float trans_z2 = trans3D2.at<float>(2,i);

             //  cout << "w2:" << w2 << endl ;
       //      std::cout<<"["<<x2<<","<<y2<<","<<z2<<"]"<<std::endl;

       //      std::cout<<"3D Points3\n";//<<pnts3D3<<std::endl;
               float w3 = c3D3.at<float>(3,i);
               float x3 = c3D3.at<float>(0,i)/w3;
               float y3 = c3D3.at<float>(1,i)/w3;
               float z3 = c3D3.at<float>(2,i)/w3;
               float trans_z3 = trans3D3.at<float>(2,i);
                //      std::cout<<"["<<x3<<","<<y3<<","<<z3<<"]"<<std::endl;

              //      std::cout<<"3D Points4\n";//<<pnts3D4<<std::endl;

                float w4 = c3D4.at<float>(3,i);
                float x4 = c3D4.at<float>(0,i)/w4;
                float y4 = c3D4.at<float>(1,i)/w4;
                float z4 = c3D4.at<float>(2,i)/w4;
                float trans_z4 = trans3D4.at<float>(2,i);
                //      std::cout<<"["<<x4<<","<<y4<<","<<z4<<"]"<<std::endl;

                      //for(int j=0;j<points4d.size();j++){
                              //std::cout<<"Point no: "<<j<<" ["<<points4d[j].at<float>(0)<<", "<<points4d[j].at<float>(0,1)<<", "<<points4d[j].at<float>(0,2)<<", "<<points4d[j].at<float>(0,3)<<"] \n";
                      //      std::cout<<"Point"<<points4d[j]<<std::endl;


                      if(z1<0||trans_z1<0){
                              case1_neg++;
                      }
                   if(z2<0 || trans_z2<0){
                              case2_neg++;
                      }

                   if(z3<0 ||trans_z3<0){
                              case3_neg++;
                      }

                   if(z4<0 || trans_z4<0 ){
                              case4_neg++;
                      }

}
//}
std::cout<<"Num neg depths 1: "<<case1_neg<<std::endl;
std::cout<<"Num neg depths 2: "<<case2_neg<<std::endl;
std::cout<<"Num neg depths 3: "<<case3_neg<<std::endl;
std::cout<<"Num neg depths 4: "<<case4_neg<<std::endl;


int depths[4];
depths[0]=case1_neg ;
depths[1]=case2_neg;
depths[2]=case3_neg ;
depths[3]=case4_neg ;

int index=find_minimum_depth_index(depths,4);

	cout << "\nusing depth :: " << index ;

	if(index == 0){
		validR=candidateR1;
		validT=candidateT1;

		cout << "valid R" << validR ;
	}else if (index == 1){

		validR=candidateR1;
		validT=candidateT2;

		cout << "valid R" << validR ;
	}else if (index ==2){

		validR=candidateR2;
		validT=candidateT1;

		cout << "valid R" << validR ;
	} else if(index== 3)
	{

        	validR=candidateR2;
        	validT=candidateT2 ;
		cout << "valid R" << validR ;
	}



}


static cv::Mat convert_channel_mat(cv::Mat channel_mat){
int s=channel_mat.cols;
cv::Mat c(4,s,CV_32FC1);
 for(int i=0;i<s;i++){
        float w1 = channel_mat.at<Vec4f>(0,i)[3];
        float x1 = channel_mat.at<Vec4f>(0,i)[0];
        float y1 = channel_mat.at<Vec4f>(0,i)[1];
        float z1 = channel_mat.at<Vec4f>(0,i)[2];

	c.at<float>(0,i) = x1;
	c.at<float>(1,i) = y1;
	c.at<float>(2,i) = z1;
	c.at<float>(3,i) = w1;
	

}
return c;
}
static cv::Mat rigid_transform(cv::Mat R, cv::Mat T, cv::Mat points4f){

cv::Mat P;
cv::Mat transformed_points4f;
P = ConstructP(R,T,cv::Mat::eye(3,3,CV_32FC1));



return P*points4f;

}

static cv::Mat ConstructP(cv::Mat R, cv::Mat T, cv::Mat K)
{
  cv::Mat P(3,4,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      P.at<float>(i,j) = R.at<float>(i,j);
      }
    }

  for(unsigned int row = 0; row < 3; row++)
    {
    P.at<float>(row,3) = T.at<float>(row,0);
    }


  P = K * P;

  return P;
}





