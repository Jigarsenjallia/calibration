#include "cv.h"
#include "highgui.h"
#include <string>
#include <iostream>

using namespace std;

int main()
{
	int cube_length=7;

	CvCapture* capture;

	capture=cvCreateCameraCapture(0);

	if(capture==0){
		printf("�޷���������ͷ�豸��\n\n");
		return 0;
	}else{
		printf("��������ͷ�豸�ɹ�����\n\n");
	}

	IplImage* frame;

	cvNamedWindow("�����֡��ȡ����",1);

	printf("�����ո������ȡ��ǰ֡������Ϊ�궨ͼƬ...\n����Q�����˳���ȡ֡����...\n\n");

	int number_image=1;
	char *str1;
	str1=".jpg";
	char filename[20]="";

	while(true)
	{
		frame=cvQueryFrame(capture);
		if(!frame)
			 break;
		cvShowImage("�����֡��ȡ����",frame);

		if(cvWaitKey(10)==' '){
			sprintf(filename,"%d.jpg",number_image);
			cvSaveImage(filename,frame);
			cout<<"�ɹ���ȡ��ǰ֡�������ļ���"<<filename<<"����...\n\n";
			printf("�����ո������ȡ��ǰ֡������Ϊ�궨ͼƬ...\n����Q�����˳���ȡ֡����...\n\n");
			number_image++;
		}else if(cvWaitKey(10)=='ESC_key'){
			printf("��ȡͼ��֡�������...\n\n");
			cout<<"���ɹ���ȡ"<<--number_image<<"֡ͼ�񣡣�\n\n";
			break;
		}
	}

	cvReleaseImage(&frame);
	cvDestroyWindow("�����֡��ȡ����");

	IplImage * show;
	cvNamedWindow("RePlay",1);

	int a=1;
	int number_image_copy=number_image;

	CvSize board_size=cvSize(10,10);
	int board_width=board_size.width;
	int board_height=board_size.height;
	int total_per_image=board_width*board_height;
	CvPoint2D32f* image_points_buf = new CvPoint2D32f[total_per_image];
	CvMat* image_points=cvCreateMat(number_image*total_per_image,2,CV_32FC1);
	CvMat* object_points=cvCreateMat(number_image*total_per_image,3,CV_32FC1);
	CvMat* point_counts=cvCreateMat(number_image,1,CV_32SC1);
	CvMat* intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat* distortion_coeffs=cvCreateMat(5,1,CV_32FC1);

	int count;
	int found;
	int step;
	int successes=0;

	while(a<=number_image_copy){
		sprintf (filename,"%d.jpg",a);
		show=cvLoadImage(filename,-1);

		found=cvFindChessboardCorners(show,board_size,image_points_buf,&count,
									 CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
		if(found==0){
			cout<<"��"<<a<<"֡ͼƬ�޷��ҵ����̸����нǵ�!\n\n";
			cvNamedWindow("RePlay",1);
			cvShowImage("RePlay",show);
			cvWaitKey(0);

		}else{
			cout<<"��"<<a<<"֡ͼ��ɹ����"<<count<<"���ǵ�...\n";

			cvNamedWindow("RePlay",1);

			IplImage * gray_image= cvCreateImage(cvGetSize(show),8,1);
			cvCvtColor(show,gray_image,CV_BGR2GRAY);
			cout<<"��ȡԴͼ��Ҷ�ͼ�������...\n";
			cvFindCornerSubPix(gray_image,image_points_buf,count,cvSize(11,11),cvSize(-1,-1),
								cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			cout<<"�Ҷ�ͼ�����ػ��������...\n";
			cvDrawChessboardCorners(show,board_size,image_points_buf,count,found);
			cout<<"��Դͼ���ϻ��ƽǵ�������...\n\n";
			cvShowImage("RePlay",show);



			cvWaitKey(0);
		}

		if(total_per_image==count){
			step=successes*total_per_image;
			for(int i=step,j=0;j<total_per_image;++i,++j){
				CV_MAT_ELEM(*image_points,float,i,0)=image_points_buf[j].x;
				CV_MAT_ELEM(*image_points,float,i,1)=image_points_buf[j].y;
				CV_MAT_ELEM(*object_points,float,i,0)=(float)(j/cube_length);
				CV_MAT_ELEM(*object_points,float,i,1)=(float)(j%cube_length);
				CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
			}
			CV_MAT_ELEM(*point_counts,int,successes,0)=total_per_image;
			successes++;
		}
		a++;
	}

	cvReleaseImage(&show);
	cvDestroyWindow("RePlay");


	cout<<"*********************************************\n";
	cout<<number_image<<"֡ͼƬ�У��궨�ɹ���ͼƬΪ"<<successes<<"֡...\n";
	cout<<number_image<<"֡ͼƬ�У��궨ʧ�ܵ�ͼƬΪ"<<number_image-successes<<"֡...\n\n";
	cout<<"*********************************************\n\n";

	cout<<"���������ʼ����������ڲ���...\n\n";


	CvCapture* capture1;
	capture1=cvCreateCameraCapture(0);
	IplImage * show_colie;
	show_colie=cvQueryFrame(capture1);

	CvMat * object_points2=cvCreateMat(successes*total_per_image,3,CV_32FC1);
	CvMat * image_points2=cvCreateMat(successes*total_per_image,2,CV_32FC1);
	CvMat * point_counts2=cvCreateMat(successes,1,CV_32SC1);


	for(int i=0;i<successes*total_per_image;++i){
		CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
		CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
		CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);
	}

	for(int i=0;i<successes;++i){
		CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);


	CV_MAT_ELEM(*intrinsic_matrix,float,0,0)=1.0f;
	CV_MAT_ELEM(*intrinsic_matrix,float,1,1)=1.0f;

	cvCalibrateCamera2(object_points2,image_points2,point_counts2,cvGetSize(show_colie),
						intrinsic_matrix,distortion_coeffs,NULL,NULL,0);


	cout<<"������ڲ�������Ϊ��\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,1)
															<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,2)
															<<"\n\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,1)
															<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,2)
															<<"\n\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,2,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,1)
															<<"          "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,2)
															<<"\n\n";

	cout<<"����ϵ������Ϊ��\n";
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,0,0)<<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,1,0)
															<<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,2,0)
															<<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,3,0)
															<<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,4,0)
															<<"\n\n";

	cvSave("Intrinsics.xml",intrinsic_matrix);
	cvSave("Distortion.xml",distortion_coeffs);

	cout<<"��������󡢻���ϵ�������Ѿ��ֱ�洢����ΪIntrinsics.xml��Distortion.xml�ĵ���\n\n";

	CvMat * intrinsic=(CvMat *)cvLoad("Intrinsics.xml");
	CvMat * distortion=(CvMat *)cvLoad("Distortion.xml");

	IplImage * mapx=cvCreateImage(cvGetSize(show_colie),IPL_DEPTH_32F,1);
	IplImage * mapy=cvCreateImage(cvGetSize(show_colie),IPL_DEPTH_32F,1);

	cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

	cvNamedWindow("ԭʼͼ��",1);
	cvNamedWindow("�ǻ���ͼ��",1);

	cout<<"����E�����˳���ʾ...\n\n";

	while(show_colie){
		IplImage * clone=cvCloneImage(show_colie);
		cvShowImage("ԭʼͼ��",show_colie);
		cvRemap(clone,show_colie,mapx,mapy);
		cvReleaseImage(&clone);
		cvShowImage("�ǻ���ͼ��",show_colie);

		if(cvWaitKey(10)=='e'){
			break;
		}

		show_colie=cvQueryFrame(capture1);
	}

	return 0;

}
